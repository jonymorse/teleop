using System;
using System.Collections.Generic;
using UnityEngine;

namespace Teleoperation.Robots.G1
{
    /// <summary>Simulation representation and rate-limited control of Unitree Dex3-1 hands.</summary>
    [DisallowMultipleComponent]
    public sealed class G1Dex3HandController : MonoBehaviour
    {
        [SerializeField, Min(30f)] private float maximumJointSpeedDegrees = 560f;
        [SerializeField, Range(0f, 1f)] private float curlSmoothing = 0.28f;
        [SerializeField, Range(0.05f, 0.5f)] private float thumbstickOverrideDeadzone = 0.2f;

        private readonly float[] commandRadians = new float[14];
        private readonly float[] targetRadians = new float[14];
        private readonly float[] contactHoldRadians = new float[14];
        private readonly List<DexJoint> joints = new(14);
        private Material handMaterial;
        private bool active;
        private bool available;
        private bool leftContactHold;
        private bool rightContactHold;
        private string inputSource = "tracking Quest fingers";

        public bool Active => active;
        public bool Available => available;
        public string Status => !available ? "Dex3 meshes unavailable" : active ? inputSource : "paused";
        public float[] CommandRadians => commandRadians;

        private void Start()
        {
            handMaterial = FindHandMaterial();
            var leftMount = FindDescendant("left_rubber_hand");
            var rightMount = FindDescendant("right_rubber_hand");
            if (leftMount == null || rightMount == null)
                return;

            DisableFixedHandGeometry(leftMount);
            DisableFixedHandGeometry(rightMount);
            var left = BuildHand(leftMount, true, 0);
            var right = BuildHand(rightMount, false, 7);
            available = left && right && joints.Count == 14;
            SetOpenTargets();
            Array.Copy(targetRadians, commandRadians, commandRadians.Length);
            ApplyCommands();
        }

        private void FixedUpdate()
        {
            if (!available)
                return;
            for (var i = 0; i < commandRadians.Length; i++)
                commandRadians[i] = Mathf.MoveTowards(commandRadians[i], targetRadians[i],
                    maximumJointSpeedDegrees * Mathf.Deg2Rad * Time.fixedDeltaTime);
            ApplyCommands();
        }

        public void SetActive(bool value)
        {
            active = value && available;
            if (!active)
            {
                EndContactHold(true);
                EndContactHold(false);
            }
        }

        public void BeginContactHold(bool left)
        {
            if (!available)
                return;
            var offset = left ? 0 : 7;
            for (var i = offset; i < offset + 7; i++)
            {
                contactHoldRadians[i] = commandRadians[i];
                targetRadians[i] = commandRadians[i];
            }
            if (left) leftContactHold = true;
            else rightContactHold = true;
        }

        public void EndContactHold(bool left)
        {
            if (left) leftContactHold = false;
            else rightContactHold = false;
        }

        public void RelieveContact(bool left, float degrees)
        {
            if (!(left ? leftContactHold : rightContactHold))
                return;
            var offset = left ? 0 : 7;
            var amount = Mathf.Clamp(degrees, 0f, 3f) * Mathf.Deg2Rad;
            for (var i = offset; i < offset + 7; i++)
            {
                var open = OpenRadians(i);
                contactHoldRadians[i] = Mathf.MoveTowards(contactHoldRadians[i], open, amount);
                targetRadians[i] = Mathf.MoveTowards(targetRadians[i], contactHoldRadians[i], amount);
            }
        }

        public void SetTrackedCurls(Vector3 leftCurls, Vector3 rightCurls)
        {
            if (!active || !available)
                return;
            inputSource = "tracking Quest fingers";
            SetHandTargets(true, 0, leftCurls);
            SetHandTargets(false, 7, rightCurls);
        }

        public void SetControllerInputs(Vector2 leftThumbstick, float leftIndex, float leftMiddle,
            Vector2 rightThumbstick, float rightIndex, float rightMiddle)
        {
            if (!active || !available)
                return;
            inputSource = "controller thumbsticks + triggers";
            SetControllerHandTargets(true, 0, leftThumbstick, leftIndex, leftMiddle);
            SetControllerHandTargets(false, 7, rightThumbstick, rightIndex, rightMiddle);
        }

        public void SetRetargetedRadians(float[] radians)
        {
            if (!active || !available || radians == null || radians.Length != targetRadians.Length)
                return;
            inputSource = "Unitree Quest-hand retargeting";
            for (var i = 0; i < targetRadians.Length; i++)
                targetRadians[i] = radians[i];
        }

        public void OpenHands()
        {
            SetOpenTargets();
            active = false;
        }

        public bool MoveHome(float deltaTime)
        {
            SetOpenTargets();
            var complete = true;
            for (var i = 0; i < commandRadians.Length; i++)
            {
                commandRadians[i] = Mathf.MoveTowards(commandRadians[i], targetRadians[i],
                    maximumJointSpeedDegrees * Mathf.Deg2Rad * deltaTime);
                complete &= Mathf.Abs(commandRadians[i] - targetRadians[i]) < 0.002f;
            }
            ApplyCommands();
            return complete;
        }

        private void SetHandTargets(bool left, int offset, Vector3 curls)
        {
            var blend = 1f - Mathf.Pow(1f - curlSmoothing, Time.deltaTime * 90f);
            var thumb = Mathf.Clamp01(curls.x);
            var index = Mathf.Clamp01(curls.y);
            var middle = Mathf.Clamp01(curls.z);
            var sign = left ? 1f : -1f;
            SetSmoothed(offset + 0, Mathf.Lerp(-12f * sign, 28f * sign, thumb) * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 1, Mathf.Lerp(-18f * sign, 50f * sign, thumb) * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 2, 100f * sign * thumb * Mathf.Deg2Rad, blend);
            var fingerSign = left ? -1f : 1f;
            SetSmoothed(offset + 3, 90f * fingerSign * middle * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 4, 100f * fingerSign * middle * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 5, 90f * fingerSign * index * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 6, 100f * fingerSign * index * Mathf.Deg2Rad, blend);
        }

        private void SetControllerHandTargets(bool left, int offset, Vector2 thumbstick,
            float indexInput, float middleInput)
        {
            var blend = 1f - Mathf.Pow(1f - curlSmoothing, Time.deltaTime * 90f);
            var index = Mathf.Clamp01(indexInput);
            var middle = Mathf.Clamp01(middleInput);

            // Controller grasp synergy: trigger forms a thumb/index pinch and grip forms a
            // three-finger power grasp. Moving either thumbstick axis outside the deadzone
            // overrides that thumb degree of freedom for precise manual adjustment.
            var powerIndex = middle * 0.72f;
            index = Mathf.Max(index, powerIndex);
            var synergy = Mathf.Max(index, middle);
            var manualInward = Mathf.Clamp(left ? thumbstick.x : -thumbstick.x, -1f, 1f);
            var inward = Mathf.Abs(thumbstick.x) > thumbstickOverrideDeadzone
                ? manualInward : Mathf.Lerp(-0.25f, 0.82f, synergy);
            var thumbCurl = Mathf.Abs(thumbstick.y) > thumbstickOverrideDeadzone
                ? Mathf.Clamp01((thumbstick.y - thumbstickOverrideDeadzone) /
                                (1f - thumbstickOverrideDeadzone))
                : Mathf.Lerp(0f, 0.88f, synergy);
            var sign = left ? 1f : -1f;
            SetSmoothed(offset + 0, Mathf.Lerp(-45f, 45f, (inward + 1f) * 0.5f) * sign * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 1, Mathf.Lerp(-18f, 50f, thumbCurl) * sign * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 2, 100f * thumbCurl * sign * Mathf.Deg2Rad, blend);

            var fingerSign = left ? -1f : 1f;
            SetSmoothed(offset + 3, 90f * fingerSign * middle * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 4, 100f * fingerSign * middle * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 5, 90f * fingerSign * index * Mathf.Deg2Rad, blend);
            SetSmoothed(offset + 6, 100f * fingerSign * index * Mathf.Deg2Rad, blend);
        }

        private void SetSmoothed(int index, float value, float blend)
        {
            var next = Mathf.Lerp(targetRadians[index], value, blend);
            if ((index < 7 && leftContactHold) || (index >= 7 && rightContactHold))
                next = contactHoldRadians[index];
            targetRadians[index] = next;
        }

        private void SetOpenTargets()
        {
            for (var i = 0; i < targetRadians.Length; i++)
                targetRadians[i] = OpenRadians(i);
        }

        private static float OpenRadians(int index) => index == 0 ? -12f * Mathf.Deg2Rad
            : index == 1 ? -18f * Mathf.Deg2Rad
            : index == 7 ? 12f * Mathf.Deg2Rad
            : index == 8 ? 18f * Mathf.Deg2Rad : 0f;

        private void ApplyCommands()
        {
            foreach (var joint in joints)
            {
                var radians = Mathf.Clamp(commandRadians[joint.CommandIndex], joint.LowerRadians, joint.UpperRadians);
                joint.Pivot.localRotation = joint.RestRotation *
                                            Quaternion.AngleAxis(radians * Mathf.Rad2Deg, joint.UnityAxis);
            }
            Physics.SyncTransforms();
        }

        private bool BuildHand(Transform mount, bool left, int commandOffset)
        {
            var prefix = left ? "left" : "right";
            var palm = new GameObject($"{prefix}_hand_palm_link").transform;
            palm.SetParent(mount, false);
            if (!AttachMesh(palm, $"{prefix}_hand_palm_link"))
                return false;
            AddBox(palm.gameObject, new Vector3(0f, 0f, 0.055f), new Vector3(0.082f, 0.045f, 0.11f));

            var thumb0 = CreateJoint(palm, $"{prefix}_hand_thumb_0", Ros(0.0255f, 0f, 0f),
                RosAxis(0f, 1f, 0f), -60f, 60f, commandOffset + 0);
            if (!AttachMesh(thumb0, $"{prefix}_hand_thumb_0_link")) return false;
            AddBox(thumb0.gameObject, Vector3.zero, Vector3.one * 0.025f);
            var thumbDirection = left ? 1f : -1f;
            var thumb1 = CreateJoint(thumb0, $"{prefix}_hand_thumb_1",
                Ros(-0.0025f, left ? -0.0193f : 0.0193f, 0f), RosAxis(0f, 0f, 1f),
                left ? -35f : -60f, left ? 60f : 35f, commandOffset + 1);
            if (!AttachMesh(thumb1, $"{prefix}_hand_thumb_1_link")) return false;
            AddBox(thumb1.gameObject, new Vector3(0.023f * thumbDirection, 0f, 0f),
                new Vector3(0.046f, 0.023f, 0.023f));
            var thumb2 = CreateJoint(thumb1, $"{prefix}_hand_thumb_2",
                Ros(0f, left ? -0.0458f : 0.0458f, 0f), RosAxis(0f, 0f, 1f),
                left ? 0f : -100f, left ? 100f : 0f, commandOffset + 2);
            if (!AttachMesh(thumb2, $"{prefix}_hand_thumb_2_link")) return false;
            AddBox(thumb2.gameObject, new Vector3(0.027f * thumbDirection, 0f, 0f),
                new Vector3(0.054f, 0.021f, 0.021f));

            var side = left ? 0.0016f : -0.0016f;
            BuildFinger(palm, prefix, "middle", Ros(0.0777f, side, -0.0285f), left,
                commandOffset + 3, commandOffset + 4);
            BuildFinger(palm, prefix, "index", Ros(0.0777f, side, 0.0285f), left,
                commandOffset + 5, commandOffset + 6);
            return true;
        }

        private void BuildFinger(Transform palm, string prefix, string finger, Vector3 origin, bool left,
            int proximalCommand, int distalCommand)
        {
            var sign = left ? -1f : 1f;
            var first = CreateJoint(palm, $"{prefix}_hand_{finger}_0", origin, RosAxis(0f, 0f, 1f),
                left ? -90f : 0f, left ? 0f : 90f, proximalCommand);
            AttachMesh(first, $"{prefix}_hand_{finger}_0_link");
            AddBox(first.gameObject, new Vector3(0f, 0f, 0.023f), new Vector3(0.024f, 0.024f, 0.046f));
            var second = CreateJoint(first, $"{prefix}_hand_{finger}_1", Ros(0.0458f, 0f, 0f),
                RosAxis(0f, 0f, 1f), left ? -100f : 0f, left ? 0f : 100f, distalCommand);
            AttachMesh(second, $"{prefix}_hand_{finger}_1_link");
            AddBox(second.gameObject, new Vector3(0f, 0f, 0.028f), new Vector3(0.022f, 0.022f, 0.056f));
        }

        private Transform CreateJoint(Transform parent, string name, Vector3 localPosition, Vector3 axis,
            float lowerDegrees, float upperDegrees, int commandIndex)
        {
            var pivot = new GameObject(name + "_joint").transform;
            pivot.SetParent(parent, false);
            pivot.localPosition = localPosition;
            joints.Add(new DexJoint(pivot, axis.normalized, lowerDegrees * Mathf.Deg2Rad,
                upperDegrees * Mathf.Deg2Rad, commandIndex));
            return pivot;
        }

        private bool AttachMesh(Transform parent, string resourceName)
        {
            var prefab = Resources.Load<GameObject>($"Dex3/Meshes/{resourceName}");
            if (prefab == null)
            {
                Debug.LogWarning($"Missing generated Dex3 mesh prefab: {resourceName}", this);
                return false;
            }
            var visual = Instantiate(prefab, parent, false);
            visual.name = resourceName + "_visual";
            foreach (var renderer in visual.GetComponentsInChildren<Renderer>(true))
                if (handMaterial != null) renderer.sharedMaterial = handMaterial;
            return true;
        }

        private Material FindHandMaterial()
        {
            foreach (var renderer in GetComponentsInChildren<Renderer>(true))
                if (renderer.sharedMaterial != null && renderer.sharedMaterial.name.Contains("G1Dark"))
                    return renderer.sharedMaterial;
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            return shader != null ? new Material(shader) { color = new Color(0.035f, 0.045f, 0.055f) } : null;
        }

        private static void DisableFixedHandGeometry(Transform mount)
        {
            foreach (var renderer in mount.GetComponentsInChildren<Renderer>(true)) renderer.enabled = false;
            foreach (var collider in mount.GetComponentsInChildren<Collider>(true))
                if (collider.gameObject.name != "Manipulation Palm Collider") collider.enabled = false;
        }

        private static void AddBox(GameObject target, Vector3 center, Vector3 size)
        {
            var collider = target.AddComponent<BoxCollider>();
            collider.center = center;
            collider.size = size;
            collider.contactOffset = 0.001f;
        }

        private Transform FindDescendant(string objectName)
        {
            foreach (var child in GetComponentsInChildren<Transform>(true))
                if (child.name == objectName) return child;
            return null;
        }

        private static Vector3 Ros(float x, float y, float z) => new(-y, z, x);

        // ROS-to-Unity is a handedness-changing reflection. Positions transform directly,
        // but a rotation axis is an axial vector and therefore needs the additional minus
        // sign. Without it the official Dex3 joint limits flex every finger backward.
        private static Vector3 RosAxis(float x, float y, float z) => new(y, -z, -x);

        private sealed class DexJoint
        {
            public readonly Transform Pivot;
            public readonly Quaternion RestRotation;
            public readonly Vector3 UnityAxis;
            public readonly float LowerRadians;
            public readonly float UpperRadians;
            public readonly int CommandIndex;

            public DexJoint(Transform pivot, Vector3 axis, float lower, float upper, int commandIndex)
            {
                Pivot = pivot;
                RestRotation = pivot.localRotation;
                UnityAxis = axis;
                LowerRadians = lower;
                UpperRadians = upper;
                CommandIndex = commandIndex;
            }
        }
    }
}
