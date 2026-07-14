using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation.Robots.G1
{
    [DisallowMultipleComponent]
    public sealed class G1JointController : MonoBehaviour
    {
        public static readonly string[] JointNames =
        {
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
        };

        private static readonly string[] LinkNames = Array.ConvertAll(JointNames, name =>
            name == "waist_pitch_joint" ? "torso_link" : name.Replace("_joint", "_link"));

        [SerializeField] private ArticulationBody[] joints = new ArticulationBody[29];
        [SerializeField] private float[] targetDegrees = new float[29];
        [SerializeField, Min(0f)] private float stiffness = 8000f;
        [SerializeField, Min(0f)] private float damping = 120f;
        [SerializeField, Min(0f)] private float forceLimit = 600f;
        [SerializeField] private bool anchorPelvis = true;
        [SerializeField] private bool applyContinuously = true;
        [Header("Joint Validation")]
        [SerializeField] private bool keyboardValidationEnabled = true;
        [SerializeField, Min(1f)] private float keyboardSpeedDegreesPerSecond = 35f;
        [SerializeField, Range(0, 28)] private int selectedJointIndex;
        [SerializeField, Range(0f, 1f)] private float highlightStrength = 0.35f;
        [SerializeField] private Color highlightColor = new Color(0.2f, 0.85f, 1f, 1f);

        private readonly Dictionary<Renderer, MaterialPropertyBlock> originalHighlights = new();
        private readonly List<Renderer> selectedRenderers = new();
        private MaterialPropertyBlock highlightBlock;

        public int JointCount => JointNames.Length;
        public int SelectedJointIndex => selectedJointIndex;
        public string SelectedJointName => JointNames[selectedJointIndex];
        public bool KeyboardValidationEnabled
        {
            get => keyboardValidationEnabled;
            set => keyboardValidationEnabled = value;
        }

        public ArticulationBody GetJoint(int index)
        {
            ValidateIndex(index);
            return joints[index];
        }

        public float GetCurrentJointPositionDegrees(int index)
        {
            ValidateIndex(index);
            var body = joints[index];
            return body != null && body.dofCount > 0
                ? body.jointPosition[0] * Mathf.Rad2Deg
                : targetDegrees[index];
        }

        public bool TryGetRootBody(out ArticulationBody root)
        {
            root = GetRootBody();
            return root != null;
        }

        public void MoveBase(Vector3 worldDelta, float yawDeltaDegrees)
        {
            var root = GetRootBody();
            if (root == null)
                return;
            var rotation = Quaternion.AngleAxis(yawDeltaDegrees, Vector3.up) * root.transform.rotation;
            root.TeleportRoot(root.transform.position + worldDelta, rotation);
        }

        public void PlaceBase(Vector3 worldPosition)
        {
            var root = GetRootBody();
            if (root != null)
                root.TeleportRoot(worldPosition, root.transform.rotation);
        }

        private void Awake()
        {
            BindJoints();
            ConfigureRoot();
            ApplyTargets();
            highlightBlock = new MaterialPropertyBlock();
            RefreshHighlight();
        }

        private void Update()
        {
            if (!keyboardValidationEnabled || Keyboard.current == null)
                return;

            if (Keyboard.current.leftArrowKey.wasPressedThisFrame)
                SelectJoint((selectedJointIndex + JointCount - 1) % JointCount);
            if (Keyboard.current.rightArrowKey.wasPressedThisFrame)
                SelectJoint((selectedJointIndex + 1) % JointCount);
            if (Keyboard.current.homeKey.wasPressedThisFrame)
                ResetNeutralPose();

            var direction = 0f;
            if (Keyboard.current.upArrowKey.isPressed)
                direction += 1f;
            if (Keyboard.current.downArrowKey.isPressed)
                direction -= 1f;
            if (!Mathf.Approximately(direction, 0f))
            {
                SetTargetDegrees(selectedJointIndex,
                    GetTargetDegrees(selectedJointIndex) + direction * keyboardSpeedDegreesPerSecond * Time.deltaTime);
                ApplyTargets();
            }
        }

        private void FixedUpdate()
        {
            if (applyContinuously)
                ApplyTargets();
        }

        public void BindJoints()
        {
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            var byName = new Dictionary<string, ArticulationBody>(StringComparer.Ordinal);
            foreach (var body in bodies)
                byName[body.name] = body;

            Array.Resize(ref joints, JointCount);
            Array.Resize(ref targetDegrees, JointCount);
            for (var i = 0; i < JointCount; i++)
                byName.TryGetValue(LinkNames[i], out joints[i]);
        }

        public void SetTargetDegrees(int index, float degrees)
        {
            ValidateIndex(index);
            var body = joints[index];
            if (body == null)
                return;

            var drive = body.xDrive;
            targetDegrees[index] = Mathf.Clamp(degrees, drive.lowerLimit, drive.upperLimit);
        }

        public float GetTargetDegrees(int index)
        {
            ValidateIndex(index);
            return targetDegrees[index];
        }

        public void SelectJoint(int index)
        {
            ValidateIndex(index);
            if (selectedJointIndex == index && selectedRenderers.Count > 0)
                return;

            RestoreHighlight();
            selectedJointIndex = index;
            RefreshHighlight();
        }

        public void ResetNeutralPose()
        {
            Array.Clear(targetDegrees, 0, targetDegrees.Length);
            ApplyTargets();
        }

        public void ApplyTargets()
        {
            for (var i = 0; i < JointCount; i++)
            {
                var body = joints[i];
                if (body == null)
                    continue;

                var drive = body.xDrive;
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                drive.target = Mathf.Clamp(targetDegrees[i], drive.lowerLimit, drive.upperLimit);
                body.xDrive = drive;
            }
        }

        private void ConfigureRoot()
        {
            var body = GetRootBody();
            if (body != null)
                body.immovable = anchorPelvis;
        }

        private ArticulationBody GetRootBody()
        {
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
                if (body.isRoot) return body;
            return null;
        }

        private void RefreshHighlight()
        {
            if (highlightBlock == null || joints[selectedJointIndex] == null)
                return;

            var visuals = joints[selectedJointIndex].transform.Find("Visuals");
            if (visuals == null)
                return;

            foreach (var renderer in visuals.GetComponentsInChildren<Renderer>(true))
            {
                var original = new MaterialPropertyBlock();
                renderer.GetPropertyBlock(original);
                originalHighlights[renderer] = original;

                var baseColor = renderer.sharedMaterial != null ? renderer.sharedMaterial.color : Color.white;
                highlightBlock.Clear();
                highlightBlock.SetColor("_BaseColor", Color.Lerp(baseColor, highlightColor, highlightStrength));
                renderer.SetPropertyBlock(highlightBlock);
                selectedRenderers.Add(renderer);
            }
        }

        private void RestoreHighlight()
        {
            foreach (var renderer in selectedRenderers)
            {
                if (renderer != null && originalHighlights.TryGetValue(renderer, out var block))
                    renderer.SetPropertyBlock(block);
            }
            selectedRenderers.Clear();
            originalHighlights.Clear();
        }

        private void OnGUI()
        {
            if (!keyboardValidationEnabled)
                return;

            var body = joints[selectedJointIndex];
            var drive = body != null ? body.xDrive : default;
            var currentDegrees = body != null && body.dofCount > 0
                ? body.jointPosition[0] * Mathf.Rad2Deg
                : 0f;
            var group = selectedJointIndex < 6 ? "LEFT LEG"
                : selectedJointIndex < 12 ? "RIGHT LEG"
                : selectedJointIndex < 15 ? "WAIST"
                : selectedJointIndex < 22 ? "LEFT ARM"
                : "RIGHT ARM";

            GUI.Box(new Rect(18f, 18f, 500f, 116f), string.Empty);
            GUI.Label(new Rect(32f, 28f, 470f, 24f),
                $"G1 JOINT VALIDATION  |  {selectedJointIndex + 1:D2}/{JointCount}  |  {group}");
            GUI.Label(new Rect(32f, 52f, 470f, 24f),
                $"{SelectedJointName}   Target: {targetDegrees[selectedJointIndex]:F1}°   Measured: {currentDegrees:F1}°");
            GUI.Label(new Rect(32f, 76f, 470f, 22f),
                $"Limits: {drive.lowerLimit:F1}° to {drive.upperLimit:F1}°");
            GUI.Label(new Rect(32f, 100f, 470f, 22f),
                "←/→ select joint    ↑/↓ move joint    Home reset neutral pose");
        }

        private static void ValidateIndex(int index)
        {
            if (index < 0 || index >= JointNames.Length)
                throw new ArgumentOutOfRangeException(nameof(index));
        }
    }

    internal static class G1TestSceneCamera
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void FrameRobotForDesktopTest()
        {
            if (UnityEngine.SceneManagement.SceneManager.GetActiveScene().name != "G1ImportTest")
                return;

            var robot = UnityEngine.Object.FindFirstObjectByType<G1JointController>();
            var camera = Camera.main;
            if (robot == null || camera == null)
                return;

            var renderers = robot.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length == 0)
                return;

            var bounds = renderers[0].bounds;
            for (var i = 1; i < renderers.Length; i++)
                bounds.Encapsulate(renderers[i].bounds);

            camera.transform.position = new Vector3(bounds.center.x, bounds.center.y, bounds.min.z - 2f);
            camera.transform.rotation = Quaternion.identity;
        }
    }
}
