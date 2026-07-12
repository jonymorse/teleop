using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Teleoperation.Robots.UR5e
{
    /// <summary>
    /// Simulator-first joint target adapter for an imported UR5e articulation.
    /// Keeps robot control independent from the URDF importer and future ROS transport.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class UR5eJointController : MonoBehaviour
    {
        public static readonly string[] JointNames =
        {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        // Unity's URDF Importer attaches each ArticulationBody to the joint's child link.
        private static readonly string[] JointBodyNames =
        {
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        };

        [SerializeField] private ArticulationBody[] joints = new ArticulationBody[6];
        [SerializeField] private float[] targetDegrees = new float[6];
        [SerializeField, Min(0f)] private float stiffness = 10000f;
        [SerializeField, Min(0f)] private float damping = 100f;
        [SerializeField, Min(0f)] private float forceLimit = 1000f;
        [SerializeField] private bool applyTargetsContinuously = true;
        [SerializeField] private bool anchorRobotBase = true;
        [SerializeField] private bool keyboardControlEnabled = true;
        [SerializeField, Min(1f)] private float keyboardSpeedDegreesPerSecond = 30f;
        [SerializeField, Range(0, 5)] private int selectedJointIndex;
        [SerializeField] private bool highlightSelectedJoint = true;
        [SerializeField, ColorUsage(false, false)] private Color highlightColor = new Color(1f, 0.55f, 0.05f, 1f);

        private readonly Dictionary<Renderer, MaterialPropertyBlock> originalPropertyBlocks = new();
        private MaterialPropertyBlock highlightPropertyBlock;

        public int JointCount => JointNames.Length;
        public int SelectedJointIndex => selectedJointIndex;
        public string SelectedJointName => JointNames[selectedJointIndex];
        public bool KeyboardControlEnabled
        {
            get => keyboardControlEnabled;
            set => keyboardControlEnabled = value;
        }

        public ArticulationBody GetJoint(int jointIndex)
        {
            ValidateIndex(jointIndex);
            return joints[jointIndex];
        }

        private void Reset()
        {
            BindAndValidateJoints();
        }

        private void Awake()
        {
            highlightPropertyBlock = new MaterialPropertyBlock();
            DisableLegacyUrdfController();
            AnchorBase();
            if (!HasAllJointReferences())
                BindAndValidateJoints();
            ApplyTargets();
            RefreshJointHighlight();
        }

        private void Update()
        {
            if (!keyboardControlEnabled || Keyboard.current == null)
                return;

            if (Keyboard.current.leftArrowKey.wasPressedThisFrame)
                SelectJoint((selectedJointIndex + JointCount - 1) % JointCount);
            if (Keyboard.current.rightArrowKey.wasPressedThisFrame)
                SelectJoint((selectedJointIndex + 1) % JointCount);

            var direction = 0f;
            if (Keyboard.current.upArrowKey.isPressed)
                direction += 1f;
            if (Keyboard.current.downArrowKey.isPressed)
                direction -= 1f;

            if (!Mathf.Approximately(direction, 0f))
            {
                SetTargetDegrees(
                    selectedJointIndex,
                    GetTargetDegrees(selectedJointIndex) +
                    direction * keyboardSpeedDegreesPerSecond * Time.deltaTime);
                ApplyTargets();
            }
        }

        private void OnValidate()
        {
            EnsureArraySizes();
            stiffness = Mathf.Max(0f, stiffness);
            damping = Mathf.Max(0f, damping);
            forceLimit = Mathf.Max(0f, forceLimit);
        }

        private void FixedUpdate()
        {
            if (applyTargetsContinuously)
                ApplyTargets();
        }

        public float GetTargetDegrees(int jointIndex)
        {
            ValidateIndex(jointIndex);
            return targetDegrees[jointIndex];
        }

        public void SetTargetDegrees(int jointIndex, float degrees)
        {
            ValidateIndex(jointIndex);
            targetDegrees[jointIndex] = ClampToDriveLimits(joints[jointIndex], degrees);
        }

        public void SelectJoint(int jointIndex)
        {
            ValidateIndex(jointIndex);
            if (selectedJointIndex == jointIndex && originalPropertyBlocks.Count > 0)
                return;

            selectedJointIndex = jointIndex;
            RefreshJointHighlight();
        }

        public void SetTargetsDegrees(float[] degrees)
        {
            if (degrees == null || degrees.Length != JointCount)
                throw new ArgumentException($"Expected exactly {JointCount} joint targets.", nameof(degrees));

            for (var i = 0; i < JointCount; i++)
                SetTargetDegrees(i, degrees[i]);
        }

        [ContextMenu("Bind And Validate Joints")]
        public void BindAndValidateJoints()
        {
            EnsureArraySizes();
            var bodies = GetComponentsInChildren<ArticulationBody>(true);

            for (var i = 0; i < JointCount; i++)
            {
                joints[i] = Array.Find(bodies, body => body.name == JointBodyNames[i]);
                if (joints[i] == null)
                {
                    Debug.LogError(
                        $"UR5e joint '{JointNames[i]}' on link '{JointBodyNames[i]}' was not found below '{name}'.",
                        this);
                    continue;
                }

                if (joints[i].jointType != ArticulationJointType.RevoluteJoint)
                    Debug.LogError($"UR5e joint '{JointNames[i]}' is not revolute.", joints[i]);
            }

            ApplyTargets();
            RefreshJointHighlight();
        }

        [ContextMenu("Apply Joint Targets")]
        public void ApplyTargets()
        {
            EnsureArraySizes();

            for (var i = 0; i < JointCount; i++)
            {
                var joint = joints[i];
                if (joint == null)
                    continue;

                var drive = joint.xDrive;
                targetDegrees[i] = ClampToDriveLimits(joint, targetDegrees[i]);
                drive.target = targetDegrees[i];
                drive.stiffness = stiffness;
                drive.damping = damping;
                drive.forceLimit = forceLimit;
                joint.xDrive = drive;
            }
        }

        [ContextMenu("Reset Joint Targets")]
        public void ResetTargets()
        {
            EnsureArraySizes();
            Array.Clear(targetDegrees, 0, targetDegrees.Length);
            ApplyTargets();
        }

        private static float ClampToDriveLimits(ArticulationBody joint, float degrees)
        {
            if (joint == null)
                return degrees;

            var drive = joint.xDrive;
            return drive.lowerLimit <= drive.upperLimit
                ? Mathf.Clamp(degrees, drive.lowerLimit, drive.upperLimit)
                : degrees;
        }

        private void EnsureArraySizes()
        {
            if (joints == null || joints.Length != JointCount)
                Array.Resize(ref joints, JointCount);
            if (targetDegrees == null || targetDegrees.Length != JointCount)
                Array.Resize(ref targetDegrees, JointCount);
        }

        private bool HasAllJointReferences()
        {
            if (joints == null || joints.Length != JointCount)
                return false;

            for (var i = 0; i < joints.Length; i++)
            {
                if (joints[i] == null)
                    return false;
            }

            return true;
        }

        private void DisableLegacyUrdfController()
        {
            var behaviours = GetComponents<MonoBehaviour>();
            foreach (var behaviour in behaviours)
            {
                if (behaviour == null || behaviour == this)
                    continue;

                if (behaviour.GetType().FullName != "Unity.Robotics.UrdfImporter.Control.Controller")
                    continue;

                behaviour.enabled = false;
                Debug.Log("Disabled the URDF Importer's legacy keyboard controller; UR5eJointController owns the drives.", this);
            }
        }

        private void OnDisable()
        {
            RestoreJointHighlight();
        }

        private void RefreshJointHighlight()
        {
            RestoreJointHighlight();
            if (!highlightSelectedJoint || joints == null || selectedJointIndex >= joints.Length)
                return;

            var joint = joints[selectedJointIndex];
            if (joint == null)
                return;

            var visuals = joint.transform.Find("Visuals");
            if (visuals == null)
                return;

            highlightPropertyBlock ??= new MaterialPropertyBlock();

            foreach (var renderer in visuals.GetComponentsInChildren<Renderer>(true))
            {
                var original = new MaterialPropertyBlock();
                renderer.GetPropertyBlock(original);
                originalPropertyBlocks[renderer] = original;

                renderer.GetPropertyBlock(highlightPropertyBlock);
                highlightPropertyBlock.SetColor("_BaseColor", highlightColor);
                highlightPropertyBlock.SetColor("_Color", highlightColor);
                renderer.SetPropertyBlock(highlightPropertyBlock);
            }
        }

        private void RestoreJointHighlight()
        {
            foreach (var entry in originalPropertyBlocks)
            {
                if (entry.Key != null)
                    entry.Key.SetPropertyBlock(entry.Value);
            }

            originalPropertyBlocks.Clear();
        }

        private void AnchorBase()
        {
            if (!anchorRobotBase)
                return;

            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            foreach (var body in bodies)
            {
                if (!body.isRoot)
                    continue;

                body.immovable = true;
                body.useGravity = false;
                return;
            }

            Debug.LogError($"No root ArticulationBody was found below '{name}'.", this);
        }

        private static void ValidateIndex(int jointIndex)
        {
            if (jointIndex < 0 || jointIndex >= JointNames.Length)
                throw new ArgumentOutOfRangeException(nameof(jointIndex));
        }
    }
}
