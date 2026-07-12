using Teleoperation.Robots.UR5e;
using UnityEditor;
using UnityEngine;

namespace Teleoperation.Robots.UR5e.Editor
{
    public sealed class UR5eJointControlWindow : EditorWindow
    {
        private UR5eJointController _controller;

        [MenuItem("Teleoperation/UR5e/Open Joint Controls")]
        public static void Open()
        {
            GetWindow<UR5eJointControlWindow>("UR5e Joints");
        }

        private void OnEnable()
        {
            FindController();
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("UR5e Joint Control", EditorStyles.boldLabel);
            _controller = (UR5eJointController)EditorGUILayout.ObjectField(
                "Robot", _controller, typeof(UR5eJointController), true);

            if (_controller == null)
            {
                EditorGUILayout.HelpBox(
                    "Open the UR5eImportTest scene or assign a UR5eJointController.",
                    MessageType.Info);
                if (GUILayout.Button("Find Robot in Scene"))
                    FindController();
                return;
            }

            EditorGUILayout.HelpBox(
                EditorApplication.isPlaying
                    ? $"Selected: {_controller.SelectedJointName}. Use Left/Right to select and Up/Down to move, or drag a slider."
                    : "Enter Play mode to watch the robot respond to these targets.",
                MessageType.None);

            EditorGUI.BeginChangeCheck();
            var targets = new float[_controller.JointCount];
            var changedJoint = -1;
            for (var i = 0; i < targets.Length; i++)
            {
                var oldTarget = _controller.GetTargetDegrees(i);
                targets[i] = EditorGUILayout.Slider(
                    ObjectNames.NicifyVariableName(UR5eJointController.JointNames[i]),
                    oldTarget,
                    -360f,
                    360f);
                if (!Mathf.Approximately(targets[i], oldTarget))
                    changedJoint = i;
            }

            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(_controller, "Change UR5e joint targets");
                _controller.SetTargetsDegrees(targets);
                if (changedJoint >= 0)
                    _controller.SelectJoint(changedJoint);
                _controller.ApplyTargets();
                EditorUtility.SetDirty(_controller);
            }

            EditorGUILayout.Space();
            if (GUILayout.Button("Bind and Validate Joints"))
                _controller.BindAndValidateJoints();
            if (GUILayout.Button("Reset Targets"))
                _controller.ResetTargets();
        }

        private void FindController()
        {
            _controller = FindFirstObjectByType<UR5eJointController>();
            Repaint();
        }
    }
}
