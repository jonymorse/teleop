using Teleoperation.Robots.UR5e;
using UnityEditor;
using UnityEngine;

namespace Teleoperation.Robots.UR5e.Editor
{
    [CustomEditor(typeof(UR5eJointController))]
    public sealed class UR5eJointControllerEditor : UnityEditor.Editor
    {
        private SerializedProperty _targets;

        private void OnEnable()
        {
            _targets = serializedObject.FindProperty("targetDegrees");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            DrawPropertiesExcluding(serializedObject, "m_Script", "targetDegrees");

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Joint Targets", EditorStyles.boldLabel);

            for (var i = 0; i < UR5eJointController.JointNames.Length; i++)
            {
                var target = _targets.GetArrayElementAtIndex(i);
                target.floatValue = EditorGUILayout.Slider(
                    ObjectNames.NicifyVariableName(UR5eJointController.JointNames[i]),
                    target.floatValue,
                    -360f,
                    360f);
            }

            serializedObject.ApplyModifiedProperties();

            var controller = (UR5eJointController)target;
            EditorGUILayout.Space();
            if (GUILayout.Button("Bind and Validate Joints"))
                controller.BindAndValidateJoints();
            if (GUILayout.Button("Apply Targets"))
                controller.ApplyTargets();
            if (GUILayout.Button("Reset Targets"))
                controller.ResetTargets();
        }
    }
}

