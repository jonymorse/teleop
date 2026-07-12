using UnityEditor;
using UnityEditor.XR.Management;
using UnityEditor.XR.Management.Metadata;
using UnityEngine;

namespace Teleoperation.Robots.UR5e.Editor
{
    [InitializeOnLoad]
    public static class QuestBuildCompatibility
    {
        private const string XrSettingsPath = "Assets/XR/XRGeneralSettingsPerBuildTarget.asset";
        private const string OpenXrLoaderType = "UnityEngine.XR.OpenXR.OpenXRLoader";

        static QuestBuildCompatibility()
        {
            EditorApplication.delayCall += Apply;
        }

        [MenuItem("Teleoperation/UR5e/Fix Quest Build Compatibility")]
        public static void Apply()
        {
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel32;
            PlayerSettings.Android.targetSdkVersion = AndroidSdkVersions.AndroidApiLevel32;
            PlayerSettings.Android.applicationEntry = AndroidApplicationEntry.GameActivity;

            // Meta's validator checks the generated manifest as well as PlayerSettings.
            // Generate/update it so it declares UnityPlayerGameActivity.
            OVRManifestPreprocessor.GenerateOrUpdateAndroidManifest(silentMode: true);

            var perBuildTarget = AssetDatabase.LoadAssetAtPath<XRGeneralSettingsPerBuildTarget>(XrSettingsPath);
            if (perBuildTarget == null)
            {
                Debug.LogError($"XR settings asset was not found at {XrSettingsPath}.");
                return;
            }

            if (!perBuildTarget.HasManagerSettingsForBuildTarget(BuildTargetGroup.Android))
                perBuildTarget.CreateDefaultManagerSettingsForBuildTarget(BuildTargetGroup.Android);

            var manager = perBuildTarget.ManagerSettingsForBuildTarget(BuildTargetGroup.Android);
            manager.automaticLoading = true;
            manager.automaticRunning = true;

            if (!XRPackageMetadataStore.IsLoaderAssigned(OpenXrLoaderType, BuildTargetGroup.Android))
            {
                if (!XRPackageMetadataStore.AssignLoader(manager, OpenXrLoaderType, BuildTargetGroup.Android))
                    Debug.LogError("Failed to assign OpenXRLoader to the Android XR manager.");
            }

            EditorUtility.SetDirty(manager);
            EditorUtility.SetDirty(perBuildTarget);
            AssetDatabase.SaveAssets();
            Debug.Log("Quest compatibility applied: API 32, GameActivity, Android OpenXR loader.");
        }
    }
}
