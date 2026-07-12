using UnityEditor;
using UnityEditor.PackageManager;
using UnityEngine;
using System.IO;

namespace Teleoperation.Robots.UR5e.Editor
{
    /// <summary>
    /// URDF Importer 0.5.2 predates Unity 6 and leaves its Windows Assimp DLLs
    /// enabled for every platform. Unity's Android builder then sees duplicate
    /// assimp.dll plugins. Keep both binaries out of Quest/Android builds.
    /// </summary>
    [InitializeOnLoad]
    public static class URDFImporterPluginCompatibility
    {
        private const string X86Plugin =
            "Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/Native/win/x86/assimp.dll";
        private const string X64Plugin =
            "Packages/com.unity.robotics.urdf-importer/Runtime/UnityMeshImporter/Plugins/AssimpNet/Native/win/x86_64/assimp.dll";

        static URDFImporterPluginCompatibility()
        {
            EditorApplication.delayCall += Apply;
        }

        [MenuItem("Teleoperation/UR5e/Fix URDF Importer Android Plugins")]
        public static void Apply()
        {
            Configure(X86Plugin, false);
            Configure(X64Plugin, true);
        }

        private static void Configure(string assetPath, bool editorCompatible)
        {
            // This old package contains two differently serialized "Any" records.
            // Unity 6's PluginImporter API only updates one of them, leaving the DLL
            // eligible for Android. Normalize the legacy record before reimporting.
            var package = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(assetPath);
            var packageRelativePath = assetPath.Substring(assetPath.IndexOf('/', "Packages/".Length) + 1);
            var metaPath = package == null
                ? Path.GetFullPath(assetPath + ".meta")
                : Path.Combine(package.resolvedPath, packageRelativePath + ".meta");
            if (File.Exists(metaPath))
            {
                var metadata = File.ReadAllText(metaPath);
                var normalized = metadata.Replace(
                    "      Any: \r\n    second:\r\n      enabled: 1",
                    "      Any: \r\n    second:\r\n      enabled: 0");
                normalized = normalized.Replace(
                    "      Any: \n    second:\n      enabled: 1",
                    "      Any: \n    second:\n      enabled: 0");
                if (normalized != metadata)
                    File.WriteAllText(metaPath, normalized);
            }

            var importer = AssetImporter.GetAtPath(assetPath) as PluginImporter;
            if (importer == null)
            {
                Debug.LogWarning($"URDF native plugin was not found: {assetPath}");
                return;
            }

            var changed = importer.GetCompatibleWithAnyPlatform() ||
                          importer.GetCompatibleWithPlatform(BuildTarget.Android) ||
                          importer.GetCompatibleWithEditor() != editorCompatible;
            if (!changed)
                return;

            importer.SetCompatibleWithAnyPlatform(false);
            importer.SetCompatibleWithPlatform(BuildTarget.Android, false);
            importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows, !editorCompatible);
            importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64, editorCompatible);
            importer.SetCompatibleWithEditor(editorCompatible);
            importer.SaveAndReimport();
            Debug.Log($"Updated URDF native plugin compatibility: {assetPath}");
        }
    }
}
