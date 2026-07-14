using System.IO;
using Unity.Robotics.UrdfImporter;
using UnityEditor;

namespace Teleoperation.Robots.G1.Editor
{
    [InitializeOnLoad]
    internal static class G1Dex3AssetSetup
    {
        private const string MeshFolder = "Assets/Teleoperation/Robots/G1/Dex3/Resources/Dex3/Meshes";

        static G1Dex3AssetSetup()
        {
            EditorApplication.delayCall += EnsureImportedMeshes;
        }

        private static void EnsureImportedMeshes()
        {
            if (!Directory.Exists(MeshFolder))
                return;
            AssetDatabase.Refresh();
            var changed = false;
            foreach (var stlPath in Directory.GetFiles(MeshFolder, "*.STL"))
            {
                var assetPath = stlPath.Replace('\\', '/');
                var prefabPath = Path.ChangeExtension(assetPath, ".prefab");
                if (File.Exists(prefabPath))
                    continue;
                StlAssetPostProcessor.PostprocessStlFile(assetPath);
                changed = true;
            }
            if (changed)
            {
                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();
            }
        }
    }
}
