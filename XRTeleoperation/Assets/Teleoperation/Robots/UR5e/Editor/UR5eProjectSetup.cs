using System.IO;
using Teleoperation.Robots.UR5e;
using Teleoperation;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Teleoperation.Robots.UR5e.Editor
{
    [InitializeOnLoad]
    public static class UR5eProjectSetup
    {
        private const string PaletteSessionKey = "Teleoperation.UR5e.Palette.v1";
        private const string UrdfAssetPath = "Assets/Teleoperation/Robots/UR5e/ur5e.urdf";
        private const string PrefabDirectory = "Assets/Teleoperation/Robots/UR5e/Prefabs";
        private const string PrefabPath = PrefabDirectory + "/UR5e.prefab";
        private const string MaterialDirectory = "Assets/Teleoperation/Robots/UR5e/Materials";
        private const string VisualMeshDirectory =
            "Assets/Teleoperation/Robots/UR5e/ur_description/meshes/ur5e/visual";
        private const string SceneDirectory = "Assets/Teleoperation/Scenes";
        private const string ScenePath = SceneDirectory + "/UR5eImportTest.unity";

        static UR5eProjectSetup()
        {
            EditorApplication.delayCall += ApplyPaletteOncePerEditorSession;
            EditorApplication.delayCall += EnsureIkSolverInOpenTestScene;
            EditorApplication.delayCall += EnsureFlyCameraInOpenTestScene;
            EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
        }

        private static void OnPlayModeStateChanged(PlayModeStateChange state)
        {
            if (state == PlayModeStateChange.EnteredEditMode)
            {
                EditorApplication.delayCall += EnsureIkSolverInOpenTestScene;
                EditorApplication.delayCall += EnsureFlyCameraInOpenTestScene;
            }
        }

        private static void EnsureFlyCameraInOpenTestScene()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            var scene = SceneManager.GetActiveScene();
            if (scene.path != ScenePath)
                return;

            var camera = Object.FindFirstObjectByType<Camera>();
            if (camera == null || camera.GetComponent<FlyCameraController>() != null)
                return;

            Undo.AddComponent<FlyCameraController>(camera.gameObject);
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log("Fly camera added to UR5eImportTest and the scene was saved.");
        }

        private static void EnsureIkSolverInOpenTestScene()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            var scene = SceneManager.GetActiveScene();
            if (scene.path != ScenePath)
                return;

            var controller = Object.FindFirstObjectByType<UR5eJointController>();
            if (controller == null)
                return;

            var solver = controller.GetComponent<UR5ePositionIkSolver>();
            var changed = false;
            if (solver == null)
            {
                solver = Undo.AddComponent<UR5ePositionIkSolver>(controller.gameObject);
                changed = true;
            }

            var endEffector = FindDescendant(controller.transform, "tool0");
            var targetObject = GameObject.Find("UR5e IK Target");
            if (targetObject == null && endEffector != null)
            {
                targetObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                Undo.RegisterCreatedObjectUndo(targetObject, "Create UR5e IK target");
                targetObject.name = "UR5e IK Target";
                targetObject.transform.position = endEffector.position;
                targetObject.transform.localScale = Vector3.one * 0.14f;
                var collider = targetObject.GetComponent<Collider>();
                if (collider != null)
                    collider.isTrigger = true;
                targetObject.AddComponent<UR5eIkTarget>();
                targetObject.AddComponent<UR5eTargetMouseDrag>();
                changed = true;
            }

            else if (targetObject != null && targetObject.transform.localScale.x < 0.14f)
            {
                targetObject.transform.localScale = Vector3.one * 0.14f;
                changed = true;
            }

            if (targetObject != null)
            {
                var collider = targetObject.GetComponent<SphereCollider>();
                if (collider == null)
                    collider = Undo.AddComponent<SphereCollider>(targetObject);
                collider.isTrigger = true;

                if (targetObject.GetComponent<UR5eTargetMouseDrag>() == null)
                    Undo.AddComponent<UR5eTargetMouseDrag>(targetObject);
                changed = true;
            }

            if (endEffector != null && targetObject != null)
            {
                var serializedSolver = new SerializedObject(solver);
                serializedSolver.FindProperty("robot").objectReferenceValue = controller;
                serializedSolver.FindProperty("endEffector").objectReferenceValue = endEffector;
                serializedSolver.FindProperty("target").objectReferenceValue = targetObject.transform;
                serializedSolver.ApplyModifiedPropertiesWithoutUndo();
                changed = true;
            }

            if (!changed)
                return;

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log("UR5e IK solver and persistent target configured in UR5eImportTest; the scene was saved.");
        }

        private static Transform FindDescendant(Transform root, string descendantName)
        {
            foreach (var child in root.GetComponentsInChildren<Transform>(true))
            {
                if (child.name == descendantName)
                    return child;
            }

            return null;
        }

        private static void ApplyPaletteOncePerEditorSession()
        {
            if (SessionState.GetBool(PaletteSessionKey, false))
                return;

            SessionState.SetBool(PaletteSessionKey, true);
            CreateAndRemapOfficialMaterials();
        }

        [MenuItem("Teleoperation/UR5e/Rebuild Prefab and Test Scene")]
        public static void RebuildPrefabAndTestScene()
        {
            CreateAndRemapOfficialMaterials();
            EnsureAssetDirectory(PrefabDirectory);
            EnsureAssetDirectory(SceneDirectory);

            var fullUrdfPath = Path.GetFullPath(UrdfAssetPath);
            if (!File.Exists(fullUrdfPath))
                throw new FileNotFoundException("The generated UR5e URDF is missing.", fullUrdfPath);

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var settings = ImportSettings.DefaultSettings();
            settings.chosenAxis = ImportSettings.axisType.yAxis;
            settings.convexMethod = ImportSettings.convexDecomposer.unity;
            settings.OverwriteExistingPrefabs = true;

            GameObject robot = null;
            var import = UrdfRobotExtensions.Create(fullUrdfPath, settings);
            while (import.MoveNext())
            {
                if (import.Current != null)
                    robot = import.Current;
            }

            if (robot == null)
                throw new System.InvalidOperationException("URDF Importer did not create a robot root.");

            robot.name = "UR5e";
            RemoveLegacyImporterController(robot);
            var controller = robot.GetComponent<UR5eJointController>();
            if (controller == null)
                controller = robot.AddComponent<UR5eJointController>();
            controller.BindAndValidateJoints();
            if (robot.GetComponent<UR5ePositionIkSolver>() == null)
                robot.AddComponent<UR5ePositionIkSolver>();

            ConvertImportedMaterialsToUrp();

            var prefab = PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
            if (prefab == null)
                throw new System.InvalidOperationException($"Failed to create prefab at {PrefabPath}.");

            AddTestEnvironment();
            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
                throw new System.InvalidOperationException($"Failed to save test scene at {ScenePath}.");

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Selection.activeObject = prefab;
            Debug.Log($"UR5e setup complete: {PrefabPath} and {ScenePath}");
        }

        [MenuItem("Teleoperation/UR5e/Add IK Solver to Current Scene")]
        public static void AddIkSolverToCurrentScene()
        {
            var controller = Object.FindFirstObjectByType<UR5eJointController>();
            if (controller == null)
                throw new System.InvalidOperationException("No UR5eJointController exists in the current scene.");

            var solver = controller.GetComponent<UR5ePositionIkSolver>();
            if (solver == null)
                solver = Undo.AddComponent<UR5ePositionIkSolver>(controller.gameObject);

            EditorUtility.SetDirty(solver);
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
            Selection.activeObject = solver;
            Debug.Log("UR5e position IK solver added. Enter Play mode to create and manipulate its target.");
        }

        [MenuItem("Teleoperation/UR5e/Fix Imported Materials for URP")]
        public static void ConvertImportedMaterialsToUrp()
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                throw new System.InvalidOperationException("Universal Render Pipeline/Lit shader was not found.");

            var materialGuids = AssetDatabase.FindAssets(
                "t:Material",
                new[] { "Assets/Teleoperation/Robots/UR5e/Materials" });

            foreach (var guid in materialGuids)
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);
                var material = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (material == null || material.shader == shader)
                    continue;

                var color = material.HasProperty("_Color") ? material.color : Color.white;
                material.shader = shader;
                if (material.HasProperty("_BaseColor"))
                    material.SetColor("_BaseColor", color);
                EditorUtility.SetDirty(material);
            }

            AssetDatabase.SaveAssets();
        }

        [MenuItem("Teleoperation/UR5e/Restore Official Material Palette")]
        public static void CreateAndRemapOfficialMaterials()
        {
            EnsureAssetDirectory(MaterialDirectory);
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                throw new System.InvalidOperationException("Universal Render Pipeline/Lit shader was not found.");

            var palette = new[]
            {
                CreateOrUpdateMaterial("JointGrey", new Color(0.2784314f, 0.2784314f, 0.2784314f, 1f), 0.2f, shader),
                CreateOrUpdateMaterial("Black", new Color(0.03310248f, 0.03310248f, 0.03310248f, 1f), 0.15f, shader),
                CreateOrUpdateMaterial("LinkGrey", new Color(0.8203922f, 0.8203922f, 0.8203922f, 1f), 0.45f, shader),
                CreateOrUpdateMaterial("URBlue", new Color(0.4901961f, 0.6784314f, 0.8f, 1f), 0.25f, shader)
            };

            AssetDatabase.SaveAssets();

            var modelGuids = AssetDatabase.FindAssets("t:Model", new[] { VisualMeshDirectory });
            foreach (var modelGuid in modelGuids)
            {
                var modelPath = AssetDatabase.GUIDToAssetPath(modelGuid);
                var importer = AssetImporter.GetAtPath(modelPath) as ModelImporter;
                if (importer == null)
                    continue;

                var changed = false;
                foreach (var material in palette)
                {
                    var source = new AssetImporter.SourceAssetIdentifier(typeof(Material), material.name);
                    if (importer.GetExternalObjectMap().TryGetValue(source, out var current) && current == material)
                        continue;

                    importer.AddRemap(source, material);
                    changed = true;
                }

                if (changed)
                    importer.SaveAndReimport();
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("UR5e official URP material palette restored and remapped.");
        }

        private static Material CreateOrUpdateMaterial(
            string name,
            Color color,
            float smoothness,
            Shader shader)
        {
            var path = $"{MaterialDirectory}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                material = new Material(shader) { name = name };
                AssetDatabase.CreateAsset(material, path);
            }

            material.shader = shader;
            material.SetColor("_BaseColor", color);
            material.SetColor("_Color", color);
            material.SetFloat("_Smoothness", smoothness);
            material.SetFloat("_Surface", 0f);
            material.SetFloat("_AlphaClip", 0f);
            material.renderQueue = -1;
            EditorUtility.SetDirty(material);
            return material;
        }

        private static void AddTestEnvironment()
        {
            var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
            floor.name = "Floor";
            floor.transform.localScale = new Vector3(0.5f, 1f, 0.5f);

            var lightObject = new GameObject("Directional Light");
            lightObject.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            var light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;

            var cameraObject = new GameObject("Test Camera");
            cameraObject.tag = "MainCamera";
            cameraObject.transform.SetPositionAndRotation(
                new Vector3(1.8f, 1.4f, -1.8f),
                Quaternion.Euler(18f, -42f, 0f));
            cameraObject.AddComponent<Camera>();
        }

        private static void RemoveLegacyImporterController(GameObject robot)
        {
            var behaviours = robot.GetComponents<MonoBehaviour>();
            foreach (var behaviour in behaviours)
            {
                if (behaviour != null &&
                    behaviour.GetType().FullName == "Unity.Robotics.UrdfImporter.Control.Controller")
                {
                    Object.DestroyImmediate(behaviour);
                }
            }
        }

        private static void EnsureAssetDirectory(string assetPath)
        {
            var parts = assetPath.Split('/');
            var current = parts[0];
            for (var i = 1; i < parts.Length; i++)
            {
                var next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[i]);
                current = next;
            }
        }
    }
}
