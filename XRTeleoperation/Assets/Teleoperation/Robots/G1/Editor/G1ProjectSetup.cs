using System.IO;
using System.Collections.Generic;
using Teleoperation.Robots.G1;
using Teleoperation.Robots.UR5e;
using Unity.Robotics.UrdfImporter;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Teleoperation.Robots.G1.Editor
{
    [InitializeOnLoad]
    public static class G1ProjectSetup
    {
        private const string UrdfPath = "Assets/Teleoperation/Robots/G1/Model/g1_29dof.urdf";
        private const string PrefabPath = "Assets/Teleoperation/Robots/G1/Prefabs/G1.prefab";
        private const string ScenePath = "Assets/Teleoperation/Scenes/G1ImportTest.unity";
        private const string XrSourceScenePath = "Assets/Teleoperation/Scenes/UR5eXRTeleoperation.unity";
        private const string XrScenePath = "Assets/Teleoperation/Scenes/G1XRTeleoperation.unity";
        private const string MaterialsPath = "Assets/Teleoperation/Robots/G1/Materials";
        private const string GeneratedMeshesPath = "Assets/Teleoperation/Robots/G1/GeneratedMeshes";

        static G1ProjectSetup()
        {
            EditorApplication.delayCall += CreateInitialAssetsIfNeeded;
            EditorApplication.delayCall += CreateXrSceneIfNeeded;
            EditorApplication.delayCall += RepairOfficialPalette;
            EditorApplication.delayCall += UpgradeManipulationSceneIfNeeded;
            EditorApplication.playModeStateChanged += state =>
            {
                if (state == PlayModeStateChange.EnteredEditMode)
                    EditorApplication.delayCall += UpgradeManipulationSceneIfNeeded;
            };
        }

        public static void BuildQuestApkFromCommandLine()
        {
            var scenes = new List<string>();
            foreach (var scene in EditorBuildSettings.scenes)
                if (scene.enabled)
                    scenes.Add(scene.path);
            var options = new BuildPlayerOptions
            {
                scenes = scenes.ToArray(),
                locationPathName = Path.GetFullPath("XRTeleoperation.apk"),
                target = BuildTarget.Android,
                options = BuildOptions.None
            };
            var report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
                throw new System.Exception($"Quest APK build failed: {report.summary.result}");
        }

        [MenuItem("Teleoperation/G1/Import Official G1 and Create Test Scene")]
        public static void ImportAndCreateTestScene()
        {
            var previousScene = SceneManager.GetActiveScene().path;
            var projectRoot = Directory.GetParent(Application.dataPath)!.FullName;
            var absoluteUrdf = Path.Combine(projectRoot, UrdfPath);
            var settings = ImportSettings.DefaultSettings();
            settings.chosenAxis = ImportSettings.axisType.yAxis;
            settings.convexMethod = ImportSettings.convexDecomposer.unity;

            var robot = UrdfRobotExtensions.CreateRuntime(absoluteUrdf, settings);
            if (robot == null)
            {
                Debug.LogError("G1 URDF import failed.");
                return;
            }

            robot.name = "Unitree G1 29DoF";
            var legacyController = robot.GetComponent<Unity.Robotics.UrdfImporter.Control.Controller>();
            if (legacyController != null)
                Object.DestroyImmediate(legacyController);
            robot.AddComponent<G1JointController>().BindJoints();
            robot.AddComponent<G1DualArmIkController>();
            PersistGeneratedMeshes(robot);
            ApplyUrpMaterials(robot);

            PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath);
            Object.DestroyImmediate(robot);

            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var instance = PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath)) as GameObject;
            instance!.transform.position = new Vector3(0f, 0.8f, 0f);

            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Floor";
            floor.transform.position = new Vector3(0f, -0.025f, 0f);
            floor.transform.localScale = new Vector3(6f, 0.05f, 6f);

            var lightObject = new GameObject("Directional Light");
            var light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = 1.2f;
            lightObject.transform.rotation = Quaternion.Euler(45f, -35f, 0f);

            var cameraObject = new GameObject("Main Camera");
            cameraObject.tag = "MainCamera";
            var camera = cameraObject.AddComponent<Camera>();
            camera.transform.position = new Vector3(0f, 0.6f, -2f);
            camera.transform.rotation = Quaternion.identity;

            EditorSceneManager.SaveScene(scene, ScenePath);
            AssetDatabase.SaveAssets();
            if (!string.IsNullOrEmpty(previousScene) && previousScene != ScenePath)
                EditorSceneManager.OpenScene(previousScene);

            Debug.Log("Official Unitree G1 29-DoF prefab and G1ImportTest scene created.");
        }

        private static void CreateInitialAssetsIfNeeded()
        {
            if (Application.isPlaying || EditorApplication.isCompiling || !PrefabNeedsRepair())
                return;

            ImportAndCreateTestScene();
        }

        private static bool PrefabNeedsRepair()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null)
                return true;

            foreach (var filter in prefab.GetComponentsInChildren<MeshFilter>(true))
            {
                if (filter.sharedMesh == null)
                    return true;
            }

            return false;
        }

        private static void PersistGeneratedMeshes(GameObject robot)
        {
            if (AssetDatabase.IsValidFolder(GeneratedMeshesPath))
                AssetDatabase.DeleteAsset(GeneratedMeshesPath);
            AssetDatabase.CreateFolder("Assets/Teleoperation/Robots/G1", "GeneratedMeshes");

            var persisted = new Dictionary<Mesh, Mesh>();
            var nextId = 0;
            Mesh Persist(Mesh source)
            {
                if (source == null)
                    return null;
                if (persisted.TryGetValue(source, out var existing))
                    return existing;

                var mesh = Object.Instantiate(source);
                mesh.name = $"G1Mesh_{nextId++:D3}_{Sanitize(source.name)}";
                AssetDatabase.CreateAsset(mesh, $"{GeneratedMeshesPath}/{mesh.name}.asset");
                persisted[source] = mesh;
                return mesh;
            }

            foreach (var filter in robot.GetComponentsInChildren<MeshFilter>(true))
                filter.sharedMesh = Persist(filter.sharedMesh);
            foreach (var collider in robot.GetComponentsInChildren<MeshCollider>(true))
                collider.sharedMesh = Persist(collider.sharedMesh);
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return "Mesh";
            foreach (var invalid in Path.GetInvalidFileNameChars())
                value = value.Replace(invalid, '_');
            return value;
        }

        private static void ApplyUrpMaterials(GameObject robot)
        {
            Directory.CreateDirectory(Path.Combine(Application.dataPath, "Teleoperation/Robots/G1/Materials"));
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                return;

            var dark = GetOrCreateMaterial($"{MaterialsPath}/G1Dark.mat", shader, new Color(0.035f, 0.045f, 0.055f));
            var light = GetOrCreateMaterial($"{MaterialsPath}/G1Light.mat", shader, new Color(0.78f, 0.8f, 0.82f));
            var darkLinks = new HashSet<string>
            {
                "pelvis", "left_hip_pitch_link", "left_ankle_roll_link",
                "right_hip_pitch_link", "right_ankle_roll_link", "logo_link", "head_link",
                "left_rubber_hand", "right_rubber_hand"
            };
            foreach (var renderer in robot.GetComponentsInChildren<Renderer>(true))
                renderer.sharedMaterial = HasNamedAncestor(renderer.transform, darkLinks) ? dark : light;
        }

        private static bool HasNamedAncestor(Transform transform, HashSet<string> names)
        {
            while (transform != null)
            {
                if (transform.name == "pelvis" || transform.name.EndsWith("_link") || transform.name.EndsWith("_hand"))
                    return names.Contains(transform.name);
                transform = transform.parent;
            }

            return false;
        }

        private static void RepairOfficialPalette()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null)
                return;

            var dark = AssetDatabase.LoadAssetAtPath<Material>($"{MaterialsPath}/G1Dark.mat");
            var light = AssetDatabase.LoadAssetAtPath<Material>($"{MaterialsPath}/G1Light.mat");
            var darkLinks = new HashSet<string>
            {
                "pelvis", "left_hip_pitch_link", "left_ankle_roll_link",
                "right_hip_pitch_link", "right_ankle_roll_link", "logo_link", "head_link",
                "left_rubber_hand", "right_rubber_hand"
            };
            var needsRepair = dark == null || light == null;
            if (!needsRepair)
            {
                foreach (var renderer in prefab.GetComponentsInChildren<Renderer>(true))
                {
                    var expected = HasNamedAncestor(renderer.transform, darkLinks) ? dark : light;
                    if (renderer.sharedMaterial != expected) { needsRepair = true; break; }
                }
            }
            if (!needsRepair)
                return;

            var contents = PrefabUtility.LoadPrefabContents(PrefabPath);
            if (contents.GetComponent<G1DualArmIkController>() == null)
                contents.AddComponent<G1DualArmIkController>();
            ApplyUrpMaterials(contents);
            PrefabUtility.SaveAsPrefabAsset(contents, PrefabPath);
            PrefabUtility.UnloadPrefabContents(contents);
            AssetDatabase.SaveAssets();
        }

        private static Material GetOrCreateMaterial(string path, Shader shader, Color color)
        {
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material != null)
            {
                material.shader = shader;
                material.color = color;
                material.SetFloat("_Metallic", 0f);
                material.SetFloat("_Smoothness", 0.35f);
                EditorUtility.SetDirty(material);
                return material;
            }

            material = new Material(shader) { color = color };
            material.SetFloat("_Metallic", 0f);
            material.SetFloat("_Smoothness", 0.35f);
            AssetDatabase.CreateAsset(material, path);
            return material;
        }

        [MenuItem("Teleoperation/G1/Create G1 XR Teleoperation Scene")]
        public static void CreateXrTeleoperationScene()
        {
            if (AssetDatabase.LoadAssetAtPath<SceneAsset>(XrSourceScenePath) == null)
            {
                Debug.LogError($"XR source scene was not found at {XrSourceScenePath}.");
                return;
            }

            var previousScene = SceneManager.GetActiveScene().path;
            AssetDatabase.DeleteAsset(XrScenePath);
            AssetDatabase.CopyAsset(XrSourceScenePath, XrScenePath);
            var scene = EditorSceneManager.OpenScene(XrScenePath, OpenSceneMode.Single);

            foreach (var controller in Object.FindObjectsByType<UR5eJointController>(FindObjectsSortMode.None))
                Object.DestroyImmediate(controller.gameObject);
            var oldTarget = GameObject.Find("UR5e IK Target");
            if (oldTarget != null)
                Object.DestroyImmediate(oldTarget);

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            var g1 = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
            g1!.name = "Unitree G1 Teleoperation Digital Twin";
            g1.transform.position = new Vector3(0f, 0.8f, 1.45f);
            g1.AddComponent<G1XrTeleoperationController>();
            g1.AddComponent<G1SimulatedGraspController>();
            CreateManipulationTestContent();

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene, XrScenePath);
            AddSceneToBuildSettings(XrScenePath);
            AssetDatabase.SaveAssets();
            if (!string.IsNullOrEmpty(previousScene) && previousScene != XrScenePath)
                EditorSceneManager.OpenScene(previousScene);
            Debug.Log("G1 XR teleoperation scene created with Meta rig, passthrough, calibration, clutching, and safety controls.");
        }

        private static void CreateXrSceneIfNeeded()
        {
            if (Application.isPlaying || EditorApplication.isCompiling || AssetDatabase.LoadAssetAtPath<SceneAsset>(XrScenePath) != null)
                return;
            CreateXrTeleoperationScene();
        }

        private static void AddSceneToBuildSettings(string path)
        {
            var scenes = new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
            var found = false;
            for (var i = 0; i < scenes.Count; i++)
            {
                if (scenes[i].path == path)
                {
                    scenes[i].enabled = true;
                    found = true;
                }
                else if (scenes[i].path == XrSourceScenePath)
                {
                    scenes[i].enabled = false;
                }
            }
            if (!found)
                scenes.Add(new EditorBuildSettingsScene(path, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }

        private static void UpgradeManipulationSceneIfNeeded()
        {
            if (Application.isPlaying || EditorApplication.isCompiling ||
                AssetDatabase.LoadAssetAtPath<SceneAsset>(XrScenePath) == null)
                return;

            var previousScene = SceneManager.GetActiveScene().path;
            var scene = EditorSceneManager.OpenScene(XrScenePath, OpenSceneMode.Single);
            var controller = Object.FindFirstObjectByType<G1XrTeleoperationController>();
            var changed = false;
            if (controller != null && controller.GetComponent<G1SimulatedGraspController>() == null)
            {
                controller.gameObject.AddComponent<G1SimulatedGraspController>();
                changed = true;
            }
            if (GameObject.Find("[G1] Manipulation Test") == null)
            {
                CreateManipulationTestContent();
                changed = true;
            }
            if (EnsurePickAndPlaceTaskContent())
                changed = true;
            if (EnsureHeadsetLayoutPreview(controller))
                changed = true;
            if (changed)
            {
                EditorSceneManager.MarkSceneDirty(scene);
                EditorSceneManager.SaveScene(scene, XrScenePath);
            }
            if (!string.IsNullOrEmpty(previousScene) && previousScene != XrScenePath)
                EditorSceneManager.OpenScene(previousScene);
        }

        private static void CreateManipulationTestContent()
        {
            var root = new GameObject("[G1] Manipulation Test");
            var table = GameObject.CreatePrimitive(PrimitiveType.Cube);
            table.name = "Manipulation Table";
            table.transform.SetParent(root.transform);
            table.transform.position = new Vector3(0f, 0.33f, 1.03f);
            table.transform.localScale = new Vector3(0.85f, 0.66f, 0.55f);
            table.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial(
                $"{MaterialsPath}/ManipulationTable.mat", Shader.Find("Universal Render Pipeline/Lit"), new Color(0.16f, 0.18f, 0.21f));

            var colors = new[] { new Color(0.15f, 0.55f, 1f) };
            for (var i = 0; i < 1; i++)
            {
                var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
                box.name = $"Pick Box {i + 1}";
                box.transform.SetParent(root.transform);
                box.transform.position = new Vector3((i - 1) * 0.18f, 0.72f, 1.03f);
                box.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                var body = box.AddComponent<Rigidbody>();
                body.mass = 0.15f;
                var graspable = box.AddComponent<G1Graspable>();
                graspable.SetDisplayName($"Box {i + 1}");
                box.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial(
                    $"{MaterialsPath}/PickBox{i + 1}.mat", Shader.Find("Universal Render Pipeline/Lit"), colors[i]);
            }
        }

        private static bool EnsurePickAndPlaceTaskContent()
        {
            var root = GameObject.Find("[G1] Manipulation Test");
            if (root == null)
                return false;
            var table = FindDescendant(root.transform, "Manipulation Table");
            if (table == null)
                return false;

            var changed = false;
            var shader = Shader.Find("Universal Render Pipeline/Lit");
            var cylinder = FindDescendant(root.transform, "Pick Cylinder");
            if (cylinder == null)
            {
                var cylinderObject = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                cylinderObject.name = "Pick Cylinder";
                cylinderObject.transform.SetParent(root.transform);
                cylinderObject.transform.localScale = new Vector3(0.085f, 0.065f, 0.085f);
                var body = cylinderObject.AddComponent<Rigidbody>();
                body.mass = 0.2f;
                var graspable = cylinderObject.AddComponent<G1Graspable>();
                graspable.SetDisplayName("Pick Cylinder");
                cylinderObject.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial(
                    $"{MaterialsPath}/PickCylinder.mat", shader, new Color(0.95f, 0.38f, 0.08f));
                cylinder = cylinderObject.transform;
                changed = true;
            }

            var desiredCylinderScale = new Vector3(0.085f, 0.065f, 0.085f);
            if ((cylinder.localScale - desiredCylinderScale).sqrMagnitude > 0.000001f)
            {
                cylinder.localScale = desiredCylinderScale;
                changed = true;
            }
            var capsuleCollider = cylinder.GetComponent<CapsuleCollider>();
            var cylinderMeshCollider = cylinder.GetComponent<MeshCollider>();
            if (capsuleCollider != null || cylinderMeshCollider == null || !cylinderMeshCollider.convex)
            {
                if (capsuleCollider != null)
                    Object.DestroyImmediate(capsuleCollider);
                cylinderMeshCollider ??= cylinder.gameObject.AddComponent<MeshCollider>();
                cylinderMeshCollider.sharedMesh = cylinder.GetComponent<MeshFilter>().sharedMesh;
                cylinderMeshCollider.convex = true;
                changed = true;
            }

            var bin = FindDescendant(root.transform, "Placement Bin");
            if (bin == null)
            {
                bin = new GameObject("Placement Bin").transform;
                bin.SetParent(root.transform);
                var binMaterial = GetOrCreateMaterial(
                    $"{MaterialsPath}/PlacementBin.mat", shader, new Color(0.1f, 0.55f, 0.7f));
                const float innerWidth = 0.2f;
                const float innerDepth = 0.18f;
                const float wallHeight = 0.13f;
                const float wallThickness = 0.025f;
                const float floorThickness = 0.018f;
                var outerWidth = innerWidth + wallThickness * 2f;
                var outerDepth = innerDepth + wallThickness * 2f;
                CreateBinPart(bin, "Bin Floor", new Vector3(0f, floorThickness * 0.5f, 0f),
                    new Vector3(outerWidth, floorThickness, outerDepth), binMaterial);
                CreateBinPart(bin, "Bin Left Wall",
                    new Vector3(-outerWidth * 0.5f + wallThickness * 0.5f, wallHeight * 0.5f, 0f),
                    new Vector3(wallThickness, wallHeight, outerDepth), binMaterial);
                CreateBinPart(bin, "Bin Right Wall",
                    new Vector3(outerWidth * 0.5f - wallThickness * 0.5f, wallHeight * 0.5f, 0f),
                    new Vector3(wallThickness, wallHeight, outerDepth), binMaterial);
                CreateBinPart(bin, "Bin Front Wall",
                    new Vector3(0f, wallHeight * 0.5f, -outerDepth * 0.5f + wallThickness * 0.5f),
                    new Vector3(innerWidth, wallHeight, wallThickness), binMaterial);
                CreateBinPart(bin, "Bin Back Wall",
                    new Vector3(0f, wallHeight * 0.5f, outerDepth * 0.5f - wallThickness * 0.5f),
                    new Vector3(innerWidth, wallHeight, wallThickness), binMaterial);
                var goal = new GameObject("Placement Goal");
                goal.transform.SetParent(bin, false);
                var goalCollider = goal.AddComponent<BoxCollider>();
                goalCollider.isTrigger = true;
                goalCollider.size = new Vector3(innerWidth - 0.012f, wallHeight, innerDepth - 0.012f);
                goalCollider.center = new Vector3(0f, floorThickness + wallHeight * 0.5f, 0f);
                changed = true;
            }

            var cube = FindDescendant(root.transform, "Pick Box 1");
            var tableCenter = table.localPosition;
            changed |= SetLocalPositionIfDifferent(cube,
                new Vector3(-0.11f, tableCenter.y + table.localScale.y * 0.5f + 0.058f,
                    tableCenter.z - 0.09f));
            changed |= SetLocalPositionIfDifferent(cylinder,
                new Vector3(0.1f, tableCenter.y + table.localScale.y * 0.5f + 0.073f,
                    tableCenter.z - 0.09f));
            changed |= SetLocalPositionIfDifferent(bin,
                new Vector3(0f, tableCenter.y + table.localScale.y * 0.5f, tableCenter.z + 0.16f));

            return changed;
        }

        private static bool EnsureHeadsetLayoutPreview(G1XrTeleoperationController controller)
        {
            if (controller == null)
                return false;
            var serializedController = new SerializedObject(controller);
            var version = serializedController.FindProperty("editorPreviewLayoutVersion");
            if (version != null && version.intValue >= 4)
                return false;

            // Scene-view preview assumes a standing, forward-facing operator at 1.6 m.
            // Runtime still performs the authoritative placement from the real Quest head pose.
            var previewHead = new Vector3(0f, 1.6f, 0f);
            var robotTransform = controller.transform;
            robotTransform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            Physics.SyncTransforms();
            var renderers = controller.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length > 0)
            {
                var bounds = renderers[0].bounds;
                for (var i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
                var desiredCenter = previewHead + Vector3.right * 1.25f - Vector3.forward * 0.35f -
                                    Vector3.up * 0.15f;
                var delta = Vector3.right * Vector3.Dot(desiredCenter - bounds.center, Vector3.right) +
                            Vector3.forward * Vector3.Dot(desiredCenter - bounds.center, Vector3.forward);
                if (TryGetFootBottom(renderers, out var footBottom))
                    delta += Vector3.up * (0.52f - footBottom);
                else
                    delta += Vector3.up * (desiredCenter.y - bounds.center.y);
                robotTransform.position += delta;
            }

            var environment = GameObject.Find("[G1] Manipulation Test");
            var table = environment != null ? FindDescendant(environment.transform, "Manipulation Table") : null;
            if (environment != null && table != null)
            {
                environment.transform.rotation = Quaternion.identity;
                Physics.SyncTransforms();
                var tableBounds = table.GetComponent<Collider>().bounds;
                var currentTop = new Vector3(tableBounds.center.x, tableBounds.max.y, tableBounds.center.z);
                var desiredTop = previewHead + Vector3.right * 1.25f + Vector3.forward * 0.35f -
                                 Vector3.up * 0.42f;
                environment.transform.position += desiredTop - currentTop;
            }

            Physics.SyncTransforms();
            if (version != null)
            {
                version.intValue = 4;
                serializedController.ApplyModifiedPropertiesWithoutUndo();
            }
            Debug.Log("G1 Scene view aligned to the headset-relative startup layout preview.");
            return true;
        }

        private static bool SetLocalPositionIfDifferent(Transform target, Vector3 value)
        {
            if (target == null || (target.localPosition - value).sqrMagnitude <= 0.000001f)
                return false;
            target.localPosition = value;
            return true;
        }

        private static bool TryGetFootBottom(Renderer[] renderers, out float bottom)
        {
            bottom = float.PositiveInfinity;
            var found = false;
            foreach (var renderer in renderers)
            {
                if (!HasNamedAncestor(renderer.transform, "left_ankle_roll_link") &&
                    !HasNamedAncestor(renderer.transform, "right_ankle_roll_link"))
                    continue;
                bottom = Mathf.Min(bottom, renderer.bounds.min.y);
                found = true;
            }
            return found;
        }

        private static bool HasNamedAncestor(Transform transform, string objectName)
        {
            while (transform != null)
            {
                if (transform.name == objectName) return true;
                transform = transform.parent;
            }
            return false;
        }

        private static void CreateBinPart(Transform parent, string objectName, Vector3 localPosition,
            Vector3 localScale, Material material)
        {
            var part = GameObject.CreatePrimitive(PrimitiveType.Cube);
            part.name = objectName;
            part.transform.SetParent(parent, false);
            part.transform.localPosition = localPosition;
            part.transform.localScale = localScale;
            part.GetComponent<Renderer>().sharedMaterial = material;
        }

        private static Transform FindDescendant(Transform parent, string objectName)
        {
            foreach (var child in parent.GetComponentsInChildren<Transform>(true))
                if (child.name == objectName) return child;
            return null;
        }


    }
}
