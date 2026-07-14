using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Teleoperation.Robots.G1
{
    /// <summary>Owns the alignable, physics-safe pick-and-place test environment.</summary>
    [DisallowMultipleComponent]
    public sealed class G1ManipulationEnvironment : MonoBehaviour
    {
        private const float BinInnerWidth = 0.2f;
        private const float BinInnerDepth = 0.18f;
        private const float BinWallHeight = 0.13f;
        private const float BinWallThickness = 0.025f;
        private const float BinFloorThickness = 0.018f;
        private const float BinCenterForwardOffset = 0.16f;

        private readonly List<TaskObject> taskObjects = new();
        private Transform environmentRoot;
        private Collider tableCollider;
        private Transform receptacleRoot;
        private BoxCollider goalVolume;
        private Renderer[] receptacleRenderers;
        private MaterialPropertyBlock receptaclePropertyBlock;
        private PhysicsMaterial gripMaterial;
        private bool placementComplete;
        private G1Graspable placementCandidate;
        private float candidateStableFor;

        public bool Available => environmentRoot != null && tableCollider != null;
        public float TableBaseHeight => tableCollider != null ? tableCollider.bounds.min.y : 0f;
        public string TaskStatus => placementComplete
            ? $"PLACED: {placementCandidate?.DisplayName ?? "object"}"
            : placementCandidate != null ? "Hold object inside bin" : "Place cube or cylinder in bin";

        private void Awake()
        {
            var root = FindSceneObject("[G1] Manipulation Test");
            if (root == null)
                return;
            environmentRoot = root.transform;
            root.SetActive(true);
            var table = FindChild(environmentRoot, "Manipulation Table");
            tableCollider = table != null ? table.GetComponent<Collider>() : null;
            if (tableCollider == null)
                return;

            gripMaterial = new PhysicsMaterial("G1 Manipulation Grip")
            {
                dynamicFriction = 0.75f,
                staticFriction = 0.9f,
                bounciness = 0.02f,
                frictionCombine = PhysicsMaterialCombine.Maximum,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
            tableCollider.material = gripMaterial;
            tableCollider.contactOffset = 0.006f;

            var existing = root.GetComponentsInChildren<G1Graspable>(true);
            Array.Sort(existing, (a, b) => string.CompareOrdinal(a.name, b.name));
            for (var i = 0; i < existing.Length; i++)
                existing[i].gameObject.SetActive(i == 0);
            if (existing.Length > 0)
            {
                existing[0].SetDisplayName("Pick Cube");
                RegisterTaskObject(existing[0], new Vector2(-0.11f, -0.09f));
            }

            var cylinder = CreateCylinder();
            RegisterTaskObject(cylinder, new Vector2(0.1f, -0.09f));
            CreateReceptacle();
            ResetObjectsOnTable();
        }

        private void Update()
        {
            if (!Available || goalVolume == null || placementComplete)
                return;

            TaskObject inside = null;
            foreach (var item in taskObjects)
                if (IsInsideReceptacle(item))
                {
                    inside = item;
                    break;
                }

            if (inside == null)
            {
                placementCandidate = null;
                candidateStableFor = 0f;
                return;
            }

            if (placementCandidate != inside.Graspable)
            {
                placementCandidate = inside.Graspable;
                candidateStableFor = 0f;
            }
            var graspController = GetComponent<G1SimulatedGraspController>();
            var released = graspController == null || !graspController.IsHolding(inside.Graspable);
            var stable = released && inside.Body.linearVelocity.sqrMagnitude < 0.0625f &&
                         inside.Body.angularVelocity.sqrMagnitude < 9f;
            candidateStableFor = stable ? candidateStableFor + Time.deltaTime : 0f;
            if (candidateStableFor < 0.3f)
                return;

            placementComplete = true;
            SetReceptacleColor(new Color(0.15f, 1f, 0.3f));
            StartCoroutine(SuccessHaptics());
        }

        public void SetAlignmentActive(bool active)
        {
            foreach (var item in taskObjects)
                if (item.Body != null) item.Body.isKinematic = active;
            if (!active)
                ResetObjectsOnTable();
        }

        public void AlignTableToSurface(Vector3 surfacePoint, float yawDeltaDegrees = 0f)
        {
            if (!Available)
                return;
            environmentRoot.Rotate(Vector3.up, yawDeltaDegrees, Space.World);
            Physics.SyncTransforms();
            var bounds = tableCollider.bounds;
            var tabletopCenter = new Vector3(bounds.center.x, bounds.max.y, bounds.center.z);
            environmentRoot.position += surfacePoint - tabletopCenter;
            Physics.SyncTransforms();
        }

        public void PlaceTabletop(Vector3 tabletopCenter, Quaternion rotation)
        {
            if (!Available)
                return;
            SetAlignmentActive(true);
            environmentRoot.rotation = Quaternion.Euler(0f, rotation.eulerAngles.y, 0f);
            Physics.SyncTransforms();
            var bounds = tableCollider.bounds;
            var currentTopCenter = new Vector3(bounds.center.x, bounds.max.y, bounds.center.z);
            environmentRoot.position += tabletopCenter - currentTopCenter;
            Physics.SyncTransforms();
            SetAlignmentActive(false);
        }

        public void NudgeTable(Vector3 worldDelta, float yawDeltaDegrees)
        {
            if (!Available)
                return;
            environmentRoot.position += worldDelta;
            environmentRoot.Rotate(Vector3.up, yawDeltaDegrees, Space.World);
            Physics.SyncTransforms();
        }

        public void ResetObjectOnTable() => ResetObjectsOnTable();

        public Vector3 ReceptacleRearPoint(Vector3 tabletopCenter, Vector3 forward)
        {
            var outerDepth = BinInnerDepth + BinWallThickness * 2f;
            return tabletopCenter + forward.normalized * (BinCenterForwardOffset - outerDepth * 0.5f);
        }

        public void ResetObjectsOnTable()
        {
            if (!Available)
                return;
            placementComplete = false;
            placementCandidate = null;
            candidateStableFor = 0f;
            SetReceptacleColor(new Color(0.1f, 0.55f, 0.7f));

            var tabletopCenter = TabletopCenter();
            foreach (var item in taskObjects)
            {
                if (item.Graspable == null || item.Body == null || item.Collider == null)
                    continue;
                item.Graspable.transform.SetParent(environmentRoot, true);
                item.Graspable.transform.rotation = environmentRoot.rotation;
                Physics.SyncTransforms();
                var halfHeight = item.Collider.bounds.extents.y;
                var position = tabletopCenter + environmentRoot.right * item.SpawnOffset.x +
                               environmentRoot.forward * item.SpawnOffset.y;
                position.y = tabletopCenter.y + halfHeight + 0.008f;
                item.Graspable.transform.position = position;
                item.Body.linearVelocity = Vector3.zero;
                item.Body.angularVelocity = Vector3.zero;
                item.Body.isKinematic = false;
                item.Body.WakeUp();
            }

            if (receptacleRoot != null)
            {
                receptacleRoot.SetParent(environmentRoot, true);
                receptacleRoot.SetPositionAndRotation(
                    tabletopCenter + environmentRoot.forward * BinCenterForwardOffset,
                    environmentRoot.rotation);
            }
            Physics.SyncTransforms();
        }

        private G1Graspable CreateCylinder()
        {
            var existing = FindChild(environmentRoot, "Pick Cylinder");
            if (existing != null)
            {
                var existingGraspable = existing.GetComponent<G1Graspable>();
                if (existingGraspable != null)
                {
                    existingGraspable.SetDisplayName("Pick Cylinder");
                    return existingGraspable;
                }
            }

            var cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cylinder.name = "Pick Cylinder";
            cylinder.transform.SetParent(environmentRoot, false);
            cylinder.transform.localScale = new Vector3(0.085f, 0.065f, 0.085f);
            var primitiveCollider = cylinder.GetComponent<Collider>();
            primitiveCollider.enabled = false;
            var meshCollider = cylinder.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = cylinder.GetComponent<MeshFilter>().sharedMesh;
            meshCollider.convex = true;
            Destroy(primitiveCollider);
            var renderer = cylinder.GetComponent<Renderer>();
            renderer.sharedMaterial = CreateVisualMaterial("Cylinder Orange", new Color(0.95f, 0.38f, 0.08f));
            cylinder.AddComponent<Rigidbody>();
            var graspable = cylinder.AddComponent<G1Graspable>();
            graspable.SetDisplayName("Pick Cylinder");
            return graspable;
        }

        private void RegisterTaskObject(G1Graspable graspable, Vector2 spawnOffset)
        {
            if (graspable == null)
                return;
            graspable.gameObject.SetActive(true);
            var body = graspable.GetComponent<Rigidbody>();
            Collider collider = null;
            foreach (var candidate in graspable.GetComponents<Collider>())
                if (candidate.enabled && !candidate.isTrigger)
                {
                    collider = candidate;
                    break;
                }
            if (body == null || collider == null)
                return;
            body.mass = 0.2f;
            body.linearDamping = 0.35f;
            body.angularDamping = 0.8f;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            body.solverIterations = 16;
            body.solverVelocityIterations = 8;
            body.maxAngularVelocity = 12f;
            body.maxDepenetrationVelocity = 2f;
            foreach (var childCollider in graspable.GetComponentsInChildren<Collider>())
            {
                childCollider.material = gripMaterial;
                childCollider.contactOffset = 0.002f;
            }
            taskObjects.Add(new TaskObject(graspable, body, collider, spawnOffset));
        }

        private void CreateReceptacle()
        {
            var existing = FindChild(environmentRoot, "Placement Bin");
            if (existing != null)
            {
                receptacleRoot = existing;
                var existingGoal = FindChild(receptacleRoot, "Placement Goal");
                goalVolume = existingGoal != null ? existingGoal.GetComponent<BoxCollider>() : null;
                receptacleRenderers = receptacleRoot.GetComponentsInChildren<Renderer>();
                receptaclePropertyBlock = new MaterialPropertyBlock();
                if (goalVolume != null)
                    return;
            }
            receptacleRoot = new GameObject("Placement Bin").transform;
            receptacleRoot.SetParent(environmentRoot, false);
            var material = CreateVisualMaterial("Placement Bin Blue", new Color(0.1f, 0.55f, 0.7f));
            var outerWidth = BinInnerWidth + BinWallThickness * 2f;
            var outerDepth = BinInnerDepth + BinWallThickness * 2f;
            CreateBinPart("Bin Floor", new Vector3(0f, BinFloorThickness * 0.5f, 0f),
                new Vector3(outerWidth, BinFloorThickness, outerDepth), material);
            CreateBinPart("Bin Left Wall", new Vector3(-outerWidth * 0.5f + BinWallThickness * 0.5f,
                    BinWallHeight * 0.5f, 0f),
                new Vector3(BinWallThickness, BinWallHeight, outerDepth), material);
            CreateBinPart("Bin Right Wall", new Vector3(outerWidth * 0.5f - BinWallThickness * 0.5f,
                    BinWallHeight * 0.5f, 0f),
                new Vector3(BinWallThickness, BinWallHeight, outerDepth), material);
            CreateBinPart("Bin Front Wall", new Vector3(0f, BinWallHeight * 0.5f,
                    -outerDepth * 0.5f + BinWallThickness * 0.5f),
                new Vector3(BinInnerWidth, BinWallHeight, BinWallThickness), material);
            CreateBinPart("Bin Back Wall", new Vector3(0f, BinWallHeight * 0.5f,
                    outerDepth * 0.5f - BinWallThickness * 0.5f),
                new Vector3(BinInnerWidth, BinWallHeight, BinWallThickness), material);

            var generatedGoal = new GameObject("Placement Goal");
            generatedGoal.transform.SetParent(receptacleRoot, false);
            goalVolume = generatedGoal.AddComponent<BoxCollider>();
            goalVolume.isTrigger = true;
            goalVolume.size = new Vector3(BinInnerWidth - 0.012f, BinWallHeight, BinInnerDepth - 0.012f);
            goalVolume.center = new Vector3(0f, BinFloorThickness + BinWallHeight * 0.5f, 0f);
            receptacleRenderers = receptacleRoot.GetComponentsInChildren<Renderer>();
            receptaclePropertyBlock = new MaterialPropertyBlock();
        }

        private void CreateBinPart(string objectName, Vector3 localPosition, Vector3 localScale, Material material)
        {
            var part = GameObject.CreatePrimitive(PrimitiveType.Cube);
            part.name = objectName;
            part.transform.SetParent(receptacleRoot, false);
            part.transform.localPosition = localPosition;
            part.transform.localScale = localScale;
            part.GetComponent<Renderer>().sharedMaterial = material;
            var collider = part.GetComponent<Collider>();
            collider.material = gripMaterial;
            collider.contactOffset = 0.006f;
        }

        private bool IsInsideReceptacle(TaskObject item)
        {
            if (item.Collider == null || goalVolume == null)
                return false;
            var localCenter = receptacleRoot.InverseTransformPoint(item.Collider.bounds.center);
            var horizontalMargin = item.Graspable.name.Contains("Cylinder") ? 0.045f : 0.052f;
            return Mathf.Abs(localCenter.x) <= BinInnerWidth * 0.5f - horizontalMargin &&
                   Mathf.Abs(localCenter.z) <= BinInnerDepth * 0.5f - horizontalMargin &&
                   localCenter.y >= BinFloorThickness && localCenter.y <= BinWallHeight + 0.04f;
        }

        private Vector3 TabletopCenter()
        {
            Physics.SyncTransforms();
            var bounds = tableCollider.bounds;
            return new Vector3(bounds.center.x, bounds.max.y, bounds.center.z);
        }

        private void SetReceptacleColor(Color color)
        {
            if (receptacleRenderers == null || receptaclePropertyBlock == null)
                return;
            foreach (var renderer in receptacleRenderers)
            {
                renderer.GetPropertyBlock(receptaclePropertyBlock);
                receptaclePropertyBlock.SetColor("_BaseColor", color);
                receptaclePropertyBlock.SetColor("_Color", color);
                renderer.SetPropertyBlock(receptaclePropertyBlock);
            }
        }

        private IEnumerator SuccessHaptics()
        {
            OVRInput.SetControllerVibration(0.35f, 0.45f, OVRInput.Controller.LTouch);
            OVRInput.SetControllerVibration(0.35f, 0.45f, OVRInput.Controller.RTouch);
            yield return new WaitForSecondsRealtime(0.16f);
            OVRInput.SetControllerVibration(0f, 0f, OVRInput.Controller.LTouch);
            OVRInput.SetControllerVibration(0f, 0f, OVRInput.Controller.RTouch);
        }

        private static Material CreateVisualMaterial(string materialName, Color color)
        {
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            var material = new Material(shader) { name = materialName };
            material.SetColor(material.HasProperty("_BaseColor") ? "_BaseColor" : "_Color", color);
            return material;
        }

        private static Transform FindChild(Transform parent, string objectName)
        {
            foreach (var child in parent.GetComponentsInChildren<Transform>(true))
                if (child.name == objectName) return child;
            return null;
        }

        private static GameObject FindSceneObject(string objectName)
        {
            foreach (var candidate in Resources.FindObjectsOfTypeAll<GameObject>())
                if (candidate.name == objectName && candidate.scene.IsValid()) return candidate;
            return null;
        }

        private sealed class TaskObject
        {
            public readonly G1Graspable Graspable;
            public readonly Rigidbody Body;
            public readonly Collider Collider;
            public readonly Vector2 SpawnOffset;

            public TaskObject(G1Graspable graspable, Rigidbody body, Collider collider, Vector2 spawnOffset)
            {
                Graspable = graspable;
                Body = body;
                Collider = collider;
                SpawnOffset = spawnOffset;
            }
        }
    }
}
