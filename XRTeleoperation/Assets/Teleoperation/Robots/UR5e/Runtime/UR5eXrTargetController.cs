using UnityEngine;
using UnityEngine.UI;

namespace Teleoperation.Robots.UR5e
{
    public enum TeleoperationMode
    {
        Placement,
        InverseKinematics,
        ForwardKinematics,
        Navigation
    }

    /// <summary>
    /// Quest-controller ray grab with clutch, reset, and emergency-stop semantics.
    /// Uses the controller anchors supplied by Meta's Camera Rig building block.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class UR5eXrTargetController : MonoBehaviour
    {
        [SerializeField] private UR5ePositionIkSolver solver;
        [SerializeField] private UR5eJointController robot;
        [SerializeField] private Transform target;
        [SerializeField, Min(0.05f)] private float selectionRadius = 0.18f;
        [SerializeField, Min(0.25f)] private float maximumRayDistance = 5f;
        [SerializeField, Range(0.1f, 0.95f)] private float triggerThreshold = 0.65f;
        [SerializeField] private TeleoperationMode mode = TeleoperationMode.InverseKinematics;
        [SerializeField, Min(0.05f)] private float placementMoveSpeed = 0.6f;
        [SerializeField, Min(1f)] private float placementRotationSpeed = 60f;
        [SerializeField, Min(0.05f)] private float placementVerticalSpeed = 0.35f;
        [SerializeField, Min(1f)] private float fkSpeedDegreesPerSecond = 30f;
        [SerializeField, Range(0.1f, 0.95f)] private float stickDeadzone = 0.2f;
        [SerializeField, Range(0.2f, 0.95f)] private float jointSelectionThreshold = 0.7f;

        private Transform leftController;
        private Transform rightController;
        private Transform activeController;
        private Vector3 targetLocalPosition;
        private Quaternion targetLocalRotation;
        private bool leftWasPressed;
        private bool rightWasPressed;
        private bool emergencyStopped;
        private bool selectionStickLatched;
        private Transform head;
        private GameObject locomotor;
        private GameObject placementMarker;
        private Text modeHudText;

        public TeleoperationMode Mode => mode;

        private void Awake()
        {
            solver ??= GetComponent<UR5ePositionIkSolver>();
            robot ??= GetComponent<UR5eJointController>();
            target ??= solver != null ? solver.Target : null;
            leftController = FindTransform("LeftControllerAnchor");
            rightController = FindTransform("RightControllerAnchor");
            head = FindTransform("CenterEyeAnchor");
            locomotor = FindSceneObject("Locomotor");
            CreatePlacementMarker();
            CreateModeHud();
            if (solver != null)
                solver.Solve = false;
            EnterMode(mode);
        }

        private void Update()
        {
            if (target == null || solver == null || robot == null)
                return;

            if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch))
                ResetTarget();
            if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch))
                ToggleEmergencyStop();
            if (OVRInput.GetDown(OVRInput.Button.Start, OVRInput.Controller.LTouch))
                CycleMode();

            if (!emergencyStopped)
            {
                switch (mode)
                {
                    case TeleoperationMode.Placement:
                        UpdatePlacementMode();
                        break;
                    case TeleoperationMode.ForwardKinematics:
                        UpdateForwardKinematicsMode();
                        break;
                }
            }

            var leftPressed = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch) >= triggerThreshold;
            var rightPressed = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch) >= triggerThreshold;

            if (mode == TeleoperationMode.InverseKinematics && !emergencyStopped && activeController == null)
            {
                if (leftPressed && !leftWasPressed)
                    TryBeginGrab(leftController);
                else if (rightPressed && !rightWasPressed)
                    TryBeginGrab(rightController);
            }

            if (activeController != null)
            {
                var stillPressed = activeController == leftController ? leftPressed : rightPressed;
                if (stillPressed && !emergencyStopped)
                {
                    target.SetPositionAndRotation(
                        activeController.TransformPoint(targetLocalPosition),
                        activeController.rotation * targetLocalRotation);
                }
                else
                {
                    EndGrab();
                }
            }

            leftWasPressed = leftPressed;
            rightWasPressed = rightPressed;
            UpdateModeHud();
        }

        private void TryBeginGrab(Transform controller)
        {
            if (controller == null)
                return;

            var toTarget = target.position - controller.position;
            var distanceAlongRay = Vector3.Dot(toTarget, controller.forward);
            if (distanceAlongRay < 0f || distanceAlongRay > maximumRayDistance)
                return;

            var closestPoint = controller.position + controller.forward * distanceAlongRay;
            if (Vector3.Distance(closestPoint, target.position) > selectionRadius)
                return;

            activeController = controller;
            targetLocalPosition = controller.InverseTransformPoint(target.position);
            targetLocalRotation = Quaternion.Inverse(controller.rotation) * target.rotation;
            solver.Solve = true;
        }

        private void EndGrab()
        {
            activeController = null;
            solver.Solve = false;
            robot.HoldCurrentPose();
        }

        private void ResetTarget()
        {
            EndGrab();
            emergencyStopped = false;
            solver.ResetTargetToEndEffector();
        }

        private void CycleMode()
        {
            var next = (TeleoperationMode)(((int)mode + 1) % System.Enum.GetValues(typeof(TeleoperationMode)).Length);
            EnterMode(next);
        }

        private void EnterMode(TeleoperationMode newMode)
        {
            EndGrab();
            mode = newMode;
            selectionStickLatched = false;

            // The Meta comprehensive interaction rig reads the same thumbsticks
            // as the robot controller. Give it exclusive ownership only while
            // navigating, which also keeps its comfort tunnel out of robot modes.
            if (locomotor != null)
                locomotor.SetActive(mode == TeleoperationMode.Navigation);
            if (placementMarker != null)
                placementMarker.SetActive(false);

            if (mode == TeleoperationMode.InverseKinematics)
                solver.ResetTargetToEndEffector();
            else if (target != null && solver.EndEffector != null)
                target.SetPositionAndRotation(solver.EndEffector.position, solver.EndEffector.rotation);
        }

        private void UpdatePlacementMode()
        {
            var leftStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch));
            var rightStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch));

            var referenceForward = head != null ? Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized : Vector3.forward;
            if (referenceForward.sqrMagnitude < 0.01f)
                referenceForward = Vector3.forward;
            var referenceRight = Vector3.Cross(Vector3.up, referenceForward).normalized;

            var planarDelta = (referenceRight * leftStick.x + referenceForward * leftStick.y) *
                              (placementMoveSpeed * Time.deltaTime);
            var verticalDelta = Vector3.up * (rightStick.y * placementVerticalSpeed * Time.deltaTime);
            var yawDelta = rightStick.x * placementRotationSpeed * Time.deltaTime;
            robot.MoveBase(planarDelta + verticalDelta, yawDelta);
            solver.ResetTargetToEndEffector();
            UpdateDirectPlacement();
        }

        private void UpdateDirectPlacement()
        {
            var controller = rightController != null ? rightController : leftController;
            var hasPoint = controller != null;

            if (placementMarker != null)
                placementMarker.SetActive(hasPoint);
            if (!hasPoint)
                return;

            // Use the physical Touch controller's tracked world pose as a probe.
            // The user can touch a real surface, then tap the index trigger to
            // place the robot base at that measured location.
            var point = controller.position;
            placementMarker.transform.position = point;

            var trigger = controller == rightController
                ? OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch)
                : OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
            var wasPressed = controller == rightController ? rightWasPressed : leftWasPressed;
            if (trigger >= triggerThreshold && !wasPressed)
            {
                robot.PlaceBase(point);
                solver.ResetTargetToEndEffector();
            }
        }

        private void UpdateForwardKinematicsMode()
        {
            var leftStick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
            var rightStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch));

            if (!selectionStickLatched && Mathf.Abs(leftStick.x) >= jointSelectionThreshold)
            {
                var direction = leftStick.x > 0f ? 1 : -1;
                robot.SelectJoint((robot.SelectedJointIndex + direction + robot.JointCount) % robot.JointCount);
                selectionStickLatched = true;
            }
            else if (Mathf.Abs(leftStick.x) < stickDeadzone)
            {
                selectionStickLatched = false;
            }

            if (Mathf.Abs(rightStick.y) > 0f)
            {
                var index = robot.SelectedJointIndex;
                robot.SetTargetDegrees(
                    index,
                    robot.GetTargetDegrees(index) + rightStick.y * fkSpeedDegreesPerSecond * Time.deltaTime);
                robot.ApplyTargets();
            }
        }

        private Vector2 ApplyDeadzone(Vector2 value)
        {
            return value.magnitude < stickDeadzone ? Vector2.zero : value;
        }

        private void ToggleEmergencyStop()
        {
            emergencyStopped = !emergencyStopped;
            if (emergencyStopped)
                EndGrab();
        }

        private void CreateModeHud()
        {
            if (head == null)
                return;

            var existing = head.Find("Teleoperation Mode HUD");
            if (existing != null)
            {
                modeHudText = existing.GetComponentInChildren<Text>(true);
                return;
            }

            var canvasObject = new GameObject("Teleoperation Mode HUD", typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler));
            canvasObject.transform.SetParent(head, false);
            canvasObject.transform.localPosition = new Vector3(0f, -0.22f, 0.65f);
            canvasObject.transform.localRotation = Quaternion.identity;
            canvasObject.transform.localScale = Vector3.one * 0.0012f;

            var canvas = canvasObject.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.sortingOrder = 100;
            var canvasRect = canvasObject.GetComponent<RectTransform>();
            canvasRect.sizeDelta = new Vector2(620f, 110f);

            var backgroundObject = new GameObject("Background", typeof(RectTransform), typeof(Image));
            backgroundObject.transform.SetParent(canvasObject.transform, false);
            var backgroundRect = backgroundObject.GetComponent<RectTransform>();
            backgroundRect.anchorMin = Vector2.zero;
            backgroundRect.anchorMax = Vector2.one;
            backgroundRect.offsetMin = Vector2.zero;
            backgroundRect.offsetMax = Vector2.zero;
            backgroundObject.GetComponent<Image>().color = new Color(0.02f, 0.03f, 0.05f, 0.82f);

            var textObject = new GameObject("Mode", typeof(RectTransform), typeof(Text));
            textObject.transform.SetParent(backgroundObject.transform, false);
            var textRect = textObject.GetComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = new Vector2(18f, 8f);
            textRect.offsetMax = new Vector2(-18f, -8f);

            modeHudText = textObject.GetComponent<Text>();
            modeHudText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            modeHudText.fontSize = 28;
            modeHudText.alignment = TextAnchor.MiddleCenter;
            modeHudText.color = Color.white;
            modeHudText.horizontalOverflow = HorizontalWrapMode.Wrap;
            modeHudText.verticalOverflow = VerticalWrapMode.Truncate;
            UpdateModeHud();
        }

        private void CreatePlacementMarker()
        {
            placementMarker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            placementMarker.name = "UR5e Placement Marker";
            placementMarker.transform.localScale = new Vector3(0.12f, 0.003f, 0.12f);
            Destroy(placementMarker.GetComponent<Collider>());

            var renderer = placementMarker.GetComponent<Renderer>();
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null)
            {
                renderer.material = new Material(shader);
                renderer.material.color = new Color(0.1f, 1f, 0.35f, 0.8f);
            }

            placementMarker.SetActive(false);
        }

        private void UpdateModeHud()
        {
            if (modeHudText == null)
                return;

            if (emergencyStopped)
            {
                modeHudText.text = "EMERGENCY STOP\nRight B: clear";
                modeHudText.color = new Color(1f, 0.25f, 0.2f);
                return;
            }

            modeHudText.color = mode switch
            {
                TeleoperationMode.Placement => new Color(0.3f, 0.75f, 1f),
                TeleoperationMode.ForwardKinematics => new Color(1f, 0.65f, 0.15f),
                TeleoperationMode.Navigation => new Color(0.75f, 0.45f, 1f),
                _ => new Color(0.3f, 1f, 0.45f)
            };

            modeHudText.text = mode switch
            {
                TeleoperationMode.Placement => "PLACEMENT\nTouch location + trigger | Sticks: adjust",
                TeleoperationMode.ForwardKinematics => $"FK — {robot.SelectedJointName}\nL stick: select | R stick: move joint",
                TeleoperationMode.Navigation => "NAVIGATION\nSticks move / turn your viewpoint",
                _ => activeController != null
                    ? "IK — COMMANDING\nRelease trigger to clutch"
                    : "IK\nPoint + trigger to command | A: reset"
            };
        }

        private static Transform FindTransform(string objectName)
        {
            var found = GameObject.Find(objectName);
            return found != null ? found.transform : null;
        }

        private static GameObject FindSceneObject(string objectName)
        {
            foreach (var candidate in Resources.FindObjectsOfTypeAll<GameObject>())
            {
                if (candidate.name == objectName && candidate.scene.IsValid())
                    return candidate;
            }

            return null;
        }

        private void OnGUI()
        {
            var status = emergencyStopped
                ? "EMERGENCY STOPPED — press B to clear"
                : activeController != null
                    ? "IK ACTIVE — release trigger to clutch"
                    : mode switch
                    {
                        TeleoperationMode.Placement => "PLACEMENT — touch location + trigger | sticks adjust",
                        TeleoperationMode.ForwardKinematics => $"FK — L stick select | R stick move | {robot.SelectedJointName}",
                        TeleoperationMode.Navigation => "NAVIGATION — sticks move/turn viewpoint",
                        _ => "IK — point + trigger | A reset | B emergency stop"
                    };
            GUI.Box(new Rect(20f, 20f, 520f, 32f), $"{mode} | {status} | Left Menu: next mode");
        }
    }
}
