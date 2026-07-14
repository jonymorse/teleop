using System.Collections;
using UnityEngine;
using UnityEngine.Rendering.Universal;
using UnityEngine.UI;

namespace Teleoperation.Robots.G1
{
    public enum G1OperatorMode
    {
        Teleoperate, BodyTracking, FingerTracking, BackendTeleoperation, QuestHandRetargeting, Navigation,
        RobotAlignment, TableAlignment, Home
    }

    [DisallowMultipleComponent]
    [RequireComponent(typeof(G1DualArmIkController))]
    public sealed class G1XrTeleoperationController : MonoBehaviour
    {
        private static readonly G1OperatorMode[] PrimaryModes =
        {
            G1OperatorMode.BackendTeleoperation, G1OperatorMode.QuestHandRetargeting,
            G1OperatorMode.Navigation,
            G1OperatorMode.RobotAlignment, G1OperatorMode.TableAlignment
        };
        private static readonly G1OperatorMode[] DebugModes =
        {
            G1OperatorMode.Teleoperate, G1OperatorMode.BodyTracking,
            G1OperatorMode.FingerTracking, G1OperatorMode.BackendTeleoperation,
            G1OperatorMode.QuestHandRetargeting,
            G1OperatorMode.Navigation, G1OperatorMode.RobotAlignment,
            G1OperatorMode.TableAlignment
        };
        [SerializeField, Range(0.1f, 0.95f)] private float gripThreshold = 0.65f;
        [SerializeField, Range(0.25f, 2f)] private float motionScale = 1f;
        [SerializeField, Range(0.05f, 0.75f)] private float precisionScale = 0.25f;
        [SerializeField, Min(0.05f)] private float placementMoveSpeed = 0.6f;
        [SerializeField, Min(1f)] private float placementRotationSpeed = 60f;
        [SerializeField, Min(0.05f)] private float placementVerticalSpeed = 0.35f;
        [SerializeField] private G1OperatorMode mode = G1OperatorMode.Teleoperate;
        [SerializeField] private bool includeDebugModesInCycle;
        [Header("Headset-relative startup layout")]
        [SerializeField] private bool autoLayoutOnStart = true;
        [SerializeField, Min(0.5f)] private float workspaceRightDistance = 1.25f;
        [SerializeField, Min(0.1f)] private float robotBehindTableDistance = 0.35f;
        [SerializeField, Min(0.1f)] private float tableInFrontDistance = 0.35f;
        [SerializeField, Range(-0.5f, 0.8f)] private float robotCenterBelowHead = 0.15f;
        [SerializeField, Range(0.1f, 1f)] private float tabletopBelowHead = 0.42f;
        [SerializeField, HideInInspector] private int editorPreviewLayoutVersion;
        [Header("Robot egocentric camera")]
        [SerializeField, Range(60f, 110f)] private float robotCameraFieldOfView = 90f;
        [SerializeField, Range(256, 1024)] private int robotCameraWidth = 768;
        [SerializeField, Range(144, 576)] private int robotCameraHeight = 432;

        private G1DualArmIkController ik;
        private Transform leftController;
        private Transform rightController;
        private Transform head;
        private GameObject locomotor;
        private Text hud;
        private CanvasGroup hudCanvasGroup;
        private bool hudVisible;
        private float hudFullVisibilityUntil;
        private Camera robotViewCamera;
        private RenderTexture robotViewTexture;
        private GameObject robotViewPanel;
        private bool robotViewVisible;
        private GameObject handModeMenu;
        private Text handModeMenuText;
        private CanvasGroup handModeMenuCanvasGroup;
        private bool handMenuVisible;
        private float handMenuFacingSince = -1f;
        private float handMenuLastFacingAt = -999f;
        private HandMenuAction pendingHandAction;
        private float handActionHeldSince = -1f;
        private bool handMenuLatched;
        private bool calibrated;
        private bool controlArmed;
        private float questHandCalibrationUntil;
        private bool emergencyStopped;
        private bool leftClutched;
        private bool rightClutched;
        private bool leftOrientationMode;
        private bool rightOrientationMode;
        private bool rightIndexWasPressed;
        private GameObject placementMarker;
        private G1BodyTrackingController bodyTracking;
        private G1TeleoperationBridge backendBridge;
        private G1Dex3HandController dex3Hands;
        private G1ManipulationEnvironment manipulationEnvironment;
        private G1JointController robot;
        private bool homeComplete;
        private bool returnFromHome;
        private float returnFromHomeAt;
        private G1OperatorMode modeBeforeHome = G1OperatorMode.BackendTeleoperation;
        private float pelvisHeightAboveFeet = 0.8f;
        private Vector3 leftControllerOrigin;
        private Vector3 rightControllerOrigin;
        private Vector3 leftTargetOrigin;
        private Vector3 rightTargetOrigin;
        private Quaternion leftControllerRotationOrigin;
        private Quaternion rightControllerRotationOrigin;
        private Quaternion leftTargetRotationOrigin;
        private Quaternion rightTargetRotationOrigin;
        private float leftGripperCommand;
        private float rightGripperCommand;

        public float LeftGripperCommand => leftGripperCommand;
        public float RightGripperCommand => rightGripperCommand;

        private void Start()
        {
            ik = GetComponent<G1DualArmIkController>();
            robot = GetComponent<G1JointController>();
            bodyTracking = GetComponent<G1BodyTrackingController>() ?? gameObject.AddComponent<G1BodyTrackingController>();
            backendBridge = GetComponent<G1TeleoperationBridge>() ?? gameObject.AddComponent<G1TeleoperationBridge>();
            dex3Hands = GetComponent<G1Dex3HandController>() ?? gameObject.AddComponent<G1Dex3HandController>();
            manipulationEnvironment = GetComponent<G1ManipulationEnvironment>() ?? gameObject.AddComponent<G1ManipulationEnvironment>();
            var grasp = GetComponent<G1SimulatedGraspController>() ?? gameObject.AddComponent<G1SimulatedGraspController>();
            grasp.enabled = true;
            leftController = FindTransform("LeftControllerAnchor");
            rightController = FindTransform("RightControllerAnchor");
            head = FindTransform("CenterEyeAnchor");
            locomotor = FindSceneObject("Locomotor");
            CreatePlacementMarker();
            MeasurePelvisHeight();
            CreateHud();
            CreateHandModeMenu();
            CreateRobotView();
            ik.Solve = false;
            if (!includeDebugModesInCycle &&
                (mode == G1OperatorMode.Teleoperate || mode == G1OperatorMode.BodyTracking ||
                 mode == G1OperatorMode.FingerTracking || mode == G1OperatorMode.Home))
                mode = G1OperatorMode.BackendTeleoperation;
            SetMode(mode);
            if (autoLayoutOnStart)
                StartCoroutine(InitializeHeadsetRelativeLayout());
        }

        private IEnumerator InitializeHeadsetRelativeLayout()
        {
            // On a Quest cold start the HMD can be reported as present before the floor-relative
            // tracking pose is ready. Placing from that temporary (0, 0, 0) pose puts everything
            // at the tracking-origin floor and it previously stayed there for the entire session.
            const int stableFramesRequired = 12;
            const float stablePositionTolerance = 0.025f;
            var timeout = Time.realtimeSinceStartup + 10f;
            var stableFrames = 0;
            var previousPosition = Vector3.zero;
            var havePreviousPosition = false;

            while (Time.realtimeSinceStartup < timeout && stableFrames < stableFramesRequired)
            {
                if (HasValidFloorRelativeHeadPose())
                {
                    if (havePreviousPosition &&
                        Vector3.Distance(previousPosition, head.position) <= stablePositionTolerance)
                        stableFrames++;
                    else
                        stableFrames = 0;

                    previousPosition = head.position;
                    havePreviousPosition = true;
                }
                else
                {
                    stableFrames = 0;
                    havePreviousPosition = false;
                }

                yield return null;
            }

            if (!HasValidFloorRelativeHeadPose())
            {
                Debug.LogWarning("G1 startup layout skipped because no valid floor-relative headset pose was received. " +
                                 "Once tracking is active, press A in an alignment mode to align manually.");
                yield break;
            }

            ApplyHeadsetRelativeLayout();

            // OpenXR can adjust the tracking origin once more just after its first usable pose.
            // Watch briefly and correct only a large, stable origin shift (not normal head motion).
            var appliedHeadPosition = head.position;
            var correctionDeadline = Time.realtimeSinceStartup + 2f;
            stableFrames = 0;
            havePreviousPosition = false;
            while (Time.realtimeSinceStartup < correctionDeadline)
            {
                if (HasValidFloorRelativeHeadPose() &&
                    Vector3.Distance(appliedHeadPosition, head.position) >= 0.25f)
                {
                    if (havePreviousPosition &&
                        Vector3.Distance(previousPosition, head.position) <= stablePositionTolerance)
                        stableFrames++;
                    else
                        stableFrames = 0;

                    previousPosition = head.position;
                    havePreviousPosition = true;
                    if (stableFrames >= stableFramesRequired)
                    {
                        ApplyHeadsetRelativeLayout();
                        yield break;
                    }
                }
                else
                {
                    stableFrames = 0;
                    havePreviousPosition = false;
                }

                yield return null;
            }
        }

        private bool HasValidFloorRelativeHeadPose()
        {
            if (head == null || head.position.y < 0.4f)
                return false;

            return Vector3.ProjectOnPlane(head.forward, Vector3.up).sqrMagnitude > 0.01f;
        }

        private void ApplyHeadsetRelativeLayout()
        {
            if (head == null || robot == null)
                return;
            // Keep the main view in front of the operator clear: the table and robot sit to
            // their right while sharing the operator's horizontal forward orientation.
            var layoutDirection = Vector3.ProjectOnPlane(head.right, Vector3.up).normalized;
            if (layoutDirection.sqrMagnitude < 0.01f) layoutDirection = Vector3.right;
            var playerForward = Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized;
            if (playerForward.sqrMagnitude < 0.01f) playerForward = Vector3.forward;
            var playerFacing = Quaternion.LookRotation(playerForward, Vector3.up);

            var tabletopCenter = head.position + layoutDirection * workspaceRightDistance +
                                 playerForward * tableInFrontDistance -
                                 Vector3.up * tabletopBelowHead;
            if (manipulationEnvironment != null && manipulationEnvironment.Available)
                manipulationEnvironment.PlaceTabletop(tabletopCenter, playerFacing);

            if (robot.TryGetRootBody(out var root))
            {
                root.TeleportRoot(root.transform.position, playerFacing);
                Physics.SyncTransforms();
                var renderers = robot.GetComponentsInChildren<Renderer>(true);
                if (renderers.Length > 0)
                {
                    var bounds = renderers[0].bounds;
                    for (var i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
                    var desiredCenter = head.position + layoutDirection * workspaceRightDistance -
                                        playerForward * robotBehindTableDistance -
                                        Vector3.up * robotCenterBelowHead;
                    var delta = layoutDirection * Vector3.Dot(desiredCenter - bounds.center, layoutDirection) +
                                playerForward * Vector3.Dot(desiredCenter - bounds.center, playerForward);
                    if (TryGetFootBottom(renderers, out var footBottom) &&
                        manipulationEnvironment != null && manipulationEnvironment.Available)
                        delta += Vector3.up * (manipulationEnvironment.TableBaseHeight - footBottom);
                    else
                        delta += Vector3.up * (desiredCenter.y - bounds.center.y);
                    root.TeleportRoot(root.transform.position + delta, playerFacing);
                }
            }
            Physics.SyncTransforms();
            ik.ResetTargetsToHands();
        }

        private static bool TryGetFootBottom(Renderer[] renderers, out float bottom)
        {
            bottom = float.PositiveInfinity;
            var found = false;
            foreach (var renderer in renderers)
            {
                if (!HasAncestor(renderer.transform, "left_ankle_roll_link") &&
                    !HasAncestor(renderer.transform, "right_ankle_roll_link"))
                    continue;
                bottom = Mathf.Min(bottom, renderer.bounds.min.y);
                found = true;
            }
            return found;
        }

        private static bool HasAncestor(Transform transform, string objectName)
        {
            while (transform != null)
            {
                if (transform.name == objectName) return true;
                transform = transform.parent;
            }
            return false;
        }

        private void Update()
        {
            if (ik == null)
                return;

            if (OVRInput.GetDown(OVRInput.Button.Start, OVRInput.Controller.LTouch))
            {
                var reverse = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger,
                    OVRInput.Controller.LTouch) >= gripThreshold;
                CycleMode(reverse ? -1 : 1);
            }

            if (OVRInput.GetDown(OVRInput.RawButton.X, OVRInput.Controller.LTouch) && !emergencyStopped)
                StartHomeAction();

            if (!controlArmed &&
                OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch) < gripThreshold &&
                OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.LTouch))
            {
                hudVisible = !hudVisible;
                hudFullVisibilityUntil = Time.unscaledTime + 2f;
            }

            if (OVRInput.GetDown(OVRInput.RawButton.Y, OVRInput.Controller.LTouch))
                SetRobotViewVisible(!robotViewVisible);

            UpdateHandModeMenu();

            if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch))
            {
                emergencyStopped = true;
                DisarmControl();
                SetRobotViewVisible(false);
                leftClutched = rightClutched = false;
                ik.Solve = false;
            }

            if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch))
            {
                if (mode == G1OperatorMode.RobotAlignment || mode == G1OperatorMode.TableAlignment)
                    ApplyHeadsetRelativeLayout();
                else if (IsControlMode(mode))
                {
                    if (controlArmed)
                        DisarmControl();
                    else
                        ArmControl();
                }
                else
                    Calibrate();
            }

            if (OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.RTouch) &&
                controlArmed &&
                (mode == G1OperatorMode.BodyTracking || mode == G1OperatorMode.FingerTracking ||
                 mode == G1OperatorMode.BackendTeleoperation ||
                 mode == G1OperatorMode.QuestHandRetargeting) &&
                bodyTracking != null && bodyTracking.ResetNeutralOrientation())
                backendBridge?.RequestSolverReset();

            var leftTracked = IsTracked(OVRInput.Controller.LTouch);
            var rightTracked = IsTracked(OVRInput.Controller.RTouch);
            var trackingValid = leftTracked && rightTracked;

            if (mode == G1OperatorMode.RobotAlignment && !emergencyStopped && rightTracked)
            {
                ik.Solve = false;
                leftClutched = rightClutched = false;
                UpdatePlacement();
            }
            else if (mode == G1OperatorMode.TableAlignment && !emergencyStopped && rightTracked)
            {
                ik.Solve = false;
                leftClutched = rightClutched = false;
                UpdateTableAlignment();
            }
            else if (mode == G1OperatorMode.Home && !emergencyStopped)
            {
                ik.Solve = false;
                leftClutched = rightClutched = false;
                UpdateHome();
            }
            else if (mode == G1OperatorMode.Teleoperate && controlArmed && calibrated && !emergencyStopped && trackingValid)
            {
                ik.Solve = true;
                UpdateArm(leftController, ik.LeftTarget, OVRInput.Controller.LTouch, ref leftClutched,
                    ref leftOrientationMode, true,
                    ref leftControllerOrigin, ref leftTargetOrigin, ref leftControllerRotationOrigin,
                    ref leftTargetRotationOrigin);
                UpdateArm(rightController, ik.RightTarget, OVRInput.Controller.RTouch, ref rightClutched,
                    ref rightOrientationMode, false,
                    ref rightControllerOrigin, ref rightTargetOrigin, ref rightControllerRotationOrigin,
                    ref rightTargetRotationOrigin);
                leftGripperCommand = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
                rightGripperCommand = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
            }
            else if (mode == G1OperatorMode.BodyTracking && controlArmed && !emergencyStopped)
            {
                leftClutched = rightClutched = false;
                ik.Solve = bodyTracking != null && bodyTracking.TrackingValid;
                leftGripperCommand = bodyTracking != null ? bodyTracking.LeftPinch : 0f;
                rightGripperCommand = bodyTracking != null ? bodyTracking.RightPinch : 0f;
            }
            else if (mode == G1OperatorMode.FingerTracking && controlArmed && !emergencyStopped)
            {
                leftClutched = rightClutched = false;
                ik.Solve = bodyTracking != null && bodyTracking.TrackingValid;
                if (bodyTracking != null && bodyTracking.FingerTrackingValid)
                    dex3Hands?.SetTrackedCurls(bodyTracking.LeftFingerCurls, bodyTracking.RightFingerCurls);
                leftGripperCommand = bodyTracking != null ? bodyTracking.LeftFingerCurls.y : 0f;
                rightGripperCommand = bodyTracking != null ? bodyTracking.RightFingerCurls.y : 0f;
            }
            else if (mode == G1OperatorMode.BackendTeleoperation && controlArmed && !emergencyStopped)
            {
                leftClutched = rightClutched = false;
                ik.Solve = false;
                if (trackingValid)
                {
                    var leftIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
                    var rightIndex = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
                    var leftMiddle = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.LTouch);
                    var rightMiddle = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, OVRInput.Controller.RTouch);
                    dex3Hands?.SetControllerInputs(
                        OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch),
                        leftIndex, leftMiddle,
                        OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch),
                        rightIndex, rightMiddle);
                    leftGripperCommand = Mathf.Max(leftIndex, leftMiddle);
                    rightGripperCommand = Mathf.Max(rightIndex, rightMiddle);
                }
                else if (bodyTracking != null && bodyTracking.FingerTrackingValid)
                {
                    dex3Hands?.SetTrackedCurls(bodyTracking.LeftFingerCurls, bodyTracking.RightFingerCurls);
                    leftGripperCommand = bodyTracking.LeftPinch;
                    rightGripperCommand = bodyTracking.RightPinch;
                }
                else
                    leftGripperCommand = rightGripperCommand = 0f;
            }
            else if (mode == G1OperatorMode.QuestHandRetargeting && controlArmed && !emergencyStopped)
            {
                leftClutched = rightClutched = false;
                ik.Solve = false;
                leftGripperCommand = bodyTracking != null ? bodyTracking.LeftPinch : 0f;
                rightGripperCommand = bodyTracking != null ? bodyTracking.RightPinch : 0f;
            }
            else
            {
                leftClutched = rightClutched = false;
                leftGripperCommand = rightGripperCommand = 0f;
                ik.Solve = false;
                if ((mode == G1OperatorMode.RobotAlignment || mode == G1OperatorMode.TableAlignment) && placementMarker != null)
                    placementMarker.SetActive(false);
            }

            rightIndexWasPressed = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch) >= gripThreshold;
            UpdateHud(leftTracked, rightTracked);
        }

        private void UpdateTableAlignment()
        {
            if (manipulationEnvironment == null || !manipulationEnvironment.Available)
                return;
            var leftStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch));
            var rightStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch));
            var forward = head != null ? Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized : Vector3.forward;
            if (forward.sqrMagnitude < 0.01f) forward = Vector3.forward;
            var right = Vector3.Cross(Vector3.up, forward).normalized;
            var delta = (right * leftStick.x + forward * leftStick.y) * (placementMoveSpeed * Time.deltaTime) +
                        Vector3.up * (rightStick.y * placementVerticalSpeed * Time.deltaTime);
            manipulationEnvironment.NudgeTable(delta, rightStick.x * placementRotationSpeed * Time.deltaTime);

            if (placementMarker != null && rightController != null)
            {
                placementMarker.SetActive(true);
                placementMarker.transform.position = rightController.position;
                var pressed = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch) >= gripThreshold;
                if (pressed && !rightIndexWasPressed)
                    manipulationEnvironment.AlignTableToSurface(rightController.position);
            }
        }

        private void UpdateHome()
        {
            if (robot == null)
                return;
            if (homeComplete)
            {
                if (returnFromHome && Time.unscaledTime >= returnFromHomeAt)
                {
                    returnFromHome = false;
                    SetMode(modeBeforeHome);
                }
                return;
            }
            var complete = true;
            const float homeSpeedDegreesPerSecond = 45f;
            for (var i = 0; i < robot.JointCount; i++)
            {
                var next = Mathf.MoveTowards(robot.GetTargetDegrees(i), 0f,
                    homeSpeedDegreesPerSecond * Time.deltaTime);
                robot.SetTargetDegrees(i, next);
                complete &= Mathf.Abs(next) < 0.05f;
            }
            robot.ApplyTargets();
            complete &= dex3Hands == null || dex3Hands.MoveHome(Time.deltaTime);
            if (complete)
            {
                homeComplete = true;
                ik.ResetTargetsToHands();
                returnFromHomeAt = Time.unscaledTime + 0.4f;
            }
        }

        private void StartHomeAction()
        {
            if (mode == G1OperatorMode.Home)
                return;
            GetComponent<G1SimulatedGraspController>()?.ReleaseAll();
            manipulationEnvironment?.ResetObjectOnTable();
            modeBeforeHome = mode;
            returnFromHome = true;
            SetMode(G1OperatorMode.Home);
        }

        private void CycleMode(int direction)
        {
            var modes = includeDebugModesInCycle ? DebugModes : PrimaryModes;
            var current = -1;
            for (var i = 0; i < modes.Length; i++)
                if (modes[i] == mode)
                {
                    current = i;
                    break;
                }
            if (current < 0)
                current = direction > 0 ? -1 : 0;
            var next = (current + direction + modes.Length) % modes.Length;
            SetMode(modes[next]);
        }

        private void UpdatePlacement()
        {
            var leftStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch));
            var rightStick = ApplyDeadzone(OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch));
            var forward = head != null ? Vector3.ProjectOnPlane(head.forward, Vector3.up).normalized : Vector3.forward;
            if (forward.sqrMagnitude < 0.01f) forward = Vector3.forward;
            var right = Vector3.Cross(Vector3.up, forward).normalized;
            var delta = (right * leftStick.x + forward * leftStick.y) * (placementMoveSpeed * Time.deltaTime) +
                        Vector3.up * (rightStick.y * placementVerticalSpeed * Time.deltaTime);
            GetComponent<G1JointController>().MoveBase(delta, rightStick.x * placementRotationSpeed * Time.deltaTime);

            if (placementMarker != null && rightController != null)
            {
                placementMarker.SetActive(true);
                placementMarker.transform.position = rightController.position;
                var pressed = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch) >= gripThreshold;
                if (pressed && !rightIndexWasPressed && GetComponent<G1JointController>().TryGetRootBody(out var root))
                    GetComponent<G1JointController>().PlaceBase(rightController.position + Vector3.up * pelvisHeightAboveFeet);
            }
        }

        private void UpdateArm(Transform controller, Transform target, OVRInput.Controller controllerId,
            ref bool clutched, ref bool orientationMode, bool leftArm,
            ref Vector3 controllerOrigin, ref Vector3 targetOrigin,
            ref Quaternion controllerRotationOrigin, ref Quaternion targetRotationOrigin)
        {
            if (controller == null || target == null)
                return;

            var held = OVRInput.Get(OVRInput.Axis1D.PrimaryHandTrigger, controllerId) >= gripThreshold;
            if (held && !clutched)
            {
                controllerOrigin = controller.position;
                targetOrigin = target.position;
                controllerRotationOrigin = controller.rotation;
                targetRotationOrigin = target.rotation;
                clutched = true;
                orientationMode = false;
                ik.CapturePostureReference(leftArm);
            }
            else if (!held)
            {
                clutched = false;
                orientationMode = false;
            }

            if (clutched)
            {
                var wantsOrientation = OVRInput.Get(OVRInput.Button.PrimaryThumbstick, controllerId);
                if (wantsOrientation != orientationMode)
                {
                    // Rebase at each transition so changing sub-modes cannot jump the hand.
                    controllerOrigin = controller.position;
                    targetOrigin = target.position;
                    controllerRotationOrigin = controller.rotation;
                    targetRotationOrigin = target.rotation;
                    orientationMode = wantsOrientation;
                }

                if (orientationMode)
                {
                    var controllerDelta = controller.rotation * Quaternion.Inverse(controllerRotationOrigin);
                    target.rotation = controllerDelta * targetRotationOrigin;
                }
                else
                {
                    target.position = targetOrigin + (controller.position - controllerOrigin) * motionScale;
                }
            }
        }

        private void Calibrate()
        {
            emergencyStopped = false;
            leftClutched = rightClutched = false;
            leftOrientationMode = rightOrientationMode = false;
            ik.ResetTargetsToHands();
            if (bodyTracking != null)
                bodyTracking.Calibrate();
            calibrated = mode == G1OperatorMode.BodyTracking || mode == G1OperatorMode.FingerTracking ||
                         mode == G1OperatorMode.BackendTeleoperation ||
                         mode == G1OperatorMode.QuestHandRetargeting
                ? bodyTracking != null && bodyTracking.Calibrated
                : leftController != null && rightController != null &&
                  IsTracked(OVRInput.Controller.LTouch) && IsTracked(OVRInput.Controller.RTouch);
            ik.Solve = controlArmed && calibrated &&
                       (mode == G1OperatorMode.Teleoperate || mode == G1OperatorMode.BodyTracking);
        }

        private void ArmControl()
        {
            if (!IsControlMode(mode))
                return;

            emergencyStopped = false;
            controlArmed = true;
            leftClutched = rightClutched = false;
            leftOrientationMode = rightOrientationMode = false;
            ik.ResetTargetsToHands();
            if (bodyTracking != null)
                bodyTracking.Active = mode == G1OperatorMode.BodyTracking || mode == G1OperatorMode.FingerTracking ||
                                      mode == G1OperatorMode.BackendTeleoperation ||
                                      mode == G1OperatorMode.QuestHandRetargeting;
            if (dex3Hands != null)
                dex3Hands.SetActive(mode == G1OperatorMode.FingerTracking ||
                                    mode == G1OperatorMode.BackendTeleoperation ||
                                    mode == G1OperatorMode.QuestHandRetargeting);
            if (backendBridge != null)
            {
                backendBridge.Active = mode == G1OperatorMode.BackendTeleoperation ||
                                       mode == G1OperatorMode.QuestHandRetargeting;
                backendBridge.RetargetQuestHands = mode == G1OperatorMode.QuestHandRetargeting;
                if (backendBridge.Active)
                    backendBridge.RequestSolverReset();
            }
            if (mode == G1OperatorMode.QuestHandRetargeting)
                questHandCalibrationUntil = Time.unscaledTime + 1.4f;
            Calibrate();
            hudFullVisibilityUntil = Time.unscaledTime + 2.5f;

            // Controller teleoperation cannot start safely without both tracked controllers.
            if (mode == G1OperatorMode.Teleoperate && !calibrated)
                DisarmControl();
        }

        private void DisarmControl()
        {
            controlArmed = false;
            leftClutched = rightClutched = false;
            leftOrientationMode = rightOrientationMode = false;
            leftGripperCommand = rightGripperCommand = 0f;
            if (ik != null)
                ik.Solve = false;
            if (bodyTracking != null)
                bodyTracking.Active = false;
            if (backendBridge != null)
            {
                backendBridge.Active = false;
                backendBridge.RetargetQuestHands = false;
            }
            if (dex3Hands != null)
                dex3Hands.SetActive(false);
            hudFullVisibilityUntil = Time.unscaledTime + 2.5f;
        }

        private void SetMode(G1OperatorMode next)
        {
            var leavingTableAlignment = mode == G1OperatorMode.TableAlignment && next != G1OperatorMode.TableAlignment;
            if (mode == G1OperatorMode.Home && next != G1OperatorMode.Home)
                returnFromHome = false;
            DisarmControl();
            mode = next;
            homeComplete = false;
            leftClutched = rightClutched = false;
            leftOrientationMode = rightOrientationMode = false;
            if (locomotor != null)
                locomotor.SetActive(mode == G1OperatorMode.Navigation);
            if (manipulationEnvironment != null)
            {
                if (leavingTableAlignment)
                    manipulationEnvironment.SetAlignmentActive(false);
                if (mode == G1OperatorMode.TableAlignment)
                    manipulationEnvironment.SetAlignmentActive(true);
            }
            if (ik != null && mode != G1OperatorMode.BodyTracking && mode != G1OperatorMode.FingerTracking &&
                mode != G1OperatorMode.BackendTeleoperation && mode != G1OperatorMode.QuestHandRetargeting)
                ik.SetElbowHints(Vector3.zero, Vector3.zero, false);
            if (placementMarker != null)
                placementMarker.SetActive(mode == G1OperatorMode.RobotAlignment || mode == G1OperatorMode.TableAlignment);
            if (ik != null)
                ik.Solve = false;
            hudFullVisibilityUntil = Time.unscaledTime + 2.5f;
        }

        private void UpdateHud(bool leftTracked, bool rightTracked)
        {
            if (hud == null)
                return;

            var state = emergencyStopped ? "EMERGENCY STOP — press A to recalibrate"
                : mode == G1OperatorMode.RobotAlignment && !rightTracked ? "RIGHT CONTROLLER TRACKING LOST — alignment frozen"
                : mode == G1OperatorMode.RobotAlignment ? "ROBOT ALIGNMENT — touch surface + index trigger | sticks fine-adjust"
                : mode == G1OperatorMode.TableAlignment && !rightTracked ? "RIGHT CONTROLLER TRACKING LOST — table frozen"
                : mode == G1OperatorMode.TableAlignment ? "TABLE ALIGNMENT — touch tabletop center + index trigger | sticks fine-adjust"
                : mode == G1OperatorMode.Home ? (homeComplete ? "HOME — neutral pose reached" : "HOME — returning robot to neutral pose")
                : mode == G1OperatorMode.BodyTracking ? (bodyTracking != null && bodyTracking.TrackingValid
                    ? "BODY TRACKING — wrists + elbows driving arms"
                    : $"BODY TRACKING — {bodyTracking?.Status ?? "source unavailable"}")
                : mode == G1OperatorMode.FingerTracking ? (bodyTracking != null && bodyTracking.FingerTrackingValid
                    ? $"FINGER TRACKING — arms + Dex3 fingers | {dex3Hands?.Status ?? "hands unavailable"}"
                    : $"FINGER TRACKING — keep both bare hands visible | {bodyTracking?.Status ?? "source unavailable"}")
                : mode == G1OperatorMode.BackendTeleoperation
                    ? $"BACKEND TELEOP — {bodyTracking?.Status ?? "no body source"} | {backendBridge?.Status ?? "bridge unavailable"}"
                : mode == G1OperatorMode.QuestHandRetargeting && controlArmed &&
                  Time.unscaledTime < questHandCalibrationUntil
                    ? "CALIBRATING HANDS - release the menu pinch and hold both hands open"
                : mode == G1OperatorMode.QuestHandRetargeting
                    ? (bodyTracking != null && bodyTracking.FingerTrackingValid
                        ? $"QUEST HAND RETARGET — Unitree Dex3 | {backendBridge?.Status ?? "bridge unavailable"}"
                        : $"QUEST HAND RETARGET — keep both bare hands visible | {bodyTracking?.Status ?? "source unavailable"}")
                : !leftTracked || !rightTracked ? "TRACKING LOST — commands frozen"
                : !calibrated ? "CALIBRATION REQUIRED — hold neutral pose + press A"
                : mode == G1OperatorMode.Navigation ? "NAVIGATION — sticks move/turn viewpoint"
                : $"TELEOPERATE — L {ArmState(leftClutched, leftOrientationMode)} | R {ArmState(rightClutched, rightOrientationMode)}";
            if (IsControlMode(mode) && !controlArmed && !emergencyStopped)
                state = mode == G1OperatorMode.QuestHandRetargeting
                    ? "PAUSED - use Ring on the palm menu to start"
                    : "PAUSED - press A to start control";

            var commands = mode == G1OperatorMode.Teleoperate
                ? (controlArmed ? $"A: pause | {GraspStatusLine()}" : "A: start | Menu: next | Grip + Menu: previous")
                : mode == G1OperatorMode.BackendTeleoperation
                    ? (controlArmed
                        ? "Sticks: thumbs | triggers: index | grips: middle | A: pause"
                        : "A: start | Menu: next | Grip + Menu: previous")
                : mode == G1OperatorMode.QuestHandRetargeting
                    ? (controlArmed
                        ? (Time.unscaledTime < questHandCalibrationUntil
                            ? "Keep fingers naturally open until calibration completes"
                            : "Palm menu Ring: pause | keep both bare hands visible")
                        : "Palm menu Ring: start | Index: next | Middle: previous")
                : mode == G1OperatorMode.BodyTracking || mode == G1OperatorMode.FingerTracking
                    ? (controlArmed
                        ? "A: pause | right stick click: reset wrists | B: emergency stop"
                        : "A: start | Menu: next | Grip + Menu: previous")
                    : mode == G1OperatorMode.TableAlignment
                        ? "A: reset layout | trigger: place | Menu: next | Grip + Menu: previous"
                    : mode == G1OperatorMode.RobotAlignment
                        ? "A: reset layout | trigger: place | Menu: next | Grip + Menu: previous"
                    : "Left Menu: next | Left Grip + Menu: previous | B: emergency stop";
            var graspFeedback = GetComponent<G1SimulatedGraspController>();
            var graspLine = graspFeedback != null && graspFeedback.enabled
                ? $" | {graspFeedback.FeedbackStatus}" : string.Empty;
            var taskLine = manipulationEnvironment != null && manipulationEnvironment.Available
                ? $" | {manipulationEnvironment.TaskStatus}" : string.Empty;
            hud.text = $"G1 {ModeLabel(mode)}\n{state}\n{commands}\nX: home | Y: view {(robotViewVisible ? "ON" : "OFF")} | L-stick click paused: HUD{graspLine}{taskLine}";
            var trackingFault = IsControlMode(mode) && !controlArmed ? false
                : mode == G1OperatorMode.FingerTracking
                    ? bodyTracking == null || !bodyTracking.FingerTrackingValid || dex3Hands == null || !dex3Hands.Available
                : mode == G1OperatorMode.BodyTracking
                ? bodyTracking == null || !bodyTracking.TrackingValid
                : mode == G1OperatorMode.BackendTeleoperation
                    ? backendBridge == null || !backendBridge.Connected
                : mode == G1OperatorMode.QuestHandRetargeting
                    ? bodyTracking == null || !bodyTracking.FingerTrackingValid ||
                      backendBridge == null || !backendBridge.Connected
                    : mode == G1OperatorMode.Home ? false
                    : mode == G1OperatorMode.RobotAlignment || mode == G1OperatorMode.TableAlignment
                        ? !rightTracked : !leftTracked || !rightTracked;
            hud.color = emergencyStopped || trackingFault
                ? new Color(1f, 0.25f, 0.18f)
                : controlArmed || !IsControlMode(mode) ? Color.white : new Color(1f, 0.75f, 0.15f);
            if (hudCanvasGroup != null)
            {
                var needsAttention = emergencyStopped || trackingFault || !controlArmed ||
                                     mode == G1OperatorMode.Home ||
                                     mode == G1OperatorMode.RobotAlignment ||
                                     mode == G1OperatorMode.TableAlignment ||
                                     Time.unscaledTime < hudFullVisibilityUntil;
                var targetAlpha = emergencyStopped ? 1f : !hudVisible ? 0f : needsAttention ? 0.92f : 0.22f;
                hudCanvasGroup.alpha = Mathf.MoveTowards(hudCanvasGroup.alpha, targetAlpha,
                    Time.unscaledDeltaTime * 2.5f);
            }
        }

        private static bool IsControlMode(G1OperatorMode value) =>
            value == G1OperatorMode.Teleoperate || value == G1OperatorMode.BodyTracking ||
            value == G1OperatorMode.FingerTracking || value == G1OperatorMode.BackendTeleoperation ||
            value == G1OperatorMode.QuestHandRetargeting;

        private void CreateHud()
        {
            if (head == null)
                return;
            var canvasObject = new GameObject("G1 Operator HUD", typeof(RectTransform), typeof(Canvas),
                typeof(CanvasScaler), typeof(CanvasGroup));
            canvasObject.transform.SetParent(head, false);
            canvasObject.transform.localPosition = new Vector3(-0.25f, -0.24f, 0.85f);
            canvasObject.transform.localScale = Vector3.one * 0.00078f;
            var canvas = canvasObject.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.sortingOrder = 110;
            canvasObject.GetComponent<RectTransform>().sizeDelta = new Vector2(570f, 150f);
            hudCanvasGroup = canvasObject.GetComponent<CanvasGroup>();
            hudCanvasGroup.alpha = 0f;
            hudCanvasGroup.interactable = false;
            hudCanvasGroup.blocksRaycasts = false;

            var panel = new GameObject("Panel", typeof(RectTransform), typeof(Image));
            panel.transform.SetParent(canvasObject.transform, false);
            var panelRect = panel.GetComponent<RectTransform>();
            panelRect.anchorMin = Vector2.zero; panelRect.anchorMax = Vector2.one;
            panelRect.offsetMin = Vector2.zero; panelRect.offsetMax = Vector2.zero;
            panel.GetComponent<Image>().color = new Color(0.015f, 0.025f, 0.04f, 0.62f);

            var textObject = new GameObject("Status", typeof(RectTransform), typeof(Text));
            textObject.transform.SetParent(panel.transform, false);
            var textRect = textObject.GetComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero; textRect.anchorMax = Vector2.one;
            textRect.offsetMin = new Vector2(18f, 8f); textRect.offsetMax = new Vector2(-18f, -8f);
            hud = textObject.GetComponent<Text>();
            hud.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            hud.fontSize = 20;
            hud.alignment = TextAnchor.MiddleLeft;
        }

        private void CreateHandModeMenu()
        {
            handModeMenu = new GameObject("G1 Left Palm Mode Menu", typeof(RectTransform), typeof(Canvas),
                typeof(CanvasScaler), typeof(CanvasGroup));
            var uiLayer = LayerMask.NameToLayer("UI");
            if (uiLayer >= 0) handModeMenu.layer = uiLayer;
            var canvas = handModeMenu.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.sortingOrder = 115;
            handModeMenu.transform.localScale = Vector3.one * 0.0007f;
            handModeMenu.GetComponent<RectTransform>().sizeDelta = new Vector2(420f, 155f);
            handModeMenuCanvasGroup = handModeMenu.GetComponent<CanvasGroup>();
            handModeMenuCanvasGroup.alpha = 0f;
            handModeMenuCanvasGroup.interactable = false;
            handModeMenuCanvasGroup.blocksRaycasts = false;

            var panel = new GameObject("Panel", typeof(RectTransform), typeof(Image));
            panel.transform.SetParent(handModeMenu.transform, false);
            if (uiLayer >= 0) panel.layer = uiLayer;
            var panelRect = panel.GetComponent<RectTransform>();
            panelRect.anchorMin = Vector2.zero; panelRect.anchorMax = Vector2.one;
            panelRect.offsetMin = Vector2.zero; panelRect.offsetMax = Vector2.zero;
            panel.GetComponent<Image>().color = new Color(0.012f, 0.025f, 0.045f, 0.88f);

            var textObject = new GameObject("Instructions", typeof(RectTransform), typeof(Text));
            textObject.transform.SetParent(panel.transform, false);
            if (uiLayer >= 0) textObject.layer = uiLayer;
            var textRect = textObject.GetComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero; textRect.anchorMax = Vector2.one;
            textRect.offsetMin = new Vector2(14f, 10f); textRect.offsetMax = new Vector2(-14f, -10f);
            handModeMenuText = textObject.GetComponent<Text>();
            handModeMenuText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            handModeMenuText.fontSize = 21;
            handModeMenuText.alignment = TextAnchor.MiddleCenter;
            handModeMenu.SetActive(false);
        }

        private void UpdateHandModeMenu()
        {
            if (handModeMenu == null || handModeMenuCanvasGroup == null || bodyTracking == null || head == null)
            {
                HideHandMenuImmediately();
                return;
            }

            // In Quest-hand retargeting the operator already knows the pinch controls and the
            // palm frequently faces the headset during normal manipulation. Keep gesture input
            // active in that mode, but never render the palm panel over the workspace.
            var suppressVisual = mode == G1OperatorMode.QuestHandRetargeting;
            var tracked = bodyTracking.TryGetLeftHandMenuState(out var palm, out var normal,
                out var indexPinch, out var middlePinch, out var ringPinch, out var littlePinch);
            var facingHead = false;
            var towardHead = Vector3.forward;
            if (tracked)
            {
                var toHead = head.position - palm;
                var distance = toHead.magnitude;
                if (distance > 0.001f)
                {
                    towardHead = toHead / distance;
                    var alignment = Mathf.Abs(Vector3.Dot(normal, towardHead));
                    // Use stricter entry thresholds and looser exit thresholds so normal
                    // hand-tracking noise cannot toggle the menu every other frame.
                    facingHead = handMenuVisible
                        ? distance > 0.15f && distance < 0.95f && alignment > 0.48f
                        : distance > 0.19f && distance < 0.82f && alignment > 0.70f;
                }
            }

            if (!facingHead)
            {
                handMenuFacingSince = -1f;
                ResetHandMenuGesture();
                if (handMenuVisible && Time.unscaledTime - handMenuLastFacingAt <= 0.28f)
                    return; // Brief tracking loss: hold the last stable menu pose.
                FadeHandMenu(false);
                return;
            }

            handMenuLastFacingAt = Time.unscaledTime;
            if (handMenuFacingSince < 0f)
                handMenuFacingSince = Time.unscaledTime;
            if (!handMenuVisible && Time.unscaledTime - handMenuFacingSince < 0.16f)
                return;

            if (!handMenuVisible)
            {
                handMenuVisible = true;
                handModeMenu.SetActive(!suppressVisual);
                handModeMenuCanvasGroup.alpha = 0f;
                handModeMenu.transform.SetPositionAndRotation(
                    palm + towardHead * 0.055f + Vector3.up * 0.065f,
                    Quaternion.LookRotation(-towardHead, Vector3.up));
            }
            else
            {
                var blend = 1f - Mathf.Exp(-16f * Time.unscaledDeltaTime);
                var targetPosition = palm + towardHead * 0.055f + Vector3.up * 0.065f;
                var targetRotation = Quaternion.LookRotation(-towardHead, Vector3.up);
                handModeMenu.transform.position = Vector3.Lerp(handModeMenu.transform.position, targetPosition, blend);
                handModeMenu.transform.rotation = Quaternion.Slerp(handModeMenu.transform.rotation, targetRotation, blend);
            }
            if (suppressVisual)
            {
                handModeMenuCanvasGroup.alpha = 0f;
                if (handModeMenu.activeSelf)
                    handModeMenu.SetActive(false);
            }
            else
            {
                if (!handModeMenu.activeSelf)
                    handModeMenu.SetActive(true);
                FadeHandMenu(true);
            }

            var action = SelectHandMenuAction(indexPinch, middlePinch, ringPinch, littlePinch);
            var progress = pendingHandAction == action && action != HandMenuAction.None
                ? Mathf.Clamp01((Time.unscaledTime - handActionHeldSince) / 0.45f) : 0f;
            handModeMenuText.text =
                $"{ModeLabel(mode)}\nIndex  NEXT     Middle  BACK\nRing  {(IsControlMode(mode) ? (controlArmed ? "PAUSE" : "START") : "START/PAUSE")}     Little  HOME" +
                (progress > 0f ? $"     {(progress * 100f):0}%" : string.Empty);

            if (action == HandMenuAction.None)
            {
                pendingHandAction = HandMenuAction.None;
                handActionHeldSince = -1f;
                handMenuLatched = false;
                return;
            }
            if (handMenuLatched)
                return;
            if (action != pendingHandAction)
            {
                pendingHandAction = action;
                handActionHeldSince = Time.unscaledTime;
                return;
            }
            if (Time.unscaledTime - handActionHeldSince < 0.45f)
                return;

            handMenuLatched = true;
            if (action == HandMenuAction.Next)
                CycleMode(1);
            else if (action == HandMenuAction.Previous)
                CycleMode(-1);
            else if (action == HandMenuAction.StartPause && IsControlMode(mode))
            {
                if (controlArmed) DisarmControl(); else ArmControl();
            }
            else if (action == HandMenuAction.Home && !emergencyStopped)
                StartHomeAction();
        }

        private void FadeHandMenu(bool show)
        {
            if (handModeMenuCanvasGroup == null)
                return;
            handModeMenuCanvasGroup.alpha = Mathf.MoveTowards(handModeMenuCanvasGroup.alpha,
                show ? 1f : 0f, Time.unscaledDeltaTime * (show ? 7f : 5f));
            if (!show && handModeMenuCanvasGroup.alpha <= 0.001f)
            {
                handMenuVisible = false;
                handModeMenu.SetActive(false);
            }
        }

        private void HideHandMenuImmediately()
        {
            handMenuVisible = false;
            handMenuFacingSince = -1f;
            if (handModeMenuCanvasGroup != null)
                handModeMenuCanvasGroup.alpha = 0f;
            if (handModeMenu != null)
                handModeMenu.SetActive(false);
            ResetHandMenuGesture();
        }

        private static HandMenuAction SelectHandMenuAction(float index, float middle, float ring, float little)
        {
            const float pressed = 0.72f;
            const float released = 0.48f;
            if (index >= pressed && middle < released && ring < released && little < released) return HandMenuAction.Next;
            if (middle >= pressed && index < released && ring < released && little < released) return HandMenuAction.Previous;
            if (ring >= pressed && index < released && middle < released && little < released) return HandMenuAction.StartPause;
            if (little >= pressed && index < released && middle < released && ring < released) return HandMenuAction.Home;
            return HandMenuAction.None;
        }

        private void ResetHandMenuGesture()
        {
            pendingHandAction = HandMenuAction.None;
            handActionHeldSince = -1f;
            handMenuLatched = false;
        }

        private enum HandMenuAction
        {
            None, Next, Previous, StartPause, Home
        }

        private void CreateRobotView()
        {
            if (head == null)
                return;

            var robotHead = FindRobotLink("head_link");
            if (robotHead == null)
            {
                Debug.LogWarning("G1 robot view could not find head_link.", this);
                return;
            }

            var cameraObject = new GameObject("G1 Egocentric Camera", typeof(Camera),
                typeof(UniversalAdditionalCameraData));
            cameraObject.transform.SetParent(robotHead, true);
            var cameraPosition = robotHead.position;
            var cameraForwardOffset = 0.09f;
            var headRenderers = robotHead.GetComponentsInChildren<Renderer>(true);
            if (headRenderers.Length > 0)
            {
                var bounds = headRenderers[0].bounds;
                for (var i = 1; i < headRenderers.Length; i++) bounds.Encapsulate(headRenderers[i].bounds);
                cameraPosition = bounds.center;
                var forward = transform.forward;
                if (robot != null && robot.TryGetRootBody(out var measuredRoot))
                    forward = measuredRoot.transform.forward;
                cameraForwardOffset = Mathf.Abs(forward.x) * bounds.extents.x +
                                      Mathf.Abs(forward.y) * bounds.extents.y +
                                      Mathf.Abs(forward.z) * bounds.extents.z + 0.025f;
            }
            var robotForward = transform.forward;
            if (robot != null && robot.TryGetRootBody(out var root))
                robotForward = root.transform.forward;
            cameraObject.transform.SetPositionAndRotation(
                cameraPosition + robotForward * cameraForwardOffset + Vector3.up * 0.015f,
                Quaternion.LookRotation(robotForward, Vector3.up));

            robotViewTexture = new RenderTexture(robotCameraWidth, robotCameraHeight, 16,
                RenderTextureFormat.ARGB32)
            {
                name = "G1 Robot View",
                antiAliasing = 1,
                useMipMap = false,
                useDynamicScale = false
            };
            robotViewTexture.Create();
            robotViewCamera = cameraObject.GetComponent<Camera>();
            robotViewCamera.fieldOfView = robotCameraFieldOfView;
            robotViewCamera.nearClipPlane = 0.03f;
            robotViewCamera.farClipPlane = 50f;
            robotViewCamera.targetTexture = robotViewTexture;
            robotViewCamera.allowHDR = false;
            robotViewCamera.allowMSAA = false;
            robotViewCamera.forceIntoRenderTexture = true;
            robotViewCamera.cullingMask &= ~(1 << LayerMask.NameToLayer("UI"));
            var cameraData = cameraObject.GetComponent<UniversalAdditionalCameraData>();
            cameraData.allowXRRendering = false;
            cameraData.renderType = CameraRenderType.Base;
            cameraData.renderPostProcessing = false;
            cameraData.antialiasing = AntialiasingMode.None;
            cameraData.requiresColorOption = CameraOverrideOption.Off;
            cameraData.requiresDepthOption = CameraOverrideOption.Off;

            robotViewPanel = new GameObject("G1 Robot View Panel", typeof(RectTransform), typeof(Canvas),
                typeof(CanvasScaler));
            robotViewPanel.transform.SetParent(head, false);
            robotViewPanel.transform.localPosition = new Vector3(0.34f, 0.08f, 0.78f);
            robotViewPanel.transform.localRotation = Quaternion.Euler(0f, -12f, 0f);
            robotViewPanel.transform.localScale = Vector3.one * 0.0012f;
            SetLayerRecursively(robotViewPanel, LayerMask.NameToLayer("UI"));
            var canvas = robotViewPanel.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;
            canvas.sortingOrder = 109;
            robotViewPanel.GetComponent<RectTransform>().sizeDelta = new Vector2(400f, 250f);

            var frame = new GameObject("Frame", typeof(RectTransform), typeof(Image));
            frame.transform.SetParent(robotViewPanel.transform, false);
            var frameRect = frame.GetComponent<RectTransform>();
            frameRect.anchorMin = Vector2.zero; frameRect.anchorMax = Vector2.one;
            frameRect.offsetMin = Vector2.zero; frameRect.offsetMax = Vector2.zero;
            frame.GetComponent<Image>().color = new Color(0.02f, 0.035f, 0.055f, 0.96f);

            var imageObject = new GameObject("Robot Camera Feed", typeof(RectTransform), typeof(RawImage));
            imageObject.transform.SetParent(frame.transform, false);
            var imageRect = imageObject.GetComponent<RectTransform>();
            imageRect.anchorMin = Vector2.zero; imageRect.anchorMax = Vector2.one;
            imageRect.offsetMin = new Vector2(10f, 10f); imageRect.offsetMax = new Vector2(-10f, -36f);
            imageObject.GetComponent<RawImage>().texture = robotViewTexture;

            var labelObject = new GameObject("Label", typeof(RectTransform), typeof(Text));
            labelObject.transform.SetParent(frame.transform, false);
            var labelRect = labelObject.GetComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0f, 1f); labelRect.anchorMax = Vector2.one;
            labelRect.offsetMin = new Vector2(12f, -34f); labelRect.offsetMax = new Vector2(-12f, -4f);
            var label = labelObject.GetComponent<Text>();
            label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            label.fontSize = 21;
            label.alignment = TextAnchor.MiddleLeft;
            label.color = new Color(0.25f, 0.9f, 1f);
            label.text = "G1 HEAD CAMERA";
            SetLayerRecursively(robotViewPanel, LayerMask.NameToLayer("UI"));
            SetRobotViewVisible(false);
        }

        private void SetRobotViewVisible(bool visible)
        {
            robotViewVisible = visible && robotViewPanel != null && robotViewCamera != null;
            if (robotViewPanel != null)
                robotViewPanel.SetActive(robotViewVisible);
            if (robotViewCamera != null)
                robotViewCamera.enabled = robotViewVisible;
        }

        private Transform FindRobotLink(string linkName)
        {
            Transform fallback = null;
            foreach (var candidate in GetComponentsInChildren<Transform>(true))
            {
                if (candidate.name != linkName)
                    continue;
                if (candidate.GetComponent<ArticulationBody>() != null)
                    return candidate;
                fallback ??= candidate;
            }
            return fallback;
        }

        private static void SetLayerRecursively(GameObject rootObject, int layer)
        {
            if (rootObject == null || layer < 0)
                return;
            rootObject.layer = layer;
            foreach (Transform child in rootObject.transform)
                SetLayerRecursively(child.gameObject, layer);
        }

        private void OnDestroy()
        {
            if (robotViewTexture != null)
            {
                robotViewTexture.Release();
                Destroy(robotViewTexture);
            }
        }

        private void CreatePlacementMarker()
        {
            placementMarker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            placementMarker.name = "G1 Robot Alignment Marker";
            placementMarker.transform.localScale = new Vector3(0.14f, 0.004f, 0.14f);
            Destroy(placementMarker.GetComponent<Collider>());
            var renderer = placementMarker.GetComponent<Renderer>();
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null) renderer.material = new Material(shader) { color = new Color(0.1f, 1f, 0.35f) };
            placementMarker.SetActive(false);
        }

        private void MeasurePelvisHeight()
        {
            var robot = GetComponent<G1JointController>();
            if (!robot.TryGetRootBody(out var root)) return;
            var renderers = robot.GetComponentsInChildren<Renderer>(true);
            if (renderers.Length == 0) return;
            var bounds = renderers[0].bounds;
            for (var i = 1; i < renderers.Length; i++) bounds.Encapsulate(renderers[i].bounds);
            pelvisHeightAboveFeet = root.transform.position.y - bounds.min.y;
        }

        private static Vector2 ApplyDeadzone(Vector2 value) => value.magnitude < 0.2f ? Vector2.zero : value;

        private static bool IsTracked(OVRInput.Controller controller) =>
            OVRInput.GetControllerPositionTracked(controller) && OVRInput.GetControllerOrientationTracked(controller);

        private static string ModeLabel(G1OperatorMode value) =>
            value == G1OperatorMode.RobotAlignment ? "ROBOT ALIGNMENT"
            : value == G1OperatorMode.TableAlignment ? "TABLE ALIGNMENT"
            : value == G1OperatorMode.FingerTracking ? "FINGER TRACKING"
            : value == G1OperatorMode.BackendTeleoperation ? "BACKEND TELEOP"
            : value == G1OperatorMode.QuestHandRetargeting ? "QUEST HAND RETARGET"
            : value.ToString().ToUpperInvariant();

        private string GraspStatusLine()
        {
            var grasp = GetComponent<G1SimulatedGraspController>();
            var held = grasp != null && grasp.enabled ? $" | Held L: {grasp.LeftStatus} R: {grasp.RightStatus}" : string.Empty;
            return $"Grip: position | Grip + stick press: orientation | Triggers: grippers{held}";
        }

        private static string ArmState(bool clutched, bool orientationMode) =>
            !clutched ? "READY" : orientationMode ? "ORIENTATION" : "POSITION";

        private static Transform FindTransform(string objectName)
        {
            var found = GameObject.Find(objectName);
            return found != null ? found.transform : null;
        }

        private static GameObject FindSceneObject(string objectName)
        {
            foreach (var candidate in Resources.FindObjectsOfTypeAll<GameObject>())
                if (candidate.name == objectName && candidate.scene.IsValid()) return candidate;
            return null;
        }
    }
}
