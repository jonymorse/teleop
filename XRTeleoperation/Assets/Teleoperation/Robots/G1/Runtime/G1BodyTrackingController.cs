using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Hands;

namespace Teleoperation.Robots.G1
{
    /// <summary>Maps Meta upper-body wrist and elbow poses into the G1 arm task space.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(G1DualArmIkController))]
    public sealed class G1BodyTrackingController : MonoBehaviour
    {
        [SerializeField, Range(0.05f, 1f)] private float smoothing = 0.35f;

        private G1DualArmIkController ik;
        private OVRBody body;
        private Transform trackingSpace;
        private Vector3 leftOffset;
        private Vector3 rightOffset;
        private Quaternion leftRotationOffset = Quaternion.identity;
        private Quaternion rightRotationOffset = Quaternion.identity;
        private bool calibrated;
        private float neutralMessageUntil;
        private XRHandSubsystem xrHandSubsystem;
        private readonly List<XRHandSubsystem> handSubsystems = new();

        public bool Active { get; set; }
        public bool TrackingValid { get; private set; }
        public bool FingerTrackingValid { get; private set; }
        public bool Calibrated => calibrated;
        public string Status { get; private set; } = "waiting for Meta body permission/data";
        public float LeftPinch { get; private set; }
        public float RightPinch { get; private set; }
        public Vector3 LeftFingerCurls { get; private set; }
        public Vector3 RightFingerCurls { get; private set; }

        // OpenXR hand-joint order expected by Unitree's xr_teleoperate retargeter:
        // wrist, thumb(4), then index/middle/ring/little(5 each).
        private static readonly OVRPlugin.BoneId[] LeftHandJoints =
        {
            OVRPlugin.BoneId.Body_LeftHandWrist,
            OVRPlugin.BoneId.Body_LeftHandThumbMetacarpal, OVRPlugin.BoneId.Body_LeftHandThumbProximal,
            OVRPlugin.BoneId.Body_LeftHandThumbDistal, OVRPlugin.BoneId.Body_LeftHandThumbTip,
            OVRPlugin.BoneId.Body_LeftHandIndexMetacarpal, OVRPlugin.BoneId.Body_LeftHandIndexProximal,
            OVRPlugin.BoneId.Body_LeftHandIndexIntermediate, OVRPlugin.BoneId.Body_LeftHandIndexDistal,
            OVRPlugin.BoneId.Body_LeftHandIndexTip,
            OVRPlugin.BoneId.Body_LeftHandMiddleMetacarpal, OVRPlugin.BoneId.Body_LeftHandMiddleProximal,
            OVRPlugin.BoneId.Body_LeftHandMiddleIntermediate, OVRPlugin.BoneId.Body_LeftHandMiddleDistal,
            OVRPlugin.BoneId.Body_LeftHandMiddleTip,
            OVRPlugin.BoneId.Body_LeftHandRingMetacarpal, OVRPlugin.BoneId.Body_LeftHandRingProximal,
            OVRPlugin.BoneId.Body_LeftHandRingIntermediate, OVRPlugin.BoneId.Body_LeftHandRingDistal,
            OVRPlugin.BoneId.Body_LeftHandRingTip,
            OVRPlugin.BoneId.Body_LeftHandLittleMetacarpal, OVRPlugin.BoneId.Body_LeftHandLittleProximal,
            OVRPlugin.BoneId.Body_LeftHandLittleIntermediate, OVRPlugin.BoneId.Body_LeftHandLittleDistal,
            OVRPlugin.BoneId.Body_LeftHandLittleTip
        };

        private static readonly OVRPlugin.BoneId[] RightHandJoints =
        {
            OVRPlugin.BoneId.Body_RightHandWrist,
            OVRPlugin.BoneId.Body_RightHandThumbMetacarpal, OVRPlugin.BoneId.Body_RightHandThumbProximal,
            OVRPlugin.BoneId.Body_RightHandThumbDistal, OVRPlugin.BoneId.Body_RightHandThumbTip,
            OVRPlugin.BoneId.Body_RightHandIndexMetacarpal, OVRPlugin.BoneId.Body_RightHandIndexProximal,
            OVRPlugin.BoneId.Body_RightHandIndexIntermediate, OVRPlugin.BoneId.Body_RightHandIndexDistal,
            OVRPlugin.BoneId.Body_RightHandIndexTip,
            OVRPlugin.BoneId.Body_RightHandMiddleMetacarpal, OVRPlugin.BoneId.Body_RightHandMiddleProximal,
            OVRPlugin.BoneId.Body_RightHandMiddleIntermediate, OVRPlugin.BoneId.Body_RightHandMiddleDistal,
            OVRPlugin.BoneId.Body_RightHandMiddleTip,
            OVRPlugin.BoneId.Body_RightHandRingMetacarpal, OVRPlugin.BoneId.Body_RightHandRingProximal,
            OVRPlugin.BoneId.Body_RightHandRingIntermediate, OVRPlugin.BoneId.Body_RightHandRingDistal,
            OVRPlugin.BoneId.Body_RightHandRingTip,
            OVRPlugin.BoneId.Body_RightHandLittleMetacarpal, OVRPlugin.BoneId.Body_RightHandLittleProximal,
            OVRPlugin.BoneId.Body_RightHandLittleIntermediate, OVRPlugin.BoneId.Body_RightHandLittleDistal,
            OVRPlugin.BoneId.Body_RightHandLittleTip
        };

        private static readonly XRHandJointID[] XrHandJoints =
        {
            XRHandJointID.Wrist,
            XRHandJointID.ThumbMetacarpal, XRHandJointID.ThumbProximal,
            XRHandJointID.ThumbDistal, XRHandJointID.ThumbTip,
            XRHandJointID.IndexMetacarpal, XRHandJointID.IndexProximal,
            XRHandJointID.IndexIntermediate, XRHandJointID.IndexDistal, XRHandJointID.IndexTip,
            XRHandJointID.MiddleMetacarpal, XRHandJointID.MiddleProximal,
            XRHandJointID.MiddleIntermediate, XRHandJointID.MiddleDistal, XRHandJointID.MiddleTip,
            XRHandJointID.RingMetacarpal, XRHandJointID.RingProximal,
            XRHandJointID.RingIntermediate, XRHandJointID.RingDistal, XRHandJointID.RingTip,
            XRHandJointID.LittleMetacarpal, XRHandJointID.LittleProximal,
            XRHandJointID.LittleIntermediate, XRHandJointID.LittleDistal, XRHandJointID.LittleTip
        };

        public bool TryGetUnitreeHandKeypoints(out float[] left, out float[] right)
        {
            left = right = null;
            if (!Active)
                return false;

            EnsureXrHandSubsystem();
            if (xrHandSubsystem != null && xrHandSubsystem.running &&
                TryBuildUnitreeHand(xrHandSubsystem.leftHand, true, out left) &&
                TryBuildUnitreeHand(xrHandSubsystem.rightHand, false, out right))
                return true;

            // Body tracking remains a compatibility fallback for runtimes without XR Hands.
            if (!TryBuildUnitreeHand(LeftHandJoints, true, out left) ||
                !TryBuildUnitreeHand(RightHandJoints, false, out right))
            {
                left = right = null;
                return false;
            }
            return true;
        }

        private void EnsureXrHandSubsystem()
        {
            if (xrHandSubsystem != null && xrHandSubsystem.running)
                return;
            handSubsystems.Clear();
            SubsystemManager.GetSubsystems(handSubsystems);
            xrHandSubsystem = null;
            foreach (var subsystem in handSubsystems)
                if (subsystem != null && subsystem.running)
                {
                    xrHandSubsystem = subsystem;
                    break;
                }
        }

        private static bool TryBuildUnitreeHand(XRHand hand, bool left, out float[] points)
        {
            points = null;
            if (!hand.isTracked || !hand.GetJoint(XRHandJointID.Wrist).TryGetPose(out var wrist))
                return false;
            var values = new float[XrHandJoints.Length * 3];
            var inverseWrist = Quaternion.Inverse(wrist.rotation);
            for (var i = 0; i < XrHandJoints.Length; i++)
            {
                if (!hand.GetJoint(XrHandJoints[i]).TryGetPose(out var pose))
                    return false;
                WriteUnitreePoint(values, i, inverseWrist * (pose.position - wrist.position), left);
            }
            points = values;
            return true;
        }

        private bool TryBuildUnitreeHand(OVRPlugin.BoneId[] ids, bool left, out float[] points)
        {
            points = null;
            if (!TryGetPose(ids[0], out var wrist, out var wristRotation))
                return false;

            var values = new float[ids.Length * 3];
            var inverseWrist = Quaternion.Inverse(wristRotation);
            for (var i = 0; i < ids.Length; i++)
            {
                if (!TryGetPose(ids[i], out var position, out _))
                    return false;
                WriteUnitreePoint(values, i, inverseWrist * (position - wrist), left);
            }
            points = values;
            return true;
        }

        private static void WriteUnitreePoint(float[] values, int index, Vector3 unityWristLocal, bool left)
        {
            // Match TeleVuer's T_TO_UNITREE_HAND conversion. Unity has already reflected
            // OpenXR Z while importing the pose, so the combined wrist-local mapping is
            // identical for both hands: Unitree xyz = (Unity y, -Unity z, Unity x).
            values[index * 3] = unityWristLocal.y;
            values[index * 3 + 1] = -unityWristLocal.z;
            values[index * 3 + 2] = unityWristLocal.x;
        }

        public bool TryGetLeftHandMenuState(out Vector3 palmPosition, out Vector3 palmNormal,
            out float indexPinch, out float middlePinch, out float ringPinch, out float littlePinch)
        {
            palmPosition = Vector3.zero;
            palmNormal = Vector3.forward;
            indexPinch = middlePinch = ringPinch = littlePinch = 0f;
            if (!TryGetPose(OVRPlugin.BoneId.Body_LeftHandPalm, out palmPosition, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandWrist, out var wrist, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandIndexMetacarpal, out var indexBase, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandLittleMetacarpal, out var littleBase, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandThumbTip, out var thumbTip, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandIndexTip, out var indexTip, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandMiddleTip, out var middleTip, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandRingTip, out var ringTip, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftHandLittleTip, out var littleTip, out _))
                return false;

            palmNormal = Vector3.Cross(indexBase - wrist, littleBase - wrist).normalized;
            indexPinch = PinchStrength(thumbTip, indexTip);
            middlePinch = PinchStrength(thumbTip, middleTip);
            ringPinch = PinchStrength(thumbTip, ringTip);
            littlePinch = PinchStrength(thumbTip, littleTip);
            return palmNormal.sqrMagnitude > 0.5f;
        }

        private void Awake()
        {
            ik = GetComponent<G1DualArmIkController>();
            var source = new GameObject("G1 Meta Upper Body Source");
            source.transform.SetParent(transform, false);
            body = source.AddComponent<OVRBody>();
            body.ProvidedSkeletonType = OVRPlugin.BodyJointSet.UpperBody;
            trackingSpace = FindTransform("TrackingSpace");
        }

        public void Calibrate()
        {
            calibrated = false;
            if (TryGetPose(OVRPlugin.BoneId.Body_LeftHandWrist, out var leftPosition, out var leftRotation) &&
                TryGetPose(OVRPlugin.BoneId.Body_RightHandWrist, out var rightPosition, out var rightRotation))
            {
                leftOffset = ik.LeftTarget.position - leftPosition;
                rightOffset = ik.RightTarget.position - rightPosition;
                leftRotationOffset = Quaternion.Inverse(leftRotation) * ik.LeftTarget.rotation;
                rightRotationOffset = Quaternion.Inverse(rightRotation) * ik.RightTarget.rotation;
                ik.CapturePostureReference(true);
                ik.CapturePostureReference(false);
                calibrated = true;
                Status = "calibrated";
            }
            else
                Status = "no wrist data yet; calibration will retry automatically";
        }

        public bool ResetNeutralOrientation()
        {
            if (!TryGetPose(OVRPlugin.BoneId.Body_LeftHandWrist, out _, out var leftRotation) ||
                !TryGetPose(OVRPlugin.BoneId.Body_RightHandWrist, out _, out var rightRotation) ||
                ik.LeftEndEffector == null || ik.RightEndEffector == null)
            {
                Status = "neutral reset failed; keep both hands visible";
                return false;
            }

            ik.LeftTarget.rotation = ik.LeftEndEffector.rotation;
            ik.RightTarget.rotation = ik.RightEndEffector.rotation;
            leftRotationOffset = Quaternion.Inverse(leftRotation) * ik.LeftTarget.rotation;
            rightRotationOffset = Quaternion.Inverse(rightRotation) * ik.RightTarget.rotation;
            ik.CapturePostureReference(true);
            ik.CapturePostureReference(false);
            Status = "neutral wrist orientation reset";
            neutralMessageUntil = Time.unscaledTime + 2f;
            return true;
        }

        private void Update()
        {
            if (!Active)
            {
                TrackingValid = false;
                FingerTrackingValid = false;
                LeftPinch = RightPinch = 0f;
                LeftFingerCurls = RightFingerCurls = Vector3.zero;
                return;
            }

            if (body == null || !body.enabled || !body.BodyState.HasValue)
            {
                TrackingValid = false;
                FingerTrackingValid = false;
                Status = "waiting for Meta body permission/data";
                return;
            }

            if (!calibrated)
                Calibrate();
            TrackingValid = calibrated;
            if (!TrackingValid)
                return;

            if (!TryGetPose(OVRPlugin.BoneId.Body_LeftHandWrist, out var leftWrist, out var leftRotation) ||
                !TryGetPose(OVRPlugin.BoneId.Body_RightHandWrist, out var rightWrist, out var rightRotation) ||
                !TryGetPose(OVRPlugin.BoneId.Body_LeftArmLower, out var leftElbow, out _) ||
                !TryGetPose(OVRPlugin.BoneId.Body_RightArmLower, out var rightElbow, out _))
            {
                TrackingValid = false;
                Status = "pose incomplete; keep both arms visible";
                return;
            }

            var blend = 1f - Mathf.Pow(1f - smoothing, Time.deltaTime * 90f);
            ik.LeftTarget.position = Vector3.Lerp(ik.LeftTarget.position, leftWrist + leftOffset, blend);
            ik.RightTarget.position = Vector3.Lerp(ik.RightTarget.position, rightWrist + rightOffset, blend);
            ik.LeftTarget.rotation = Quaternion.Slerp(ik.LeftTarget.rotation,
                leftRotation * leftRotationOffset, blend);
            ik.RightTarget.rotation = Quaternion.Slerp(ik.RightTarget.rotation,
                rightRotation * rightRotationOffset, blend);
            ik.SetElbowHints(leftElbow + leftOffset, rightElbow + rightOffset, true);
            FingerTrackingValid = UpdateFingerCurls();
            LeftPinch = ReadPinch(OVRPlugin.BoneId.Body_LeftHandThumbTip,
                OVRPlugin.BoneId.Body_LeftHandIndexTip);
            RightPinch = ReadPinch(OVRPlugin.BoneId.Body_RightHandThumbTip,
                OVRPlugin.BoneId.Body_RightHandIndexTip);
            if (Time.unscaledTime >= neutralMessageUntil)
                Status = "tracking wrists and elbows";
        }

        private bool UpdateFingerCurls()
        {
            if (!TryFingerCurl(OVRPlugin.BoneId.Body_LeftHandThumbMetacarpal,
                    OVRPlugin.BoneId.Body_LeftHandThumbProximal,
                    OVRPlugin.BoneId.Body_LeftHandThumbDistal,
                    OVRPlugin.BoneId.Body_LeftHandThumbTip, out var leftThumb) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_LeftHandIndexMetacarpal,
                    OVRPlugin.BoneId.Body_LeftHandIndexProximal,
                    OVRPlugin.BoneId.Body_LeftHandIndexIntermediate,
                    OVRPlugin.BoneId.Body_LeftHandIndexTip, out var leftIndex) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_LeftHandMiddleMetacarpal,
                    OVRPlugin.BoneId.Body_LeftHandMiddleProximal,
                    OVRPlugin.BoneId.Body_LeftHandMiddleIntermediate,
                    OVRPlugin.BoneId.Body_LeftHandMiddleTip, out var leftMiddle) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_LeftHandRingMetacarpal,
                    OVRPlugin.BoneId.Body_LeftHandRingProximal,
                    OVRPlugin.BoneId.Body_LeftHandRingIntermediate,
                    OVRPlugin.BoneId.Body_LeftHandRingTip, out var leftRing) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_LeftHandLittleMetacarpal,
                    OVRPlugin.BoneId.Body_LeftHandLittleProximal,
                    OVRPlugin.BoneId.Body_LeftHandLittleIntermediate,
                    OVRPlugin.BoneId.Body_LeftHandLittleTip, out var leftLittle) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_RightHandThumbMetacarpal,
                    OVRPlugin.BoneId.Body_RightHandThumbProximal,
                    OVRPlugin.BoneId.Body_RightHandThumbDistal,
                    OVRPlugin.BoneId.Body_RightHandThumbTip, out var rightThumb) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_RightHandIndexMetacarpal,
                    OVRPlugin.BoneId.Body_RightHandIndexProximal,
                    OVRPlugin.BoneId.Body_RightHandIndexIntermediate,
                    OVRPlugin.BoneId.Body_RightHandIndexTip, out var rightIndex) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_RightHandMiddleMetacarpal,
                    OVRPlugin.BoneId.Body_RightHandMiddleProximal,
                    OVRPlugin.BoneId.Body_RightHandMiddleIntermediate,
                    OVRPlugin.BoneId.Body_RightHandMiddleTip, out var rightMiddle) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_RightHandRingMetacarpal,
                    OVRPlugin.BoneId.Body_RightHandRingProximal,
                    OVRPlugin.BoneId.Body_RightHandRingIntermediate,
                    OVRPlugin.BoneId.Body_RightHandRingTip, out var rightRing) ||
                !TryFingerCurl(OVRPlugin.BoneId.Body_RightHandLittleMetacarpal,
                    OVRPlugin.BoneId.Body_RightHandLittleProximal,
                    OVRPlugin.BoneId.Body_RightHandLittleIntermediate,
                    OVRPlugin.BoneId.Body_RightHandLittleTip, out var rightLittle))
            {
                LeftFingerCurls = RightFingerCurls = Vector3.zero;
                return false;
            }

            LeftFingerCurls = new Vector3(leftThumb, leftIndex, (leftMiddle + leftRing + leftLittle) / 3f);
            RightFingerCurls = new Vector3(rightThumb, rightIndex, (rightMiddle + rightRing + rightLittle) / 3f);
            return true;
        }

        private bool TryFingerCurl(OVRPlugin.BoneId aId, OVRPlugin.BoneId bId,
            OVRPlugin.BoneId cId, OVRPlugin.BoneId dId, out float curl)
        {
            curl = 0f;
            if (!TryGetPose(aId, out var a, out _) || !TryGetPose(bId, out var b, out _) ||
                !TryGetPose(cId, out var c, out _) || !TryGetPose(dId, out var d, out _))
                return false;
            var first = Vector3.Angle(b - a, c - b);
            var second = Vector3.Angle(c - b, d - c);
            curl = Mathf.InverseLerp(8f, 145f, first + second);
            return true;
        }

        private float ReadPinch(OVRPlugin.BoneId thumbTip, OVRPlugin.BoneId indexTip)
        {
            if (!TryGetPose(thumbTip, out var thumb, out _) || !TryGetPose(indexTip, out var index, out _))
                return 0f;
            return Mathf.InverseLerp(0.06f, 0.018f, Vector3.Distance(thumb, index));
        }

        private static float PinchStrength(Vector3 thumb, Vector3 finger) =>
            Mathf.InverseLerp(0.055f, 0.018f, Vector3.Distance(thumb, finger));

        private bool TryGetPose(OVRPlugin.BoneId id, out Vector3 position, out Quaternion rotation)
        {
            position = default;
            rotation = Quaternion.identity;
            var state = body != null ? body.BodyState : null;
            var index = (int)id;
            if (!state.HasValue || state.Value.JointLocations == null || index >= state.Value.JointLocations.Length)
                return false;
            var joint = state.Value.JointLocations[index];
            if (!joint.PositionValid || !joint.OrientationValid)
                return false;
            var localPosition = joint.Pose.Position.FromFlippedZVector3f();
            var localRotation = joint.Pose.Orientation.FromFlippedZQuatf();
            position = trackingSpace != null ? trackingSpace.TransformPoint(localPosition) : localPosition;
            rotation = trackingSpace != null ? trackingSpace.rotation * localRotation : localRotation;
            return true;
        }

        private static Transform FindTransform(string objectName)
        {
            var found = GameObject.Find(objectName);
            return found != null ? found.transform : null;
        }
    }
}
