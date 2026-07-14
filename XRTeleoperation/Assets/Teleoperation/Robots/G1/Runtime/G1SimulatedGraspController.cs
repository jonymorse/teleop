using System.Collections;
using UnityEngine;

namespace Teleoperation.Robots.G1
{
    /// <summary>
    /// A contact-qualified grasp adapter for the simulated Dex3 hands.
    /// It keeps grasping stable without allowing objects to attach through empty space.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(G1XrTeleoperationController))]
    public sealed class G1SimulatedGraspController : MonoBehaviour
    {
        [SerializeField, Range(0.001f, 0.01f)] private float contactTolerance = 0.003f;
        [SerializeField, Range(0.004f, 0.02f)] private float assistedAcquisitionTolerance = 0.01f;
        [SerializeField, Range(0f, 1f)] private float graspIntentThreshold = 0.35f;
        [SerializeField, Range(0f, 1f)] private float releaseThreshold = 0.25f;
        [SerializeField, Range(0.02f, 0.2f)] private float acquisitionDwellSeconds = 0.055f;
        [SerializeField, Range(0.3f, 2f)] private float maximumAcquisitionSpeed = 0.9f;
        [SerializeField, Range(0.05f, 0.3f)] private float contactMemorySeconds = 0.15f;
        [SerializeField, Range(0.05f, 0.5f)] private float contactLossGraceSeconds = 0.18f;
        [SerializeField, Min(5f)] private float maximumHoldingForce = 45f;
        [SerializeField, Min(0.5f)] private float maximumHoldingTorque = 8f;
        [SerializeField, Range(0f, 1f)] private float hapticAmplitude = 0.6f;

        private G1XrTeleoperationController input;
        private G1Dex3HandController dex3Hands;
        private Transform leftHand;
        private Transform rightHand;
        private G1Graspable leftHeld;
        private G1Graspable rightHeld;
        private ConfigurableJoint leftConstraint;
        private ConfigurableJoint rightConstraint;
        private bool leftFingerLatch;
        private bool rightFingerLatch;
        private float leftContactLostFor;
        private float rightContactLostFor;
        private float leftContactQuality;
        private float rightContactQuality;
        private readonly ContactMemory leftContactMemory = new();
        private readonly ContactMemory rightContactMemory = new();
        private G1Graspable leftCandidate;
        private G1Graspable rightCandidate;
        private float leftCandidateQuality;
        private float rightCandidateQuality;
        private int leftCandidateGroups;
        private int rightCandidateGroups;
        private float leftCandidateDistance = float.PositiveInfinity;
        private float rightCandidateDistance = float.PositiveInfinity;
        private G1Graspable leftQualifiedCandidate;
        private G1Graspable rightQualifiedCandidate;
        private float leftCandidateDwell;
        private float rightCandidateDwell;
        private G1Graspable feedbackObject;
        private G1GraspFeedback feedbackState;
        private bool leftReadyLastFrame;
        private bool rightReadyLastFrame;
        private Vector3 previousLeftPosition;
        private Vector3 previousRightPosition;
        private Vector3 leftVelocity;
        private Vector3 rightVelocity;

        public string LeftStatus => leftHeld != null ? leftHeld.DisplayName : "Open";
        public string RightStatus => rightHeld != null ? rightHeld.DisplayName : "Open";
        public bool IsHolding(G1Graspable graspable) => graspable != null &&
                                                        (leftHeld == graspable || rightHeld == graspable);
        public string FeedbackStatus => feedbackState == G1GraspFeedback.Ready ? "grasp READY"
            : feedbackState == G1GraspFeedback.Contact ? "one-sided contact"
            : feedbackState == G1GraspFeedback.Near ? "approaching object"
            : feedbackState == G1GraspFeedback.Slipping ? "grasp SLIPPING"
            : feedbackState == G1GraspFeedback.Held ? "grasp stable" : "no grasp contact";

        public void ReleaseAll()
        {
            if (leftHeld != null || leftConstraint != null || leftFingerLatch)
                Release(ref leftHeld, ref leftConstraint, ref leftFingerLatch, ref leftContactLostFor,
                    ref leftContactQuality, Vector3.zero, OVRInput.Controller.LTouch, true, true);
            if (rightHeld != null || rightConstraint != null || rightFingerLatch)
                Release(ref rightHeld, ref rightConstraint, ref rightFingerLatch, ref rightContactLostFor,
                    ref rightContactQuality, Vector3.zero, OVRInput.Controller.RTouch, false, true);
        }

        private void Start()
        {
            input = GetComponent<G1XrTeleoperationController>();
            dex3Hands = GetComponent<G1Dex3HandController>();
            leftHand = FindDescendant("left_rubber_hand");
            rightHand = FindDescendant("right_rubber_hand");
            if (leftHand != null) previousLeftPosition = leftHand.position;
            if (rightHand != null) previousRightPosition = rightHand.position;
        }

        private void Update()
        {
            if (leftHand != null)
            {
                leftVelocity = (leftHand.position - previousLeftPosition) / Mathf.Max(Time.deltaTime, 0.001f);
                previousLeftPosition = leftHand.position;
                UpdateHand(leftHand, input.LeftGripperCommand, ref leftHeld, ref leftConstraint,
                    ref leftFingerLatch,
                    ref leftContactLostFor, ref leftContactQuality, leftContactMemory,
                    ref leftCandidate, ref leftCandidateQuality, ref leftCandidateGroups,
                    ref leftCandidateDistance, ref leftQualifiedCandidate, ref leftCandidateDwell,
                    true, leftVelocity, OVRInput.Controller.LTouch);
            }
            if (rightHand != null)
            {
                rightVelocity = (rightHand.position - previousRightPosition) / Mathf.Max(Time.deltaTime, 0.001f);
                previousRightPosition = rightHand.position;
                UpdateHand(rightHand, input.RightGripperCommand, ref rightHeld, ref rightConstraint,
                    ref rightFingerLatch,
                    ref rightContactLostFor, ref rightContactQuality, rightContactMemory,
                    ref rightCandidate, ref rightCandidateQuality, ref rightCandidateGroups,
                    ref rightCandidateDistance, ref rightQualifiedCandidate, ref rightCandidateDwell,
                    false, rightVelocity, OVRInput.Controller.RTouch);
            }
            UpdateGraspFeedback();
        }

        private void UpdateHand(Transform hand, float command, ref G1Graspable held,
            ref ConfigurableJoint constraint, ref bool fingerLatch, ref float contactLostFor,
            ref float contactQuality,
            ContactMemory memory, ref G1Graspable candidate, ref float candidateQuality,
            ref int candidateGroups, ref float candidateDistance,
            ref G1Graspable qualifiedCandidate, ref float candidateDwell, bool left,
            Vector3 releaseVelocity, OVRInput.Controller controller)
        {
            if (fingerLatch && command <= releaseThreshold)
            {
                dex3Hands?.EndContactHold(left);
                fingerLatch = false;
            }

            if (held != null && constraint == null)
            {
                // A force or torque above the configured limit broke the physical grasp.
                held = null;
                contactLostFor = 0f;
                contactQuality = 0f;
                StartCoroutine(Pulse(controller, 0.35f));
            }

            if (held == null)
            {
                candidate = FindContactGrasp(hand, memory, out var contactPoint, out candidateQuality,
                    out candidateGroups, out candidateDistance);
                var qualifies = candidate != null && candidateQuality >= 0.55f &&
                                releaseVelocity.magnitude <= maximumAcquisitionSpeed;
                if (qualifies && candidate == qualifiedCandidate)
                    candidateDwell += Time.deltaTime;
                else
                {
                    qualifiedCandidate = qualifies ? candidate : null;
                    candidateDwell = 0f;
                }

                // A short dwell rejects fly-by collisions. The user still has to squeeze and
                // establish opposed thumb/finger or palm/finger proximity before assistance begins.
                if (!fingerLatch && command >= graspIntentThreshold && qualifies &&
                    candidateDwell >= acquisitionDwellSeconds)
                {
                    constraint = CreateGraspConstraint(hand, candidate, contactPoint, command, candidateQuality);
                    if (constraint != null)
                    {
                        held = candidate;
                        contactLostFor = 0f;
                        contactQuality = candidateQuality;
                        qualifiedCandidate = null;
                        candidateDwell = 0f;
                        memory.Clear();
                        dex3Hands?.BeginContactHold(left);
                        fingerLatch = true;
                        // Correct overlap once at acquisition, then keep this exact pose latched.
                        TryRelievePenetration(hand, held, left);
                        StartCoroutine(Pulse(controller));
                    }
                }
            }
            else if (held != null)
            {
                candidate = null;
                candidateQuality = 0f;
                candidateGroups = 0;
                candidateDistance = float.PositiveInfinity;
                qualifiedCandidate = null;
                candidateDwell = 0f;
                var liveQuality = EvaluateContactQuality(hand, held, null, out _,
                    Mathf.Max(1.5f, assistedAcquisitionTolerance / contactTolerance),
                    out _, out _);
                if (liveQuality >= 0.55f)
                {
                    contactLostFor = 0f;
                    contactQuality = Mathf.Lerp(contactQuality, liveQuality,
                        1f - Mathf.Exp(-12f * Time.deltaTime));
                }
                else
                    contactLostFor += Time.deltaTime;
                if (command <= releaseThreshold || contactLostFor >= contactLossGraceSeconds)
                    Release(ref held, ref constraint, ref fingerLatch, ref contactLostFor,
                        ref contactQuality, releaseVelocity, controller, left, command <= releaseThreshold);
                else
                    UpdateConstraintStrength(constraint, command, contactQuality);
            }
        }

        private bool TryRelievePenetration(Transform hand, G1Graspable graspable, bool left)
        {
            const float permittedPenetration = 0.002f;
            var maximumDepth = 0f;
            var handColliders = hand.GetComponentsInChildren<Collider>(true);
            var objectColliders = graspable.GetComponentsInChildren<Collider>();
            foreach (var handCollider in handColliders)
            {
                if (!IsUsableCollider(handCollider)) continue;
                foreach (var objectCollider in objectColliders)
                {
                    if (!IsUsableCollider(objectCollider)) continue;
                    if (Physics.ComputePenetration(handCollider, handCollider.transform.position,
                            handCollider.transform.rotation, objectCollider, objectCollider.transform.position,
                            objectCollider.transform.rotation, out _, out var distance))
                        maximumDepth = Mathf.Max(maximumDepth, distance);
                }
            }

            if (maximumDepth <= permittedPenetration)
                return false;

            // Apply one small correction at acquisition. Subsequent input cannot change this pose.
            var reliefDegrees = Mathf.Clamp((maximumDepth - permittedPenetration) * 1000f, 0.5f, 3f);
            dex3Hands?.RelieveContact(left, reliefDegrees);
            return true;
        }

        private static bool IsUsableCollider(Collider collider)
        {
            if (collider == null || !collider.enabled || collider.isTrigger || !collider.gameObject.activeInHierarchy)
                return false;
            return collider is not MeshCollider mesh || mesh.convex;
        }

        private G1Graspable FindContactGrasp(Transform hand, ContactMemory memory,
            out Vector3 contactPoint, out float quality, out int groupCount, out float nearestDistance)
        {
            G1Graspable nearest = null;
            var bestScore = float.NegativeInfinity;
            contactPoint = Vector3.zero;
            quality = 0f;
            groupCount = 0;
            nearestDistance = float.PositiveInfinity;
            foreach (var candidate in FindObjectsByType<G1Graspable>(FindObjectsSortMode.None))
            {
                if (candidate == leftHeld || candidate == rightHeld)
                    continue;
                var candidateQuality = EvaluateContactQuality(hand, candidate, memory,
                    out var candidatePoint,
                    Mathf.Max(1f, assistedAcquisitionTolerance / contactTolerance),
                    out var candidateGroups, out var candidateDistance);
                var score = candidateQuality * 10f + candidateGroups - Mathf.Min(candidateDistance, 1f);
                if (score <= bestScore)
                    continue;
                nearest = candidate;
                bestScore = score;
                contactPoint = candidatePoint;
                quality = candidateQuality;
                groupCount = candidateGroups;
                nearestDistance = candidateDistance;
            }
            return nearest;
        }

        private float EvaluateContactQuality(Transform hand, G1Graspable candidate, ContactMemory memory,
            out Vector3 graspPoint, float toleranceScale, out int groupCount, out float nearestDistance)
        {
            graspPoint = candidate.transform.position;
            groupCount = 0;
            nearestDistance = float.PositiveInfinity;
            var handColliders = hand.GetComponentsInChildren<Collider>(true);
            var objectColliders = candidate.GetComponentsInChildren<Collider>();
            if (objectColliders.Length == 0)
                return 0f;

            var objectBounds = objectColliders[0].bounds;
            for (var i = 1; i < objectColliders.Length; i++)
                if (objectColliders[i].enabled && !objectColliders[i].isTrigger)
                    objectBounds.Encapsulate(objectColliders[i].bounds);

            var thumb = new ContactSample();
            var index = new ContactSample();
            var middle = new ContactSample();
            var palm = new ContactSample();
            foreach (var objectCollider in objectColliders)
            {
                if (!objectCollider.enabled || objectCollider.isTrigger)
                    continue;
                foreach (var handCollider in handColliders)
                {
                    if (!handCollider.enabled || handCollider.isTrigger)
                        continue;
                    var objectPoint = objectCollider.ClosestPoint(handCollider.bounds.center);
                    var handPoint = handCollider.ClosestPoint(objectPoint);
                    objectPoint = objectCollider.ClosestPoint(handPoint);
                    var separation = Vector3.Distance(handPoint, objectPoint);
                    nearestDistance = Mathf.Min(nearestDistance, separation);
                    if (separation > contactTolerance * toleranceScale)
                        continue;
                    var point = (handPoint + objectPoint) * 0.5f;
                    var sample = new ContactSample(point, point - objectBounds.center, separation);
                    switch (ContactGroup(handCollider))
                    {
                        case 1 << 0: RecordContact(ref thumb, sample); break;
                        case 1 << 1: RecordContact(ref index, sample); break;
                        case 1 << 2: RecordContact(ref middle, sample); break;
                        default: RecordContact(ref palm, sample); break;
                    }
                }
            }

            if (memory != null)
            {
                memory.Update(candidate, thumb, index, middle, palm, Time.unscaledTime,
                    contactMemorySeconds);
                thumb = memory.Thumb.Sample;
                index = memory.Index.Sample;
                middle = memory.Middle.Sample;
                palm = memory.Palm.Sample;
            }

            groupCount = (thumb.Valid ? 1 : 0) + (index.Valid ? 1 : 0) +
                         (middle.Valid ? 1 : 0) + (palm.Valid ? 1 : 0);

            var bestOpposition = 0f;
            var bestPairMask = 0;
            var maximumSeparation = contactTolerance * toleranceScale;
            var bestProximity = 0f;
            EvaluatePair(thumb, index, (1 << 0) | (1 << 1), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);
            EvaluatePair(thumb, middle, (1 << 0) | (1 << 2), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);
            EvaluatePair(thumb, palm, (1 << 0) | (1 << 3), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);
            EvaluatePair(index, middle, (1 << 1) | (1 << 2), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);
            EvaluatePair(index, palm, (1 << 1) | (1 << 3), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);
            EvaluatePair(middle, palm, (1 << 2) | (1 << 3), maximumSeparation,
                ref bestOpposition, ref bestProximity, ref bestPairMask, ref graspPoint);

            // Thumb opposition is strongest. A palm/finger wrap is useful but weaker, while
            // two fingers without an opposing thumb or palm cannot form a reliable grasp.
            var pairFactor = (bestPairMask & (1 << 0)) != 0 ? 1f
                : (bestPairMask & (1 << 3)) != 0 ? 0.85f : 0.45f;
            // Near contacts receive at most 70% confidence at the edge of the assistance shell.
            // True collider contact receives full confidence, so exact grasps remain preferable.
            var proximityConfidence = Mathf.Lerp(0.7f, 1f, bestProximity);
            return Mathf.Clamp01(bestOpposition * pairFactor * proximityConfidence);
        }

        private ConfigurableJoint CreateGraspConstraint(Transform hand, G1Graspable candidate,
            Vector3 worldContactPoint, float command, float contactQuality)
        {
            var handArticulation = hand.GetComponent<ArticulationBody>() ??
                                   hand.GetComponentInParent<ArticulationBody>();
            var handBody = hand.GetComponent<Rigidbody>() ?? hand.GetComponentInParent<Rigidbody>();
            if (handArticulation == null && handBody == null)
            {
                Debug.LogWarning("Cannot create a dynamic grasp because the hand has no physics body.", this);
                return null;
            }

            var body = candidate.GetComponent<Rigidbody>();
            candidate.transform.SetParent(null, true);
            body.isKinematic = false;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.WakeUp();

            var joint = candidate.gameObject.AddComponent<ConfigurableJoint>();
            if (handArticulation != null) joint.connectedArticulationBody = handArticulation;
            else joint.connectedBody = handBody;
            joint.autoConfigureConnectedAnchor = false;
            // Bias the distributed contact anchor toward the centre of mass. This reduces the
            // artificial torque of a single-point joint while preserving the acquired pose.
            var stabilizedPoint = Vector3.Lerp(worldContactPoint, body.worldCenterOfMass, 0.55f);
            joint.anchor = candidate.transform.InverseTransformPoint(stabilizedPoint);
            var connectedTransform = handArticulation != null ? handArticulation.transform : handBody.transform;
            joint.connectedAnchor = connectedTransform.InverseTransformPoint(stabilizedPoint);
            joint.xMotion = joint.yMotion = joint.zMotion = ConfigurableJointMotion.Limited;
            joint.angularXMotion = joint.angularYMotion = joint.angularZMotion = ConfigurableJointMotion.Limited;
            joint.linearLimit = new SoftJointLimit { limit = 0.004f, contactDistance = 0.001f };
            joint.lowAngularXLimit = new SoftJointLimit { limit = -12f, contactDistance = 2f };
            joint.highAngularXLimit = new SoftJointLimit { limit = 12f, contactDistance = 2f };
            joint.angularYLimit = joint.angularZLimit = new SoftJointLimit { limit = 12f, contactDistance = 2f };
            joint.projectionMode = JointProjectionMode.PositionAndRotation;
            joint.projectionDistance = 0.006f;
            joint.projectionAngle = 8f;
            joint.enableCollision = true;
            joint.enablePreprocessing = true;
            UpdateConstraintStrength(joint, command, contactQuality);
            return joint;
        }

        private void UpdateConstraintStrength(ConfigurableJoint joint, float command, float contactQuality)
        {
            if (joint == null)
                return;
            var squeeze = Mathf.InverseLerp(releaseThreshold, 1f, command);
            var geometry = Mathf.InverseLerp(0.5f, 1f, contactQuality);
            var strength = Mathf.Clamp01(squeeze * geometry);
            joint.linearLimitSpring = new SoftJointLimitSpring
            {
                spring = Mathf.Lerp(100f, 900f, strength),
                damper = Mathf.Lerp(18f, 70f, strength)
            };
            joint.angularXLimitSpring = joint.angularYZLimitSpring = new SoftJointLimitSpring
            {
                spring = Mathf.Lerp(8f, 70f, strength),
                damper = Mathf.Lerp(2f, 12f, strength)
            };
            joint.breakForce = Mathf.Lerp(4f, maximumHoldingForce, strength);
            joint.breakTorque = Mathf.Lerp(0.4f, maximumHoldingTorque, strength);
        }

        private void Release(ref G1Graspable held, ref ConfigurableJoint constraint,
            ref bool fingerLatch, ref float contactLostFor, ref float contactQuality, Vector3 releaseVelocity,
            OVRInput.Controller controller, bool left, bool intentional)
        {
            if (constraint != null)
            {
                Destroy(constraint);
            }
            if (intentional)
            {
                dex3Hands?.EndContactHold(left);
                fingerLatch = false;
            }
            var body = held != null ? held.GetComponent<Rigidbody>() : null;
            if (body != null)
            {
                body.isKinematic = false;
                body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                if (intentional)
                    body.linearVelocity = Vector3.ClampMagnitude(releaseVelocity, 2f);
                body.WakeUp();
            }
            held = null;
            constraint = null;
            contactLostFor = 0f;
            contactQuality = 0f;
            StartCoroutine(Pulse(controller, intentional ? 0.25f : 0.4f));
        }

        private void UpdateGraspFeedback()
        {
            var leftReady = leftHeld == null && leftCandidate != null && leftCandidateQuality >= 0.55f;
            var rightReady = rightHeld == null && rightCandidate != null && rightCandidateQuality >= 0.55f;
            if (leftReady && !leftReadyLastFrame)
                StartCoroutine(Pulse(OVRInput.Controller.LTouch, 0.18f));
            if (rightReady && !rightReadyLastFrame)
                StartCoroutine(Pulse(OVRInput.Controller.RTouch, 0.18f));
            leftReadyLastFrame = leftReady;
            rightReadyLastFrame = rightReady;

            G1Graspable nextObject;
            G1GraspFeedback nextState;
            if (leftHeld != null || rightHeld != null)
            {
                var useLeft = leftHeld != null;
                nextObject = useLeft ? leftHeld : rightHeld;
                var lostFor = useLeft ? leftContactLostFor : rightContactLostFor;
                var quality = useLeft ? leftContactQuality : rightContactQuality;
                nextState = lostFor > 0.04f || quality < 0.62f
                    ? G1GraspFeedback.Slipping : G1GraspFeedback.Held;
            }
            else
            {
                var useLeft = leftCandidateQuality > rightCandidateQuality ||
                              (Mathf.Approximately(leftCandidateQuality, rightCandidateQuality) &&
                               leftCandidateDistance <= rightCandidateDistance);
                nextObject = useLeft ? leftCandidate : rightCandidate;
                var quality = useLeft ? leftCandidateQuality : rightCandidateQuality;
                var groups = useLeft ? leftCandidateGroups : rightCandidateGroups;
                var distance = useLeft ? leftCandidateDistance : rightCandidateDistance;
                nextState = quality >= 0.55f ? G1GraspFeedback.Ready
                    : groups > 0 ? G1GraspFeedback.Contact
                    : distance <= 0.06f ? G1GraspFeedback.Near : G1GraspFeedback.None;
            }

            if (feedbackObject != nextObject && feedbackObject != null)
                feedbackObject.SetFeedback(G1GraspFeedback.None);
            feedbackObject = nextObject;
            feedbackState = nextState;
            if (feedbackObject != null)
                feedbackObject.SetFeedback(feedbackState);
        }

        private static void RecordContact(ref ContactSample current, ContactSample candidate)
        {
            if (!current.Valid || candidate.Separation < current.Separation)
                current = candidate;
        }

        private static void EvaluatePair(ContactSample first, ContactSample second, int pairMask,
            float maximumSeparation, ref float bestOpposition, ref float bestProximity,
            ref int bestPairMask, ref Vector3 graspPoint)
        {
            if (!first.Valid || !second.Valid)
                return;
            var opposition = (1f - Vector3.Dot(first.Direction, second.Direction)) * 0.5f;
            var proximity = 1f - Mathf.Clamp01(Mathf.Max(first.Separation, second.Separation) /
                                                Mathf.Max(maximumSeparation, 0.0001f));
            var score = opposition * Mathf.Lerp(0.7f, 1f, proximity);
            var bestScore = bestOpposition * Mathf.Lerp(0.7f, 1f, bestProximity);
            if (score <= bestScore)
                return;
            bestOpposition = opposition;
            bestProximity = proximity;
            bestPairMask = pairMask;
            graspPoint = (first.Point + second.Point) * 0.5f;
        }

        private readonly struct ContactSample
        {
            public readonly bool Valid;
            public readonly Vector3 Point;
            public readonly Vector3 Direction;
            public readonly float Separation;

            public ContactSample(Vector3 point, Vector3 direction, float separation)
            {
                Valid = direction.sqrMagnitude > 0.000001f;
                Point = point;
                Direction = Valid ? direction.normalized : Vector3.zero;
                Separation = separation;
            }
        }

        private sealed class ContactMemory
        {
            public G1Graspable Candidate { get; private set; }
            public TimedContact Thumb;
            public TimedContact Index;
            public TimedContact Middle;
            public TimedContact Palm;

            public void Update(G1Graspable candidate, ContactSample thumb, ContactSample index,
                ContactSample middle, ContactSample palm, float now, float lifetime)
            {
                if (Candidate != candidate)
                {
                    Clear();
                    Candidate = candidate;
                }
                UpdateContact(ref Thumb, thumb, now, lifetime);
                UpdateContact(ref Index, index, now, lifetime);
                UpdateContact(ref Middle, middle, now, lifetime);
                UpdateContact(ref Palm, palm, now, lifetime);
            }

            public void Clear()
            {
                Candidate = null;
                Thumb = Index = Middle = Palm = default;
            }

            private static void UpdateContact(ref TimedContact remembered, ContactSample current,
                float now, float lifetime)
            {
                if (current.Valid)
                    remembered = new TimedContact(current, now);
                else if (remembered.Sample.Valid && now - remembered.SeenAt > lifetime)
                    remembered = default;
            }
        }

        private readonly struct TimedContact
        {
            public readonly ContactSample Sample;
            public readonly float SeenAt;

            public TimedContact(ContactSample sample, float seenAt)
            {
                Sample = sample;
                SeenAt = seenAt;
            }
        }

        private static int ContactGroup(Collider collider)
        {
            var current = collider.transform;
            while (current != null)
            {
                var jointName = current.name.ToLowerInvariant();
                if (jointName.Contains("thumb")) return 1 << 0;
                if (jointName.Contains("index")) return 1 << 1;
                if (jointName.Contains("middle")) return 1 << 2;
                current = current.parent;
            }
            return 1 << 3; // Palm or fixed hand shell.
        }

        private IEnumerator Pulse(OVRInput.Controller controller, float amplitudeScale = 1f)
        {
            OVRInput.SetControllerVibration(0.7f, hapticAmplitude * amplitudeScale, controller);
            yield return new WaitForSeconds(0.08f);
            OVRInput.SetControllerVibration(0f, 0f, controller);
        }

        private Transform FindDescendant(string objectName)
        {
            foreach (var child in GetComponentsInChildren<Transform>(true))
                if (child.name == objectName) return child;
            return null;
        }
    }
}
