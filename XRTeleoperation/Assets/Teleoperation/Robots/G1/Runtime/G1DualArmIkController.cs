using UnityEngine;

namespace Teleoperation.Robots.G1
{
    /// <summary>
    /// Position-only damped least-squares IK for the G1's two seven-joint arms.
    /// Commands articulation drives; it never writes robot link transforms.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(G1JointController))]
    public sealed class G1DualArmIkController : MonoBehaviour
    {
        private static readonly int[] LeftArm = { 15, 16, 17, 18, 19, 20, 21 };
        private static readonly int[] RightArm = { 22, 23, 24, 25, 26, 27, 28 };
        private static readonly float[] OrientationWeights = { 0.03f, 0.03f, 0.05f, 0.1f, 1f, 1f, 1f };

        [SerializeField] private bool solve = true;
        [SerializeField, Min(0.001f)] private float positionTolerance = 0.018f;
        [SerializeField, Min(0.1f)] private float maximumArmReach = 0.72f;
        [SerializeField, Range(0.01f, 0.5f)] private float damping = 0.14f;
        [SerializeField, Min(0.1f)] private float gain = 1.4f;
        [SerializeField, Min(0.1f)] private float maximumJointStepDegrees = 1.5f;
        [SerializeField, Min(0.05f)] private float maximumTargetSpeed = 0.45f;
        [SerializeField, Min(1f)] private float orientationToleranceDegrees = 6f;
        [SerializeField, Min(1f)] private float maximumTargetAngularSpeed = 150f;
        [SerializeField, Min(0.01f)] private float orientationGain = 0.8f;
        [SerializeField, Range(0f, 1f)] private float postureStabilizationGain = 0.35f;
        [SerializeField, Min(0.05f)] private float torsoExclusionRadius = 0.16f;
        [SerializeField, Min(0.05f)] private float minimumHandSeparation = 0.12f;
        [Header("Contact safety")]
        [SerializeField, Min(0.01f)] private float handCollisionRadius = 0.05f;
        [SerializeField, Range(0.001f, 0.03f)] private float collisionSkin = 0.006f;

        private G1JointController robot;
        private Transform leftEndEffector;
        private Transform rightEndEffector;
        private Transform leftTarget;
        private Transform rightTarget;
        private G1IkTarget leftFeedback;
        private G1IkTarget rightFeedback;
        private Vector3 leftFilteredTarget;
        private Vector3 rightFilteredTarget;
        private Quaternion leftFilteredRotation;
        private Quaternion rightFilteredRotation;
        private Transform leftValidatedPose;
        private Transform rightValidatedPose;
        private Transform torso;
        private readonly float[] leftPostureReference = new float[7];
        private readonly float[] rightPostureReference = new float[7];
        private Vector3 leftElbowHint;
        private Vector3 rightElbowHint;
        private bool elbowHintsActive;
        private readonly RaycastHit[] collisionHits = new RaycastHit[32];

        public Transform LeftTarget => leftTarget;
        public Transform RightTarget => rightTarget;
        public Transform LeftEndEffector => leftEndEffector;
        public Transform RightEndEffector => rightEndEffector;
        public bool Solve
        {
            get => solve;
            set
            {
                solve = value;
                if (!solve)
                {
                    leftFeedback?.SetStatus(G1IkStatus.Disabled);
                    rightFeedback?.SetStatus(G1IkStatus.Disabled);
                }
            }
        }

        public void ResetTargetsToHands()
        {
            if (leftTarget != null && leftEndEffector != null)
            {
                leftTarget.SetPositionAndRotation(leftEndEffector.position, leftEndEffector.rotation);
                leftFilteredTarget = leftTarget.position;
                leftFilteredRotation = leftTarget.rotation;
            }
            if (rightTarget != null && rightEndEffector != null)
            {
                rightTarget.SetPositionAndRotation(rightEndEffector.position, rightEndEffector.rotation);
                rightFilteredTarget = rightTarget.position;
                rightFilteredRotation = rightTarget.rotation;
            }
        }

        public void CapturePostureReference(bool leftArm)
        {
            var indices = leftArm ? LeftArm : RightArm;
            var reference = leftArm ? leftPostureReference : rightPostureReference;
            for (var i = 0; i < indices.Length; i++)
                reference[i] = robot.GetCurrentJointPositionDegrees(indices[i]);
        }

        public void SetElbowHints(Vector3 left, Vector3 right, bool active)
        {
            leftElbowHint = left;
            rightElbowHint = right;
            elbowHintsActive = active;
        }

        private void Awake()
        {
            robot = GetComponent<G1JointController>();
            robot.KeyboardValidationEnabled = false;
            leftEndEffector = FindDescendant("left_rubber_hand");
            rightEndEffector = FindDescendant("right_rubber_hand");
            torso = FindDescendant("torso_link");
            if (leftEndEffector != null)
                leftTarget = CreateTarget("G1 Left Hand Requested Pose", leftEndEffector.position, leftEndEffector.rotation, new Color(0.1f, 0.75f, 1f));
            if (rightEndEffector != null)
                rightTarget = CreateTarget("G1 Right Hand Requested Pose", rightEndEffector.position, rightEndEffector.rotation, new Color(1f, 0.45f, 0.1f));
            if (leftTarget != null)
                leftValidatedPose = CreateValidatedPose("G1 Left Validated Pose", leftTarget.position, leftTarget.rotation);
            if (rightTarget != null)
                rightValidatedPose = CreateValidatedPose("G1 Right Validated Pose", rightTarget.position, rightTarget.rotation);
            leftFeedback = leftTarget != null ? leftTarget.GetComponent<G1IkTarget>() : null;
            rightFeedback = rightTarget != null ? rightTarget.GetComponent<G1IkTarget>() : null;
            leftFilteredTarget = leftTarget != null ? leftTarget.position : Vector3.zero;
            rightFilteredTarget = rightTarget != null ? rightTarget.position : Vector3.zero;
            leftFilteredRotation = leftTarget != null ? leftTarget.rotation : Quaternion.identity;
            rightFilteredRotation = rightTarget != null ? rightTarget.rotation : Quaternion.identity;
            CapturePostureReference(true);
            CapturePostureReference(false);
        }

        private void FixedUpdate()
        {
            if (!solve || robot == null)
            {
                leftFeedback?.SetStatus(G1IkStatus.Disabled);
                rightFeedback?.SetStatus(G1IkStatus.Disabled);
                return;
            }

            var handsTooClose = leftTarget != null && rightTarget != null &&
                                Vector3.Distance(leftTarget.position, rightTarget.position) < minimumHandSeparation;
            SolveArm(LeftArm, leftEndEffector, leftTarget, leftValidatedPose, leftFeedback,
                ref leftFilteredTarget, ref leftFilteredRotation, handsTooClose, leftPostureReference, leftElbowHint);
            SolveArm(RightArm, rightEndEffector, rightTarget, rightValidatedPose, rightFeedback,
                ref rightFilteredTarget, ref rightFilteredRotation, handsTooClose, rightPostureReference, rightElbowHint);
            robot.ApplyTargets();
        }

        private void SolveArm(int[] indices, Transform endEffector, Transform target, Transform validatedPose,
            G1IkTarget feedback, ref Vector3 filteredTarget, ref Quaternion filteredRotation, bool handsTooClose,
            float[] postureReference, Vector3 elbowHint)
        {
            if (endEffector == null || target == null)
                return;

            var shoulder = robot.GetJoint(indices[0]);
            var insideTorso = torso != null && Vector3.Distance(torso.position, target.position) < torsoExclusionRadius;
            if (shoulder == null || Vector3.Distance(shoulder.transform.position, target.position) > maximumArmReach ||
                insideTorso || handsTooClose)
            {
                feedback?.SetStatus(G1IkStatus.Unreachable);
                return;
            }

            var speedLimitedTarget = Vector3.MoveTowards(filteredTarget, target.position,
                maximumTargetSpeed * Time.fixedDeltaTime);
            filteredTarget = ClampAgainstEnvironment(endEffector, speedLimitedTarget);
            filteredRotation = Quaternion.RotateTowards(filteredRotation, target.rotation,
                maximumTargetAngularSpeed * Time.fixedDeltaTime);
            if (validatedPose != null)
                validatedPose.SetPositionAndRotation(filteredTarget, filteredRotation);
            var finalError = target.position - endEffector.position;
            var finalAngularError = Quaternion.Angle(endEffector.rotation, target.rotation);
            if (finalError.magnitude <= positionTolerance && finalAngularError <= orientationToleranceDegrees)
            {
                feedback?.SetStatus(G1IkStatus.Reached);
                return;
            }

            feedback?.SetStatus(G1IkStatus.Solving);
            var columns = new Vector3[indices.Length];
            var angularColumns = new Vector3[indices.Length];
            var jjt = Matrix3.Zero;
            for (var column = 0; column < indices.Length; column++)
            {
                var joint = robot.GetJoint(indices[column]);
                if (joint == null || joint.transform.parent == null)
                    return;
                var parent = joint.transform.parent;
                var pivot = parent.TransformPoint(joint.parentAnchorPosition);
                var axis = parent.TransformDirection(joint.parentAnchorRotation * Vector3.right).normalized;
                angularColumns[column] = axis;
                columns[column] = Vector3.Cross(axis, endEffector.position - pivot);
                jjt += Matrix3.Outer(columns[column], columns[column]);
            }

            var stableDamping = damping * damping;
            jjt.m00 += stableDamping;
            jjt.m11 += stableDamping;
            jjt.m22 += stableDamping;
            if (!jjt.TryInverse(out var inverse))
            {
                feedback?.SetStatus(G1IkStatus.Unreachable);
                return;
            }

            var solved = inverse * ((filteredTarget - endEffector.position) * gain * Time.fixedDeltaTime);
            var rotationError = filteredRotation * Quaternion.Inverse(endEffector.rotation);
            rotationError.ToAngleAxis(out var errorDegrees, out var errorAxis);
            if (errorDegrees > 180f) errorDegrees -= 360f;
            var angularError = errorAxis.normalized * (errorDegrees * Mathf.Deg2Rad);
            var elbow = robot.GetJoint(indices[3]);
            var elbowError = elbowHintsActive && elbow != null
                ? Vector3.ClampMagnitude(elbowHint - elbow.transform.position, 0.2f)
                : Vector3.zero;
            var postureCorrection = new float[indices.Length];
            for (var joint = 0; joint < indices.Length; joint++)
            {
                postureCorrection[joint] = (postureReference[joint] -
                    robot.GetCurrentJointPositionDegrees(indices[joint])) * Mathf.Deg2Rad *
                    postureStabilizationGain * Time.fixedDeltaTime;
            }

            for (var column = 0; column < indices.Length; column++)
            {
                var index = indices[column];
                var positionDelta = Vector3.Dot(columns[column], solved);
                var orientationDelta = Vector3.Dot(angularColumns[column], angularError) *
                                       orientationGain * OrientationWeights[column] * Time.fixedDeltaTime;
                var elbowDelta = 0f;
                if (column < 3 && elbow != null && elbowHintsActive)
                {
                    var joint = robot.GetJoint(indices[column]);
                    var parent = joint.transform.parent;
                    var pivot = parent.TransformPoint(joint.parentAnchorPosition);
                    var axis = parent.TransformDirection(joint.parentAnchorRotation * Vector3.right).normalized;
                    var elbowColumn = Vector3.Cross(axis, elbow.transform.position - pivot);
                    elbowDelta = Vector3.Dot(elbowColumn, elbowError) * 0.55f * Time.fixedDeltaTime;
                }
                // N = I - J^T(JJ^T + lambda^2 I)^-1 J keeps the preferred posture
                // correction in the position task's null space.
                var nullSpaceDelta = postureCorrection[column];
                for (var source = 0; source < indices.Length; source++)
                    nullSpaceDelta -= Vector3.Dot(columns[column], inverse * columns[source]) *
                                      postureCorrection[source];
                var delta = Mathf.Clamp((positionDelta + orientationDelta + elbowDelta + nullSpaceDelta) * Mathf.Rad2Deg,
                    -maximumJointStepDegrees, maximumJointStepDegrees);
                robot.SetTargetDegrees(index, robot.GetCurrentJointPositionDegrees(index) + delta);
            }
        }

        private Vector3 ClampAgainstEnvironment(Transform endEffector, Vector3 desiredPosition)
        {
            var movement = desiredPosition - endEffector.position;
            var distance = movement.magnitude;
            if (distance < 0.0001f)
                return desiredPosition;

            var direction = movement / distance;
            var hitCount = Physics.SphereCastNonAlloc(endEffector.position, handCollisionRadius, direction,
                collisionHits, distance + collisionSkin, ~0, QueryTriggerInteraction.Ignore);
            var allowedDistance = distance;
            for (var i = 0; i < hitCount; i++)
            {
                var collider = collisionHits[i].collider;
                if (collider == null || collider.transform.IsChildOf(transform))
                    continue;

                // A grasped object is parented beneath its hand and must travel with that hand.
                if (collider.transform.IsChildOf(endEffector))
                    continue;

                allowedDistance = Mathf.Min(allowedDistance,
                    Mathf.Max(0f, collisionHits[i].distance - collisionSkin));
            }
            return endEffector.position + direction * allowedDistance;
        }

        private Transform CreateTarget(string targetName, Vector3 position, Quaternion rotation, Color color)
        {
            var targetObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            targetObject.name = targetName;
            targetObject.transform.SetPositionAndRotation(position, rotation);
            targetObject.transform.localScale = Vector3.one * 0.075f;
            var collider = targetObject.GetComponent<Collider>();
            if (collider != null)
                collider.isTrigger = true;
            var renderer = targetObject.GetComponent<Renderer>();
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null)
                renderer.material = new Material(shader) { color = color };
            targetObject.AddComponent<G1IkTarget>();
            targetObject.AddComponent<G1TargetMouseDrag>();
            CreateAxis(targetObject.transform, Vector3.right, Color.red);
            CreateAxis(targetObject.transform, Vector3.up, Color.green);
            CreateAxis(targetObject.transform, Vector3.forward, Color.blue);
            return targetObject.transform;
        }

        private Transform CreateValidatedPose(string objectName, Vector3 position, Quaternion rotation)
        {
            var marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            marker.name = objectName;
            marker.transform.SetPositionAndRotation(position, rotation);
            marker.transform.localScale = Vector3.one * 0.045f;
            Destroy(marker.GetComponent<Collider>());
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null) marker.GetComponent<Renderer>().material = new Material(shader) { color = new Color(0.2f, 1f, 0.65f) };
            return marker.transform;
        }

        private static void CreateAxis(Transform parent, Vector3 axis, Color color)
        {
            var axisObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            axisObject.name = $"Pose Axis {axis}";
            axisObject.transform.SetParent(parent, false);
            axisObject.transform.localPosition = axis * 0.7f;
            axisObject.transform.localScale = new Vector3(
                axis.x != 0f ? 1.2f : 0.12f,
                axis.y != 0f ? 1.2f : 0.12f,
                axis.z != 0f ? 1.2f : 0.12f);
            Destroy(axisObject.GetComponent<Collider>());
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            if (shader != null) axisObject.GetComponent<Renderer>().material = new Material(shader) { color = color };
        }

        private Transform FindDescendant(string objectName)
        {
            foreach (var child in GetComponentsInChildren<Transform>(true))
                if (child.name == objectName)
                    return child;
            return null;
        }

        private void OnGUI()
        {
            GUI.Box(new Rect(18f, 18f, 610f, 94f), string.Empty);
            GUI.Label(new Rect(32f, 28f, 580f, 24f), "G1 DUAL-ARM IK — POSITION VALIDATION");
            GUI.Label(new Rect(32f, 52f, 580f, 24f),
                $"Left: {leftFeedback?.Status.ToString() ?? "Missing"}    Right: {rightFeedback?.Status.ToString() ?? "Missing"}");
            GUI.Label(new Rect(32f, 76f, 580f, 24f),
                "Axis sphere=requested | Green cube=validated | Robot hand=measured | Red=rejected");
        }

        private struct Matrix3
        {
            public float m00, m01, m02, m10, m11, m12, m20, m21, m22;
            public static Matrix3 Zero => default;
            public static Matrix3 Outer(Vector3 a, Vector3 b) => new()
            {
                m00 = a.x * b.x, m01 = a.x * b.y, m02 = a.x * b.z,
                m10 = a.y * b.x, m11 = a.y * b.y, m12 = a.y * b.z,
                m20 = a.z * b.x, m21 = a.z * b.y, m22 = a.z * b.z
            };
            public static Matrix3 operator +(Matrix3 a, Matrix3 b) => new()
            {
                m00 = a.m00 + b.m00, m01 = a.m01 + b.m01, m02 = a.m02 + b.m02,
                m10 = a.m10 + b.m10, m11 = a.m11 + b.m11, m12 = a.m12 + b.m12,
                m20 = a.m20 + b.m20, m21 = a.m21 + b.m21, m22 = a.m22 + b.m22
            };
            public static Vector3 operator *(Matrix3 m, Vector3 v) => new(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z);
            public bool TryInverse(out Matrix3 inverse)
            {
                var c00 = m11 * m22 - m12 * m21;
                var c01 = m02 * m21 - m01 * m22;
                var c02 = m01 * m12 - m02 * m11;
                var determinant = m00 * c00 + m10 * c01 + m20 * c02;
                if (Mathf.Abs(determinant) < 1e-8f) { inverse = default; return false; }
                var inv = 1f / determinant;
                inverse = new Matrix3
                {
                    m00 = c00 * inv, m01 = c01 * inv, m02 = c02 * inv,
                    m10 = (m12 * m20 - m10 * m22) * inv,
                    m11 = (m00 * m22 - m02 * m20) * inv,
                    m12 = (m02 * m10 - m00 * m12) * inv,
                    m20 = (m10 * m21 - m11 * m20) * inv,
                    m21 = (m01 * m20 - m00 * m21) * inv,
                    m22 = (m00 * m11 - m01 * m10) * inv
                };
                return true;
            }
        }
    }
}
