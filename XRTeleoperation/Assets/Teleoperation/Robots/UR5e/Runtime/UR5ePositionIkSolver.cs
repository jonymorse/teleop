using UnityEngine;

namespace Teleoperation.Robots.UR5e
{
    /// <summary>
    /// Resolved-rate position IK using a damped least-squares geometric Jacobian.
    /// It commands joint drive targets and never writes link transforms directly.
    /// </summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(UR5eJointController))]
    public sealed class UR5ePositionIkSolver : MonoBehaviour
    {
        [SerializeField] private UR5eJointController robot;
        [SerializeField] private Transform endEffector;
        [SerializeField] private Transform target;
        [SerializeField] private bool solve = true;
        [SerializeField, Min(0.001f)] private float positionTolerance = 0.015f;
        [SerializeField, Min(0.1f)] private float maximumReach = 0.95f;
        [SerializeField, Range(0.001f, 1f)] private float damping = 0.08f;
        [SerializeField, Min(0.01f)] private float gain = 1.5f;
        [SerializeField, Min(0.1f)] private float maximumJointStepDegrees = 2f;

        private UR5eIkTarget targetFeedback;
        private ArticulationBody rootBody;
        private bool previousKeyboardControlState;

        public Transform Target => target;
        public Transform EndEffector => endEffector;
        public bool Solve
        {
            get => solve;
            set => solve = value;
        }

        private void Reset()
        {
            robot = GetComponent<UR5eJointController>();
        }

        private void Awake()
        {
            robot ??= GetComponent<UR5eJointController>();
            previousKeyboardControlState = robot.KeyboardControlEnabled;
            robot.KeyboardControlEnabled = false;
            endEffector ??= FindDescendant(transform, "tool0");
            rootBody = FindRootBody();

            if (target == null && endEffector != null)
                target = CreateRuntimeTarget(endEffector.position);

            if (target != null)
            {
                targetFeedback = target.GetComponent<UR5eIkTarget>();
                var collider = target.GetComponent<Collider>();
                if (collider == null)
                    collider = target.gameObject.AddComponent<SphereCollider>();
                collider.isTrigger = true;
                if (target.GetComponent<UR5eTargetMouseDrag>() == null)
                    target.gameObject.AddComponent<UR5eTargetMouseDrag>();
            }
        }

        private void OnDisable()
        {
            if (robot != null)
                robot.KeyboardControlEnabled = previousKeyboardControlState;
        }

        private void FixedUpdate()
        {
            if (!solve || robot == null || endEffector == null || target == null || rootBody == null)
            {
                targetFeedback?.SetStatus(IkTargetStatus.Disabled);
                return;
            }

            var basePosition = rootBody.transform.position;
            if (Vector3.Distance(basePosition, target.position) > maximumReach)
            {
                targetFeedback?.SetStatus(IkTargetStatus.Unreachable);
                return;
            }

            var error = target.position - endEffector.position;
            if (error.magnitude <= positionTolerance)
            {
                targetFeedback?.SetStatus(IkTargetStatus.Reachable);
                return;
            }

            targetFeedback?.SetStatus(IkTargetStatus.Solving);
            StepSolver(error);
        }

        private void StepSolver(Vector3 error)
        {
            var columns = new Vector3[robot.JointCount];
            var jjt = Matrix3x3.Zero;

            for (var i = 0; i < robot.JointCount; i++)
            {
                var joint = robot.GetJoint(i);
                if (joint == null || joint.transform.parent == null)
                    return;

                var parent = joint.transform.parent;
                var pivot = parent.TransformPoint(joint.parentAnchorPosition);
                var axis = parent.TransformDirection(joint.parentAnchorRotation * Vector3.right).normalized;
                columns[i] = Vector3.Cross(axis, endEffector.position - pivot);
                jjt += Matrix3x3.Outer(columns[i], columns[i]);
            }

            jjt.m00 += damping * damping;
            jjt.m11 += damping * damping;
            jjt.m22 += damping * damping;

            if (!jjt.TryInverse(out var inverse))
            {
                targetFeedback?.SetStatus(IkTargetStatus.Unreachable);
                return;
            }

            var solvedError = inverse * (error * gain);
            for (var i = 0; i < robot.JointCount; i++)
            {
                var deltaRadians = Vector3.Dot(columns[i], solvedError);
                var deltaDegrees = Mathf.Clamp(
                    deltaRadians * Mathf.Rad2Deg,
                    -maximumJointStepDegrees,
                    maximumJointStepDegrees);
                robot.SetTargetDegrees(i, robot.GetTargetDegrees(i) + deltaDegrees);
            }

            robot.ApplyTargets();
        }

        private ArticulationBody FindRootBody()
        {
            foreach (var body in GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.isRoot)
                    return body;
            }

            return null;
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

        private static Transform CreateRuntimeTarget(Vector3 position)
        {
            var targetObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            targetObject.name = "UR5e IK Target";
            targetObject.transform.position = position;
            targetObject.transform.localScale = Vector3.one * 0.09f;

            var collider = targetObject.GetComponent<Collider>();
            if (collider != null)
                collider.isTrigger = true;

            targetObject.AddComponent<UR5eIkTarget>();
            targetObject.AddComponent<UR5eTargetMouseDrag>();
            return targetObject.transform;
        }

        private struct Matrix3x3
        {
            public float m00, m01, m02;
            public float m10, m11, m12;
            public float m20, m21, m22;

            public static Matrix3x3 Zero => default;

            public static Matrix3x3 Outer(Vector3 a, Vector3 b) => new()
            {
                m00 = a.x * b.x, m01 = a.x * b.y, m02 = a.x * b.z,
                m10 = a.y * b.x, m11 = a.y * b.y, m12 = a.y * b.z,
                m20 = a.z * b.x, m21 = a.z * b.y, m22 = a.z * b.z
            };

            public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b) => new()
            {
                m00 = a.m00 + b.m00, m01 = a.m01 + b.m01, m02 = a.m02 + b.m02,
                m10 = a.m10 + b.m10, m11 = a.m11 + b.m11, m12 = a.m12 + b.m12,
                m20 = a.m20 + b.m20, m21 = a.m21 + b.m21, m22 = a.m22 + b.m22
            };

            public static Vector3 operator *(Matrix3x3 m, Vector3 v) => new(
                m.m00 * v.x + m.m01 * v.y + m.m02 * v.z,
                m.m10 * v.x + m.m11 * v.y + m.m12 * v.z,
                m.m20 * v.x + m.m21 * v.y + m.m22 * v.z);

            public bool TryInverse(out Matrix3x3 inverse)
            {
                var c00 = m11 * m22 - m12 * m21;
                var c01 = m02 * m21 - m01 * m22;
                var c02 = m01 * m12 - m02 * m11;
                var determinant = m00 * c00 + m10 * c01 + m20 * c02;
                if (Mathf.Abs(determinant) < 1e-8f)
                {
                    inverse = default;
                    return false;
                }

                var inv = 1f / determinant;
                inverse = new Matrix3x3
                {
                    m00 = c00 * inv,
                    m01 = c01 * inv,
                    m02 = c02 * inv,
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
