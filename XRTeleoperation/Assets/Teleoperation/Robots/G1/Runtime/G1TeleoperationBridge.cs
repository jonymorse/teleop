using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

namespace Teleoperation.Robots.G1
{
    /// <summary>Simulation-only UDP boundary between the Quest frontend and a G1 IK backend.</summary>
    [DisallowMultipleComponent]
    [RequireComponent(typeof(G1JointController), typeof(G1DualArmIkController))]
    public sealed class G1TeleoperationBridge : MonoBehaviour
    {
        private const int ProtocolVersion = 1;
        [SerializeField] private string backendHost = "192.168.1.209";
        [SerializeField] private int backendPort = 7447;
        [SerializeField] private int commandPort = 7448;
        [SerializeField, Range(10f, 90f)] private float sendRateHz = 30f;
        [SerializeField, Min(0.05f)] private float commandTimeoutSeconds = 0.35f;

        private readonly int[] armIndices = { 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };
        private G1JointController robot;
        private G1DualArmIkController ik;
        private G1Dex3HandController dex3Hands;
        private UdpClient sender;
        private UdpClient receiver;
        private IPEndPoint backendEndpoint;
        private IPEndPoint configuredEndpoint;
        private byte[] pendingCommand;
        private readonly object receiveLock = new();
        private float nextSendTime;
        private float lastCommandTime = -999f;
        private uint sequence;
        private bool solverResetRequested;

        public bool Active { get; set; }
        public bool RetargetQuestHands { get; set; }
        public bool Connected => Active && Time.unscaledTime - lastCommandTime <= commandTimeoutSeconds;
        public string Status => !Active ? "disabled" : Connected ? "backend connected" : "discovering backend on UDP 7447";

        public void RequestSolverReset() => solverResetRequested = true;

        private void Awake()
        {
            robot = GetComponent<G1JointController>();
            ik = GetComponent<G1DualArmIkController>();
            dex3Hands = GetComponent<G1Dex3HandController>();
            if (!string.IsNullOrWhiteSpace(backendHost) && IPAddress.TryParse(backendHost, out var address))
                configuredEndpoint = new IPEndPoint(address, backendPort);
            try
            {
                sender = new UdpClient { EnableBroadcast = true };
                receiver = new UdpClient(commandPort);
                receiver.BeginReceive(OnReceive, null);
            }
            catch (Exception exception)
            {
                Debug.LogError($"G1 backend bridge could not open UDP ports: {exception.Message}", this);
            }
        }

        private void Update()
        {
            ApplyPendingCommand();
            if (!Active || sender == null || ik.LeftTarget == null || ik.RightTarget == null ||
                Time.unscaledTime < nextSendTime)
                return;

            dex3Hands ??= GetComponent<G1Dex3HandController>();

            nextSendTime = Time.unscaledTime + 1f / sendRateHz;
            var packet = new PosePacket
            {
                version = ProtocolVersion,
                type = "pose",
                sequence = ++sequence,
                replyPort = commandPort,
                left = ToRobotPose(ik.LeftTarget),
                right = ToRobotPose(ik.RightTarget),
                current = CurrentArmRadians(),
                // Touch-controller fingers are applied locally for minimum latency. Only the
                // dedicated Quest-hand mode receives hand commands back from the backend.
                handRadians = null,
                simulationOnly = true,
                resetSolver = solverResetRequested
            };
            if (RetargetQuestHands && GetComponent<G1BodyTrackingController>() is { } tracking &&
                tracking.TryGetUnitreeHandKeypoints(out var leftHand, out var rightHand))
            {
                packet.retargetHands = true;
                packet.leftHandKeypoints = leftHand;
                packet.rightHandKeypoints = rightHand;
                packet.handRadians = null;
            }
            solverResetRequested = false;
            var bytes = Encoding.UTF8.GetBytes(JsonUtility.ToJson(packet));
            // Quest/Android global UDP broadcast is frequently filtered by Wi-Fi access points.
            // Prefer the configured PC address, then retain broadcast for portable setups where
            // no host is configured. A successful response replaces either with its source IP.
            var endpoint = backendEndpoint ?? configuredEndpoint ??
                           new IPEndPoint(IPAddress.Broadcast, backendPort);
            try { sender.Send(bytes, bytes.Length, endpoint); }
            catch (Exception exception) { Debug.LogWarning($"G1 backend send failed: {exception.Message}", this); }
        }

        private float[] ToRobotPose(Transform target)
        {
            robot.TryGetRootBody(out var root);
            var localPosition = root != null ? root.transform.InverseTransformPoint(target.position) : target.position;
            var localRotation = root != null ? Quaternion.Inverse(root.transform.rotation) * target.rotation : target.rotation;
            // Unity (right, up, forward) -> Unitree (forward, left, up).
            var xAxis = ToRobotVector(localRotation * Vector3.forward);
            var yAxis = ToRobotVector(localRotation * Vector3.left);
            var zAxis = ToRobotVector(localRotation * Vector3.up);
            return new[] { localPosition.z, -localPosition.x, localPosition.y,
                xAxis.x, yAxis.x, zAxis.x,
                xAxis.y, yAxis.y, zAxis.y,
                xAxis.z, yAxis.z, zAxis.z };
        }

        private static Vector3 ToRobotVector(Vector3 value) => new(value.z, -value.x, value.y);

        private float[] CurrentArmRadians()
        {
            var values = new float[armIndices.Length];
            for (var i = 0; i < values.Length; i++)
                values[i] = robot.GetCurrentJointPositionDegrees(armIndices[i]) * Mathf.Deg2Rad;
            return values;
        }

        private void OnReceive(IAsyncResult result)
        {
            try
            {
                var source = new IPEndPoint(IPAddress.Any, 0);
                var bytes = receiver.EndReceive(result, ref source);
                lock (receiveLock) pendingCommand = bytes;
                backendEndpoint = new IPEndPoint(source.Address, backendPort);
            }
            catch (ObjectDisposedException) { return; }
            catch (Exception exception) { Debug.LogWarning($"G1 backend receive failed: {exception.Message}", this); }
            finally
            {
                try { receiver?.BeginReceive(OnReceive, null); }
                catch (ObjectDisposedException) { }
            }
        }

        private void ApplyPendingCommand()
        {
            byte[] bytes;
            lock (receiveLock) { bytes = pendingCommand; pendingCommand = null; }
            if (bytes == null || !Active)
                return;
            var command = JsonUtility.FromJson<JointPacket>(Encoding.UTF8.GetString(bytes));
            if (command == null || command.version != ProtocolVersion || command.type != "joints" ||
                command.armRadians == null || command.armRadians.Length != armIndices.Length || !command.valid)
                return;
            for (var i = 0; i < armIndices.Length; i++)
                robot.SetTargetDegrees(armIndices[i], command.armRadians[i] * Mathf.Rad2Deg);
            robot.ApplyTargets();
            if (RetargetQuestHands && command.handRadians != null && command.handRadians.Length == 14)
                dex3Hands?.SetRetargetedRadians(command.handRadians);
            lastCommandTime = Time.unscaledTime;
        }

        private void OnDestroy()
        {
            receiver?.Close();
            sender?.Close();
        }

        [Serializable] private sealed class PosePacket
        {
            public int version; public string type; public uint sequence; public int replyPort;
            public float[] left; public float[] right; public float[] current; public bool simulationOnly;
            public float[] handRadians;
            public bool retargetHands; public float[] leftHandKeypoints; public float[] rightHandKeypoints;
            public bool resetSolver;
        }

        [Serializable] private sealed class JointPacket
        {
            public int version; public string type; public uint sequence; public bool valid;
            public float[] armRadians; public float[] handRadians; public string status;
        }
    }
}
