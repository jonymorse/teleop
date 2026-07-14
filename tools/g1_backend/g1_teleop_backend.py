#!/usr/bin/env python3
"""Simulation-only Unity bridge for Unitree's official G1_29_ArmIK solver."""

import argparse
import json
import os
import socket
import struct
import sys
import time
from pathlib import Path


def pose_matrix(values):
    if len(values) != 12:
        raise ValueError("pose must contain position plus a row-major 3x3 rotation")
    import numpy as np
    matrix = np.eye(4)
    matrix[:3, 3] = values[:3]
    matrix[:3, :3] = np.asarray(values[3:]).reshape(3, 3)
    return matrix


class OfficialUnitreeSolver:
    def __init__(self, repository):
        root = Path(repository).resolve()
        teleop = root / "teleop"
        if not (teleop / "robot_control" / "robot_arm_ik.py").exists():
            raise FileNotFoundError(f"xr_teleoperate not found at {root}")
        os.chdir(teleop)
        sys.path.insert(0, str(root))
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
        from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
        self.solver = G1_29_ArmIK(Unit_Test=False, Visualization=False)
        self.hand_retargeting = HandRetargeting(HandType.UNITREE_DEX3)
        # Unitree's stock alpha=0.2 is intentionally conservative for hardware. This backend
        # drives only the Unity simulation, so use a quicker filter without removing smoothing.
        for retargeter in (self.hand_retargeting.left_retargeting,
                           self.hand_retargeting.right_retargeting):
            if retargeter.filter is not None:
                retargeter.filter.alpha = 0.55
            # Keypoints already use Unitree TeleVuer's physical-scale hand convention.
            retargeter.optimizer.scaling = 1.0
        import numpy as np
        self.thumb_neutral_offsets = np.zeros(14, dtype=float)
        self.hand_calibration_delay_frames = 0
        self.hand_calibration_samples = []
        self.hand_neutral_calibrated = False

    @property
    def hand_status(self):
        return ("hand neutral calibrated" if self.hand_neutral_calibrated
                else "calibrating open-hand neutral")

    def solve(self, packet):
        import numpy as np
        current = np.asarray(packet["current"], dtype=float)
        if packet.get("resetSolver"):
            self.solver.init_data = current.copy()
            self.solver.smooth_filter._data_queue.clear()
            self.solver.smooth_filter._filtered_data = current.copy()
        joints, _ = self.solver.solve_ik(
            pose_matrix(packet["left"]), pose_matrix(packet["right"]), current, np.zeros(14))
        return joints.tolist()

    def retarget_hands(self, packet):
        import numpy as np
        left = np.asarray(packet["leftHandKeypoints"], dtype=float)
        right = np.asarray(packet["rightHandKeypoints"], dtype=float)
        if left.size != 75 or right.size != 75:
            raise ValueError("Quest hand retargeting requires 25 xyz keypoints per hand")
        left = left.reshape(25, 3)
        right = right.reshape(25, 3)
        retarget = self.hand_retargeting
        left_vectors = left[retarget.left_indices[1, :]] - left[retarget.left_indices[0, :]]
        right_vectors = right[retarget.right_indices[1, :]] - right[retarget.right_indices[0, :]]
        left_joints = retarget.left_retargeting.retarget(left_vectors)[
            retarget.left_dex_retargeting_to_hardware]
        right_joints = retarget.right_retargeting.retarget(right_vectors)[
            retarget.right_dex_retargeting_to_hardware]
        joints = np.concatenate((left_joints, right_joints))

        # Starting this mode uses a palm-menu ring pinch. Wait for that gesture to be
        # released, then average several relaxed open-hand frames before defining neutral.
        if self.hand_calibration_delay_frames > 0:
            self.hand_calibration_delay_frames -= 1
        elif not self.hand_neutral_calibrated:
            self.hand_calibration_samples.append(joints.copy())
            if len(self.hand_calibration_samples) >= 10:
                measured = np.median(np.asarray(self.hand_calibration_samples), axis=0)
                desired = measured.copy()
                desired[1], desired[2] = np.deg2rad(-18.0), 0.0
                desired[8], desired[9] = np.deg2rad(18.0), 0.0
                self.thumb_neutral_offsets[[1, 2, 8, 9]] = (
                    measured[[1, 2, 8, 9]] - desired[[1, 2, 8, 9]])
                self.hand_neutral_calibrated = True

        if self.hand_neutral_calibrated:
            joints = joints - self.thumb_neutral_offsets
        else:
            # Keep the thumb visibly open while the calibration sample is collected.
            joints[1], joints[2] = np.deg2rad(-18.0), 0.0
            joints[8], joints[9] = np.deg2rad(18.0), 0.0

        # Preserve the physical Dex3 limits after applying the user's neutral offset.
        joints[1] = np.clip(joints[1], -0.72431163, 0.920)
        joints[2] = np.clip(joints[2], 0.0, 1.74532925)
        joints[8] = np.clip(joints[8], -0.920, 0.72431163)
        joints[9] = np.clip(joints[9], -1.74532925, 0.0)
        return joints.tolist()

    def reset_hands(self, calibrate=False):
        for retargeter in (self.hand_retargeting.left_retargeting,
                           self.hand_retargeting.right_retargeting):
            retargeter.reset()
            if retargeter.filter is not None:
                retargeter.filter.reset()
        if calibrate:
            import numpy as np
            self.thumb_neutral_offsets = np.zeros(14, dtype=float)
            self.hand_calibration_delay_frames = 20
            self.hand_calibration_samples = []
            self.hand_neutral_calibrated = False


class ConnectionTestSolver:
    hand_status = "connection test"
    def solve(self, packet):
        return packet["current"]

    def retarget_hands(self, packet):
        return packet.get("handRadians")

    def reset_hands(self, calibrate=False):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7447)
    parser.add_argument("--unitree-repo", help="path to unitreerobotics/xr_teleoperate")
    parser.add_argument("--connection-test", action="store_true",
                        help="echo current arm angles; verifies networking without solving IK")
    args = parser.parse_args()
    if not args.connection_test and not args.unitree_repo:
        parser.error("provide --unitree-repo or use --connection-test")

    solver = ConnectionTestSolver() if args.connection_test else OfficialUnitreeSolver(args.unitree_repo)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    print(f"G1 simulation backend listening on UDP {args.bind}:{args.port}", flush=True)
    chunked_messages = {}
    hand_retargeting_active = False

    while True:
        data, source = sock.recvfrom(65535)
        if data.startswith(b"G1CH") and len(data) >= 12:
            _, message_id, chunk_index, chunk_count = struct.unpack("!4sIHH", data[:12])
            if chunk_count == 0 or chunk_count > 64 or chunk_index >= chunk_count:
                continue
            key = (source, message_id)
            entry = chunked_messages.setdefault(key, [None] * chunk_count)
            if len(entry) != chunk_count:
                del chunked_messages[key]
                continue
            entry[chunk_index] = data[12:]
            if any(chunk is None for chunk in entry):
                if len(chunked_messages) > 64:
                    chunked_messages.clear()
                continue
            data = b"".join(entry)
            del chunked_messages[key]
        try:
            packet = json.loads(data)
            if packet.get("version") != 1 or packet.get("type") != "pose":
                continue
            if packet.get("simulationOnly") is not True:
                raise ValueError("bridge refuses packets not marked simulationOnly")
            started = time.perf_counter()
            joints = solver.solve(packet)
            wants_hand_retargeting = bool(packet.get("retargetHands"))
            if wants_hand_retargeting and (not hand_retargeting_active or packet.get("resetSolver")):
                solver.reset_hands(calibrate=bool(packet.get("resetSolver")))
            hand_retargeting_active = wants_hand_retargeting
            hand_joints = (solver.retarget_hands(packet) if wants_hand_retargeting
                           else packet.get("handRadians"))
            # Unity JsonUtility serializes a null float array as [] on this Android build.
            # An empty array means this mode controls fingers locally, not malformed Dex3 data.
            if hand_joints == []:
                hand_joints = None
            if hand_joints is not None and len(hand_joints) != 14:
                raise ValueError("handRadians must contain 14 Dex3 joint values")
            status = f"solved in {(time.perf_counter() - started) * 1000:.1f} ms"
            if wants_hand_retargeting:
                status += f"; {solver.hand_status}"
            response = {
                "version": 1, "type": "joints", "sequence": packet.get("sequence", 0),
                "valid": len(joints) == 14, "armRadians": joints,
                "handRadians": hand_joints,
                "status": status
            }
            sock.sendto(json.dumps(response, separators=(",", ":")).encode(),
                        (source[0], int(packet.get("replyPort", 7448))))
        except Exception as error:
            response = {"version": 1, "type": "joints", "sequence": 0,
                        "valid": False, "armRadians": [], "handRadians": None,
                        "status": str(error)}
            sock.sendto(json.dumps(response).encode(), (source[0], 7448))
            print(f"Rejected packet from {source[0]}: {error}", flush=True)


if __name__ == "__main__":
    main()
