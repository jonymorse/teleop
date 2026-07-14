#!/usr/bin/env python3
"""Relay Quest UDP discovery between the Windows LAN and a WSL localhost backend."""

import argparse
from pathlib import Path
import socket
import struct
import threading


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quest-port", type=int, default=7447)
    parser.add_argument("--reply-port", type=int, default=7448)
    parser.add_argument("--wsl-port", type=int, default=7547)
    args = parser.parse_args()
    log_dir = Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(exist_ok=True)

    poses = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    poses.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    poses.bind(("0.0.0.0", args.quest_port))
    replies = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    replies.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    replies.bind(("0.0.0.0", args.reply_port))
    backend = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Windows reports an ICMP "port unreachable" as WinError 10054 on the next recvfrom.
    # That is expected if Quest discovery arrives while the WSL solver is still starting;
    # it must not terminate the long-running relay.
    if hasattr(socket, "SIO_UDP_CONNRESET"):
        poses.ioctl(socket.SIO_UDP_CONNRESET, False)
        replies.ioctl(socket.SIO_UDP_CONNRESET, False)
        backend.ioctl(socket.SIO_UDP_CONNRESET, False)
    latest_quest = [None]
    pose_count = [0]
    reply_count = [0]
    message_id = [0]

    def forward_replies():
        while True:
            try:
                data, _ = replies.recvfrom(65535)
            except ConnectionResetError:
                continue
            reply_count[0] += 1
            destination = latest_quest[0]
            if destination:
                if reply_count[0] == 1 or reply_count[0] % 300 == 0:
                    print(f"Backend replies: {reply_count[0]} -> Quest {destination[0]}:{args.reply_port}",
                          flush=True)
                replies.sendto(data, (destination[0], args.reply_port))
                if reply_count[0] == 1 or reply_count[0] % 60 == 0:
                    (log_dir / "latest_reply.json").write_bytes(data)

    threading.Thread(target=forward_replies, daemon=True).start()
    print(f"Quest relay UDP {args.quest_port}/{args.reply_port} -> WSL localhost:{args.wsl_port}", flush=True)
    while True:
        try:
            data, source = poses.recvfrom(65535)
        except ConnectionResetError:
            continue
        pose_count[0] += 1
        if pose_count[0] == 1 or pose_count[0] % 300 == 0:
            print(f"Quest poses: {pose_count[0]} from {source[0]}:{source[1]} ({len(data)} bytes)",
                  flush=True)
        latest_quest[0] = source
        if pose_count[0] == 1 or pose_count[0] % 60 == 0:
            (log_dir / "latest_pose.json").write_bytes(data)
        # A separate ephemeral socket is required here. Reusing the LAN socket bound to
        # 0.0.0.0:7447 can be swallowed by Windows/WSL localhost forwarding.
        message_id[0] = (message_id[0] + 1) & 0xFFFFFFFF
        chunk_size = 1200
        chunk_count = (len(data) + chunk_size - 1) // chunk_size
        for index in range(chunk_count):
            payload = data[index * chunk_size:(index + 1) * chunk_size]
            header = struct.pack("!4sIHH", b"G1CH", message_id[0], index, chunk_count)
            backend.sendto(header + payload, ("127.0.0.1", args.wsl_port))


if __name__ == "__main__":
    main()
