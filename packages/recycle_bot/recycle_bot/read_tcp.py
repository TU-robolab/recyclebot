#!/usr/bin/env python3
"""Read the live TCP pose and joint angles straight off a UR controller.

Used for measuring a cell: jog the arm to a physical feature, read where it is,
and write the number into config. Needed for the camera transform in
calibration.yaml, the collision geometry in cell.yaml, and the taught poses in
sorting_sequence.yaml.

Reads the real-time client interface (port 30003), which streams robot state at
125 Hz on CB3 and 500 Hz on e-Series. Read-only: it opens a socket, takes some
samples and closes. No program need be running on the robot, External Control is
not required, and nothing is sent to the controller.

TWO THINGS THAT SILENTLY CORRUPT A MEASUREMENT:

1. Frames. The controller reports poses in the UR "base" frame, which is what
   the teach pendant shows. ROS uses "base_link", rotated 180 degrees about Z,
   so x and y flip sign. sorting_sequence.yaml is in "base"; the camera
   transform in calibration.yaml is in "base_link". Both are printed below so
   the right one can be copied without doing the negation by hand.

2. TCP offset. The reported pose is of the TCP configured in the PolyScope
   installation, NOT the flange. If the installation defines the vacuum cup as
   the TCP, this reports the cup tip; if the TCP is zero, it reports the flange.
   The URDF places tool0 150 mm past the flange for the E-Pick, so the two agree
   only when the pendant TCP matches. --tcp-offset reports what the controller
   currently has configured.

CLI:
    ros2 run recycle_bot read_tcp
    ros2 run recycle_bot read_tcp --samples 50 --label "under camera"
    ros2 run recycle_bot read_tcp --json
"""

import argparse
import json
import math
import socket
import struct
import sys

RT_PORT = 30003

# Byte offsets into the real-time packet (UR "Client Interfaces" spec).
OFF_TIME = 4
OFF_Q_TARGET = 12
OFF_Q_ACTUAL = 252
OFF_TCP_ACTUAL = 444
OFF_TCP_TARGET = 588


def read_packet(ip, timeout=5.0, port=RT_PORT):
    """One complete real-time packet."""
    sock = socket.create_connection((ip, port), timeout=timeout)
    try:
        sock.settimeout(timeout)
        buf = b""
        while len(buf) < 4:
            chunk = sock.recv(4 - len(buf))
            if not chunk:
                raise OSError("connection closed while reading packet length")
            buf += chunk
        size = struct.unpack(">i", buf[:4])[0]
        if not (100 < size < 4096):
            raise OSError(f"implausible packet length {size}; is this port {port}?")
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                raise OSError("connection closed mid-packet")
            buf += chunk
        return buf
    finally:
        sock.close()


def _doubles(buf, offset, count=6):
    return list(struct.unpack(f">{count}d", buf[offset:offset + 8 * count]))


def sample(ip, samples=10, timeout=5.0, port=RT_PORT):
    """Average several packets; also report how much the readings moved.

    Averaging suppresses encoder jitter. The spread is the honest error bar: if
    it is large the arm was still moving and the measurement should be retaken.
    """
    poses, joints = [], []
    for _ in range(max(1, samples)):
        buf = read_packet(ip, timeout=timeout, port=port)
        poses.append(_doubles(buf, OFF_TCP_ACTUAL))
        joints.append(_doubles(buf, OFF_Q_ACTUAL))

    n = len(poses)
    mean_pose = [sum(p[i] for p in poses) / n for i in range(6)]
    mean_joints = [sum(j[i] for j in joints) / n for i in range(6)]
    spread_mm = max(
        max(p[i] for p in poses) - min(p[i] for p in poses) for i in range(3)
    ) * 1000.0
    return mean_pose, mean_joints, spread_mm, n


def axis_angle_to_matrix(rx, ry, rz):
    """Rodrigues rotation vector -> 3x3 rotation matrix."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    c, s, v = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
    return [
        [kx * kx * v + c,      kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c,      ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ]


def matrix_to_quaternion(m):
    """Rotation matrix -> quaternion [x, y, z, w]."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def matrix_to_rpy(m):
    """Rotation matrix -> (roll, pitch, yaw), R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = math.atan2(-m[2][0], math.sqrt(m[2][1] ** 2 + m[2][2] ** 2))
    if abs(math.cos(pitch)) < 1e-9:  # gimbal lock
        return math.atan2(-m[1][2], m[1][1]), pitch, 0.0
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def base_to_base_link(x, y, z):
    """UR "base" coordinates -> ROS "base_link".

    base_link -> base is a rotation of pi about Z, so the mapping is its own
    inverse: negate x and y, leave z.
    """
    return -x, -y, z


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def describe(pose, joints, spread_mm, count, ur_type=None, label=None):
    """Human-readable report; returns the same data as a dict."""
    x, y, z, rx, ry, rz = pose
    matrix = axis_angle_to_matrix(rx, ry, rz)
    quat = matrix_to_quaternion(matrix)
    roll, pitch, yaw = matrix_to_rpy(matrix)
    bl = base_to_base_link(x, y, z)
    reach = math.sqrt(x * x + y * y + z * z)

    lines = []
    if label:
        lines.append(f"  label:           {label}")
    lines.append(f"  samples:         {count}  (spread {spread_mm:.2f} mm)")
    if spread_mm > 1.0:
        lines.append(
            "  WARNING:         readings moved more than 1 mm — the arm may "
            "still be settling; retake before writing this into config"
        )
    lines.append("")
    lines.append("  TCP in UR 'base' frame  (what the teach pendant shows;")
    lines.append("                           use for sorting_sequence.yaml)")
    lines.append(f"    position:      [{x:.5f}, {y:.5f}, {z:.5f}]")
    lines.append("")
    lines.append("  TCP in ROS 'base_link' frame  (x/y negated;")
    lines.append("                                 use for calibration.yaml + cell.yaml)")
    lines.append(f"    position:      [{bl[0]:.5f}, {bl[1]:.5f}, {bl[2]:.5f}]")
    lines.append("")
    lines.append("  orientation")
    lines.append(f"    axis-angle:    [{rx:.5f}, {ry:.5f}, {rz:.5f}]   (as the pendant reports)")
    lines.append(f"    quaternion:    [{quat[0]:.5f}, {quat[1]:.5f}, {quat[2]:.5f}, {quat[3]:.5f}]   (x, y, z, w)")
    lines.append(
        f"    rpy (deg):     [{math.degrees(roll):.2f}, "
        f"{math.degrees(pitch):.2f}, {math.degrees(yaw):.2f}]"
    )
    lines.append("")
    lines.append(f"  distance from base origin: {reach:.4f} m")
    if ur_type:
        try:
            from recycle_bot.robot_profile import PROFILES

            prof = PROFILES.get(ur_type)
            if prof:
                lines.append(
                    f"    {ur_type} planning envelope: {prof.planning_reach_m:.3f} m "
                    f"({'inside' if reach <= prof.planning_reach_m else 'OUTSIDE'})"
                )
                lines.append(
                    "    note: this is the TCP, which sits past the flange by the "
                    "configured tool offset, so exceeding the envelope here does "
                    "not by itself mean the pose is unreachable"
                )
        except ImportError:
            pass
    # The controller reports its configured TCP, which on this cell is zero —
    # i.e. the flange. tool0 (flange + the E-Pick's 150 mm) is what MoveIt plans
    # to and what sorting_sequence.yaml poses mean, so compute it from the joint
    # angles rather than leaving a 150 mm trap in every measurement.
    if ur_type:
        try:
            from recycle_bot.kinematics import link_poses

            frames = link_poses(dict(zip(JOINT_NAMES, joints)), ur_type=ur_type)
            flange = frames.get("flange")
            tool0 = frames.get("tool0")
            if flange and tool0:
                residual_mm = math.dist(flange, bl) * 1000.0
                offset_mm = math.dist(flange, tool0) * 1000.0
                lines.append("")
                lines.append("  FORWARD KINEMATICS from the joint angles (base_link frame)")
                lines.append(
                    f"    flange:        [{flange[0]:.5f}, {flange[1]:.5f}, {flange[2]:.5f}]"
                )
                lines.append(
                    f"    tool0:         [{tool0[0]:.5f}, {tool0[1]:.5f}, {tool0[2]:.5f}]"
                    "   <- gripper tip; use for sorting_sequence.yaml"
                )
                tb = base_to_base_link(*tool0)  # involution: back to 'base'
                lines.append(
                    f"    tool0 in 'base': [{tb[0]:.5f}, {tb[1]:.5f}, {tb[2]:.5f}]"
                )
                lines.append("")
                if residual_mm < 15.0:
                    lines.append(
                        f"    The controller's reported pose matches the FLANGE "
                        f"(within {residual_mm:.1f} mm), so the PolyScope TCP is "
                        f"zero. tool0 is {offset_mm:.0f} mm further out."
                    )
                else:
                    tool_residual_mm = math.dist(tool0, bl) * 1000.0
                    if tool_residual_mm < 15.0:
                        lines.append(
                            f"    The controller's reported pose matches TOOL0 "
                            f"(within {tool_residual_mm:.1f} mm), so the PolyScope "
                            f"TCP already includes the E-Pick offset."
                        )
                    else:
                        lines.append(
                            f"    WARNING: reported pose matches neither flange "
                            f"({residual_mm:.0f} mm off) nor tool0 "
                            f"({tool_residual_mm:.0f} mm off). The PolyScope TCP "
                            f"is set to something else — identify it before "
                            f"trusting any measurement."
                        )
        except Exception as e:
            lines.append("")
            lines.append(f"  (forward kinematics unavailable: {e})")

    lines.append("")
    lines.append("  joint angles (rad) — copy into a *_joint_pose block")
    for name, value in zip(JOINT_NAMES, joints):
        lines.append(f"    {name + ':':<22}{value:+.5f}")

    return "\n".join(lines), {
        "label": label,
        "samples": count,
        "spread_mm": spread_mm,
        "base": {"position": [x, y, z], "axis_angle": [rx, ry, rz]},
        "base_link": {"position": list(bl)},
        "quaternion_xyzw": quat,
        "rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
        "distance_from_base_m": reach,
        "joints": dict(zip(JOINT_NAMES, joints)),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read live TCP pose and joint angles from a UR controller.",
    )
    parser.add_argument("--ip", default=None, help="Controller address.")
    parser.add_argument("--ur-type", default=None, help="Arm, for IP + reach context.")
    parser.add_argument("--samples", type=int, default=10, help="Packets to average.")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--label", default=None, help="What the TCP is touching.")
    parser.add_argument("--json", action="store_true", help="Machine-readable output.")
    args = parser.parse_args(argv)

    ur_type = None
    ip = args.ip
    if ip is None or args.ur_type:
        try:
            from recycle_bot.robot_profile import resolve_ur_type, robot_ip

            ur_type = resolve_ur_type(args.ur_type)
            ip = ip or robot_ip(ur_type)
        except Exception as e:  # standalone use without the package installed
            if ip is None:
                print(f"ERROR: no --ip given and could not resolve one ({e})",
                      file=sys.stderr)
                return 2

    try:
        pose, joints, spread, count = sample(
            ip, samples=args.samples, timeout=args.timeout
        )
    except OSError as e:
        print(f"ERROR: could not read from {ip}:{RT_PORT} — {e}", file=sys.stderr)
        return 1

    text, data = describe(pose, joints, spread, count, ur_type, args.label)
    if args.json:
        data["robot_ip"] = ip
        print(json.dumps(data, indent=2))
    else:
        print(f"TCP pose from {ip}:{RT_PORT}")
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
