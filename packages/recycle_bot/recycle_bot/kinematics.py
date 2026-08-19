#!/usr/bin/env python3
"""Forward kinematics straight from the per-arm URDF.

Exists because the controller and ROS disagree about where "the tool" is.

The real-time interface reports the pose of whatever TCP the PolyScope
installation defines. When that TCP is zero — the common case, and the case on
this cell's UR3e — the controller reports the FLANGE. The URDF, meanwhile, puts
tool0 150 mm further out for the Robotiq E-Pick, and that is the frame MoveIt
plans to and the frame sorting_sequence.yaml poses are interpreted in.

Measuring a pose off the pendant and writing it into sorting_sequence.yaml
therefore lands every motion 150 mm short, with nothing to indicate anything is
wrong. Computing tool0 from the joint angles avoids that: joint angles are
unambiguous, and the URDF is the same description MoveIt uses.

No ROS runtime needed — parses the URDF XML and composes the transforms.
"""

import math
import os
import xml.etree.ElementTree as ET

IDENTITY = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def _matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
        for i in range(4)
    ]


def _origin_transform(xyz, rpy):
    """URDF <origin> -> 4x4. Rotation is R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return [
        rot[0] + [xyz[0]],
        rot[1] + [xyz[1]],
        rot[2] + [xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _axis_transform(axis, angle):
    """Rotation of `angle` about `axis` (Rodrigues) as a 4x4."""
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    rot = [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
    ]
    return [
        rot[0] + [0.0],
        rot[1] + [0.0],
        rot[2] + [0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def load_joints(urdf_path):
    """Every joint in a URDF, keyed by name."""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            if origin.get("xyz"):
                xyz = [float(v) for v in origin.get("xyz").split()]
            if origin.get("rpy"):
                rpy = [float(v) for v in origin.get("rpy").split()]
        axis_el = joint.find("axis")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": (
                [float(v) for v in axis_el.get("xyz").split()]
                if axis_el is not None
                else None
            ),
        }
    return joints


def urdf_path_for(ur_type):
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory("recycle_bot_moveit_config"),
        "config",
        ur_type,
        f"{ur_type}.urdf.xacro",
    )


def matrix_to_quaternion(m):
    """Rotation part of a 4x4 (or a 3x3) -> quaternion [x, y, z, w]."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (m[2][1] - m[1][2]) / s
        y, z = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w, x = (m[2][1] - m[1][2]) / s, 0.25 * s
        y, z = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w, x = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        y, z = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w, x = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        y, z = (m[1][2] + m[2][1]) / s, 0.25 * s
    return [x, y, z, w]


# base_link -> base is a rotation of pi about Z (see the URDF's
# base_link-base_fixed_joint). Applied on the left, it re-expresses a pose given
# in base_link into the UR "base" frame the teach pendant and
# sorting_sequence.yaml both use.
_RZ_PI = [
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def to_ur_base_frame(transform):
    """Re-express a base_link transform in the UR "base" frame."""
    return _matmul(_RZ_PI, transform)


def quaternion_angle_between(q1, q2):
    """Smallest rotation angle between two quaternions, in radians.

    Used to check a taught orientation against a canonical one. Quaternions
    double-cover rotations, so q and -q are the same orientation; abs() on the
    dot product handles that.
    """
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def link_transforms(joint_angles, ur_type=None, urdf_path=None, tip="tool0",
                    base="base_link"):
    """Full 4x4 transform of every link from `base` to `tip`.

    link_poses() gives only positions; orientation matters when teaching poses,
    because tool0 is rotated relative to the flange by the URDF's flange-tool0
    joint. Reading the controller's reported orientation would give the FLANGE's,
    which is not what a sorting_sequence.yaml pose means.
    """
    if urdf_path is None:
        if ur_type is None:
            raise ValueError("pass ur_type or urdf_path")
        urdf_path = urdf_path_for(ur_type)

    joints = load_joints(urdf_path)
    by_child = {j["child"]: (name, j) for name, j in joints.items()}

    chain = []
    link = tip
    while link != base:
        if link not in by_child:
            raise ValueError(f"no path from {base!r} to {tip!r}: stuck at {link!r}")
        name, joint = by_child[link]
        chain.append((name, joint))
        link = joint["parent"]
    chain.reverse()

    transform = [row[:] for row in IDENTITY]
    transforms = {}
    for name, joint in chain:
        transform = _matmul(transform, _origin_transform(joint["xyz"], joint["rpy"]))
        if joint["type"] in ("revolute", "continuous") and joint["axis"]:
            angle = joint_angles.get(name)
            if angle is None:
                raise ValueError(f"no angle supplied for joint {name!r}")
            transform = _matmul(transform, _axis_transform(joint["axis"], angle))
        transforms[joint["child"]] = [row[:] for row in transform]
    return transforms


def link_poses(joint_angles, ur_type=None, urdf_path=None, tip="tool0",
               base="base_link"):
    """Position of every link from `base` to `tip`, in `base` coordinates.

    joint_angles: {joint_name: radians}. Returns {link_name: (x, y, z)}.

    Uses nominal kinematics from the URDF, not the robot's own calibration, so
    results sit a couple of millimetres from the controller's own numbers. That
    is far below the 150 mm question this is here to answer, but it does mean
    this is not a substitute for the driver's calibrated FK when millimetres
    matter.
    """
    transforms = link_transforms(
        joint_angles, ur_type=ur_type, urdf_path=urdf_path, tip=tip, base=base
    )
    return {
        link: (t[0][3], t[1][3], t[2][3]) for link, t in transforms.items()
    }
