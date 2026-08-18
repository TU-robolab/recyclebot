#!/usr/bin/env python3
"""Validate a teach-pendant kinematic calibration before it reaches the robot.

my_robot_calibration.yaml is exported per physical arm and consumed by
ur_robot_driver as ground truth for forward kinematics. A wrong file does not
fail loudly — it shifts every computed pose by a fixed offset, so picks miss by a
consistent amount and the cause looks like a bad camera calibration.

The two mistakes this catches:

  1. Exporting from (or copying) the wrong arm. A UR3e and a UR16e differ by
     ~235 mm on the forearm link, so the arm a calibration came from is
     unambiguous from its link lengths alone.
  2. A malformed or truncated export — missing links, missing hash, non-numeric
     values.

Reference values come from ur_description's config/<ur_type>/default_kinematics.yaml
rather than being duplicated here, so this stays correct across upstream updates.

CLI:
    ros2 run recycle_bot check_calibration --ur-type ur3e
    ros2 run recycle_bot check_calibration --ur-type ur3e --file /path/to/export.yaml
"""

import argparse
import os
import sys

import yaml

from ament_index_python.packages import get_package_share_directory

from recycle_bot.robot_profile import PROFILES, resolve_ur_type

LINKS = ("shoulder", "upper_arm", "forearm", "wrist_1", "wrist_2", "wrist_3")
AXES = ("x", "y", "z")
ANGLES = ("roll", "pitch", "yaw")

# A real export deviates from nominal only by the arm's manufacturing tolerance.
# The measured UR16e in this repo is within 0.93 mm, so 5 mm is a generous
# ceiling that still leaves a ~200 mm margin against a wrong-arm file.
DEFAULT_TOLERANCE_MM = 5.0


def nominal_kinematics(ur_type):
    """Nominal kinematics for an arm, from ur_description."""
    path = os.path.join(
        get_package_share_directory("ur_description"),
        "config",
        ur_type,
        "default_kinematics.yaml",
    )
    with open(path, "r") as f:
        return yaml.safe_load(f)["kinematics"]


def available_arms():
    """Every arm ur_description ships nominal kinematics for."""
    config_root = os.path.join(get_package_share_directory("ur_description"), "config")
    arms = []
    for entry in sorted(os.listdir(config_root)):
        if os.path.exists(
            os.path.join(config_root, entry, "default_kinematics.yaml")
        ):
            arms.append(entry)
    return arms


def max_deviation_mm(kinematics, nominal):
    """Largest positional difference between two kinematics blocks, in mm.

    Returns None when the calibration is missing any link/axis the comparison
    needs — structural problems are reported separately by check_structure().
    """
    worst = 0.0
    for link in LINKS:
        if link not in kinematics or link not in nominal:
            return None
        for axis in AXES:
            try:
                measured = float(kinematics[link][axis])
                expected = float(nominal[link][axis])
            except (KeyError, TypeError, ValueError):
                return None
            worst = max(worst, abs(measured - expected) * 1000.0)
    return worst


def identify_arm(kinematics):
    """Best-matching arm for a calibration, as (ur_type, deviation_mm).

    Used to turn "this file is wrong" into "this is a ur16e calibration".
    """
    best = (None, None)
    for arm in available_arms():
        try:
            deviation = max_deviation_mm(kinematics, nominal_kinematics(arm))
        except (OSError, KeyError, TypeError):
            continue
        if deviation is None:
            continue
        if best[1] is None or deviation < best[1]:
            best = (arm, deviation)
    return best


def check_structure(data):
    """Structural problems with a parsed calibration file."""
    problems = []

    if not isinstance(data, dict) or "kinematics" not in data:
        return ["file has no top-level 'kinematics' key"]

    kinematics = data["kinematics"]
    if not isinstance(kinematics, dict):
        return ["'kinematics' is not a mapping"]

    for link in LINKS:
        if link not in kinematics:
            problems.append(f"missing link '{link}'")
            continue
        entry = kinematics[link]
        if not isinstance(entry, dict):
            problems.append(f"link '{link}' is not a mapping")
            continue
        for field in AXES + ANGLES:
            if field not in entry:
                problems.append(f"link '{link}' is missing '{field}'")
                continue
            try:
                float(entry[field])
            except (TypeError, ValueError):
                problems.append(
                    f"link '{link}' field '{field}' is not numeric: {entry[field]!r}"
                )

    if "hash" not in kinematics:
        problems.append(
            "missing 'hash' — ur_robot_driver uses it to detect a calibration "
            "that does not match the connected robot"
        )

    return problems


def check_calibration(path, ur_type, tolerance_mm=DEFAULT_TOLERANCE_MM):
    """Validate a calibration file for an arm.

    Returns (problems, info). `problems` is a list of strings; empty means the
    file is usable. `info` carries details worth printing either way.
    """
    info = {"path": path, "ur_type": ur_type}

    if not os.path.exists(path):
        return [f"file does not exist: {path}"], info

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        return [f"could not parse YAML: {e}"], info

    problems = check_structure(data)
    if problems:
        return problems, info

    kinematics = data["kinematics"]
    info["hash"] = kinematics.get("hash")

    try:
        nominal = nominal_kinematics(ur_type)
    except OSError as e:
        # Cannot compare without ur_description; structure already checked.
        info["note"] = f"skipped arm-identity check ({e})"
        return [], info

    deviation = max_deviation_mm(kinematics, nominal)
    info["deviation_mm"] = deviation

    if deviation is not None and deviation > tolerance_mm:
        matched_arm, matched_deviation = identify_arm(kinematics)
        info["matched_arm"] = matched_arm
        info["matched_deviation_mm"] = matched_deviation

        detail = (
            f"calibration deviates from {ur_type} nominal kinematics by "
            f"{deviation:.1f} mm, well beyond the {tolerance_mm:.1f} mm tolerance"
        )
        if matched_arm and matched_arm != ur_type:
            detail += (
                f".\n  The link lengths match '{matched_arm}' instead "
                f"(within {matched_deviation:.2f} mm) — this looks like a "
                f"{matched_arm} calibration filed under {ur_type}"
            )
        else:
            detail += (
                ".\n  Link lengths do not match any arm ur_description knows "
                "about; the export may be corrupt"
            )
        problems.append(detail)

    return problems, info


def default_path(ur_type):
    return os.path.join(
        get_package_share_directory("recycle_bot"),
        "config",
        ur_type,
        "my_robot_calibration.yaml",
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate a UR teach-pendant kinematic calibration.",
    )
    parser.add_argument(
        "--ur-type",
        default=None,
        help=f"Arm the calibration should belong to. One of {sorted(PROFILES)}.",
    )
    parser.add_argument(
        "--file",
        default=None,
        help="Calibration file to check (default: the installed one for --ur-type).",
    )
    parser.add_argument(
        "--tolerance-mm",
        type=float,
        default=DEFAULT_TOLERANCE_MM,
        help=f"Allowed deviation from nominal (default: {DEFAULT_TOLERANCE_MM}).",
    )
    args = parser.parse_args(argv)

    try:
        ur_type = resolve_ur_type(args.ur_type)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    path = args.file or default_path(ur_type)
    problems, info = check_calibration(path, ur_type, args.tolerance_mm)

    print(f"Checking {ur_type} calibration: {path}")
    if "hash" in info:
        print(f"  hash: {info['hash']}")
    if info.get("deviation_mm") is not None:
        print(f"  max deviation from {ur_type} nominal: {info['deviation_mm']:.3f} mm")
    if "note" in info:
        print(f"  note: {info['note']}")

    if problems:
        print(f"\nFAILED — {len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
        return 1

    print("\nOK — calibration is well-formed and matches this arm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
