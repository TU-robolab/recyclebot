#!/usr/bin/env python3
"""Confirm the robot on the wire is the one the configuration describes.

The failure this prevents: `ur_type` selects the URDF, joint limits, reach
envelope and cell geometry, while `robot_ip` decides which controller receives
the resulting trajectories. Nothing structurally ties the two together, so a
stale REMOTE_IP or a swapped arm silently sends UR16e-scale motion to a UR3e —
a 900 mm arm's trajectories on a 500 mm one. The driver accepts it; the arm hits
its own joint limits or the table.

The check is a single read-only query to the UR dashboard server (port 29999),
which every UR controller runs. It needs no program running on the robot and no
External Control, so it is cheap enough to do on every hardware launch.

Resolution granularity: "get robot model" returns the product FAMILY — a UR3e
and a CB3 UR3 both answer "UR3". That separates a UR3e from a UR16e, which is
the confusion that damages hardware. Distinguishing e-Series from CB3 is the
kinematic calibration's job (see calibration_check.py, which resolves them by
link length).

CLI:
    ros2 run recycle_bot check_robot --ur-type ur3e
    ros2 run recycle_bot check_robot --ur-type ur3e --robot-ip 192.168.1.102
"""

import argparse
import socket
import sys

from recycle_bot.robot_profile import PROFILES, resolve_ur_type, robot_ip

DASHBOARD_PORT = 29999
DEFAULT_TIMEOUT_S = 5.0


class RobotModelMismatch(RuntimeError):
    """The connected robot is a different arm than the configuration expects."""


def query_robot_model(ip, timeout=DEFAULT_TIMEOUT_S, port=DASHBOARD_PORT):
    """Ask the dashboard server what robot this is.

    Returns the model string (e.g. "UR3", "UR16"). Raises OSError if the
    controller is unreachable, which callers treat as "cannot verify" rather
    than "wrong robot" — an unreachable arm fails clearly at the driver anyway.
    """
    sock = socket.create_connection((ip, port), timeout=timeout)
    try:
        sock.settimeout(timeout)
        sock.recv(4096)  # greeting banner
        sock.sendall(b"get robot model\n")
        reply = sock.recv(4096).decode(errors="replace").strip()
    finally:
        sock.close()

    if not reply:
        raise OSError(f"dashboard server at {ip}:{port} returned an empty model")
    return reply


def model_to_ur_types(model):
    """Every configured arm whose family matches this dashboard model string."""
    normalized = model.strip().upper()
    return sorted(
        ur_type
        for ur_type, prof in PROFILES.items()
        if prof.dashboard_model.upper() == normalized
    )


def verify_robot_model(ur_type, ip=None, timeout=DEFAULT_TIMEOUT_S, strict=True):
    """Check the connected robot against the expected arm.

    Returns (ok, message). `ok` is False only for a definite mismatch. An
    unreachable controller returns True with a warning message: refusing to
    launch because a network check timed out would be worse than letting the
    driver report the connection failure itself.

    Raises RobotModelMismatch on a definite mismatch when strict is True.
    """
    ur_type = resolve_ur_type(ur_type)
    ip = ip or robot_ip(ur_type)
    expected = PROFILES[ur_type].dashboard_model

    try:
        model = query_robot_model(ip, timeout=timeout)
    except OSError as e:
        return True, (
            f"could not reach the dashboard server at {ip}:{DASHBOARD_PORT} to "
            f"verify the robot model ({e}). Continuing — the driver will report "
            f"the connection failure if the robot really is unreachable."
        )

    matches = model_to_ur_types(model)

    if ur_type in matches:
        detail = f"robot at {ip} reports model '{model}', matching ur_type '{ur_type}'"
        if len(matches) > 1:
            detail += (
                f" (note: {', '.join(matches)} share this model string; the "
                "kinematic calibration distinguishes them)"
            )
        return True, detail

    if matches:
        message = (
            f"WRONG ROBOT: the arm at {ip} reports model '{model}', which is "
            f"{' or '.join(matches)} — but this launch is configured for "
            f"'{ur_type}' (expects '{expected}').\n"
            f"  Launching would send {ur_type}-scale trajectories to a "
            f"{matches[0]}.\n"
            f"  Either pass ur_type:={matches[0]}, or point this launch at the "
            f"{ur_type} by setting {ur_type.upper()}_ROBOT_IP (or REMOTE_IP)."
        )
    else:
        message = (
            f"WRONG ROBOT: the arm at {ip} reports model '{model}', which does "
            f"not match ur_type '{ur_type}' (expects '{expected}') and is not a "
            f"configured arm. Known: "
            + ", ".join(f"{k}={v.dashboard_model}" for k, v in sorted(PROFILES.items()))
        )

    if strict:
        raise RobotModelMismatch(message)
    return False, message


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Verify the connected UR robot matches the expected arm.",
    )
    parser.add_argument(
        "--ur-type",
        default=None,
        help=f"Expected arm. One of {sorted(PROFILES)}.",
    )
    parser.add_argument(
        "--robot-ip",
        default=None,
        help="Controller address (default: resolved per-arm, see robot_profile).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"Dashboard query timeout in seconds (default: {DEFAULT_TIMEOUT_S}).",
    )
    args = parser.parse_args(argv)

    try:
        ur_type = resolve_ur_type(args.ur_type)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    ip = args.robot_ip or robot_ip(ur_type)
    print(f"Verifying {ur_type} at {ip}:{DASHBOARD_PORT}")

    ok, message = verify_robot_model(
        ur_type, ip=ip, timeout=args.timeout, strict=False
    )
    print(f"  {message}")
    if not ok:
        print("\nFAILED")
        return 1
    print("\nOK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
