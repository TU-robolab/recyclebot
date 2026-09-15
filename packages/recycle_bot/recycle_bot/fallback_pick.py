"""Fallback pick: pick whatever sits on one fixed spot, measuring only its height.

For when object recognition lets us down (poor light, an item YOLO has never
seen): the operator puts an item on a spot marked on the table and asks for a
fallback pick. The spot's x/y are configured (fallback_pick in
config/<ur_type>/sorting_sequence.yaml, UR "base" frame); only the height comes
from the depth camera, so the cup still lands on top of whatever is there.

rec_bot_core owns the ROS side (/fallback_pick service, TF lookup, publishing
the pick). Everything here is plain Python + numpy so it is testable without a
ROS graph: test_suite/test/test_fallback_pick.py.
"""

import math
from dataclasses import dataclass

import numpy as np

# Label the fallback pick travels under on /vision/detected_object. It is not a
# YOLO class, so bin_routing has no rule for it and control sends it to
# default_bin — an item nobody recognised is residual waste. The dashboard shows
# it by a friendlier name (dashboard.DISPLAY_NAMES).
FALLBACK_LABEL = "fallback_pick"

# Z16 depth from the RealSense is millimetres; float depth is already metres.
DEPTH_SCALE_BY_ENCODING = {"16UC1": 0.001, "mono16": 0.001, "32FC1": 1.0}


class FallbackPickError(Exception):
    """A refused fallback pick. The message is shown to the operator as is;
    detail, when set, carries the numbers for the log."""

    def __init__(self, message, detail=None):
        super().__init__(message)
        self.detail = detail


@dataclass(frozen=True)
class FallbackPickConfig:
    x: float                 # spot position in the UR "base" frame (meters)
    y: float
    sample_size_m: float     # side of the square the depth median is taken over
    min_surface_z_m: float   # measured surface below this = nothing on the spot


def parse_config(data):
    """The fallback_pick section of a sorting_sequence.yaml dict, or None if absent.

    Raises ValueError for a malformed section rather than guessing.
    """
    section = (data or {}).get("fallback_pick")
    if not section:
        return None
    position = section.get("position")
    if not isinstance(position, (list, tuple)) or len(position) != 2:
        raise ValueError(
            f"fallback_pick.position must be [x, y] in the UR 'base' frame, got "
            f"{position!r} — the height is measured by the camera, not configured"
        )
    return FallbackPickConfig(
        x=float(position[0]),
        y=float(position[1]),
        sample_size_m=float(section.get("sample_size_m", 0.03)),
        min_surface_z_m=float(section.get("min_surface_z_m", 0.0)),
    )


# -----------------------------------------------------------------------------
# geometry
# -----------------------------------------------------------------------------

def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class RigidTransform:
    """p_target = R * p_source + t, with R as a unit quaternion (x, y, z, w).

    Same convention as tf2's lookup_transform(target, source): the transform it
    returns maps points expressed in `source` into `target`.
    """
    translation: tuple
    rotation: tuple

    @classmethod
    def from_msg(cls, transform):
        """From a geometry_msgs/Transform (duck-typed; no ROS import here)."""
        t, q = transform.translation, transform.rotation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        return cls((t.x, t.y, t.z), (q.x / norm, q.y / norm, q.z / norm, q.w / norm))

    def rotate(self, v):
        x, y, z, w = self.rotation
        c1 = _cross((x, y, z), v)
        c2 = _cross((x, y, z), c1)
        return tuple(v[i] + 2.0 * w * c1[i] + 2.0 * c2[i] for i in range(3))

    def apply(self, p):
        r = self.rotate(p)
        return tuple(r[i] + self.translation[i] for i in range(3))

    def inverse(self):
        x, y, z, w = self.rotation
        inv = RigidTransform((0.0, 0.0, 0.0), (-x, -y, -z, w))
        t = inv.rotate(self.translation)
        return RigidTransform((-t[0], -t[1], -t[2]), inv.rotation)


@dataclass(frozen=True)
class Pinhole:
    """The projection rec_bot_core's detection path uses, made invertible.

    image_geometry's projectPixelTo3dRay scaled to Z-depth (process_detection)
    reduces to x = (u - cx) / fx * z on the P matrix; this is that, both ways.
    """
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, info):
        p = list(info.p)
        if p and p[0] > 0.0:
            return cls(p[0], p[5], p[2], p[6], int(info.width), int(info.height))
        k = list(info.k)
        return cls(k[0], k[4], k[2], k[5], int(info.width), int(info.height))

    def project(self, p):
        return (self.fx * p[0] / p[2] + self.cx, self.fy * p[1] / p[2] + self.cy)

    def unproject(self, u, v, z):
        return ((u - self.cx) / self.fx * z, (v - self.cy) / self.fy * z, z)


def sample_depth(depth, u, v, half_px, scale):
    """Median depth (meters) of a square window centred on (u, v), or None.

    Zero (and, for float images, NaN) means "no reading" and is excluded, as in
    rec_bot_vision. None when fewer than a tenth of the window has a reading: a
    median over a handful of stray pixels is not a measurement.
    """
    h, w = depth.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    window = depth[max(0, vi - half_px):min(h, vi + half_px + 1),
                   max(0, ui - half_px):min(w, ui + half_px + 1)]
    if window.size == 0:
        return None
    valid = window[np.isfinite(window) & (window > 0)] if window.dtype.kind == "f" \
        else window[window > 0]
    if valid.size == 0 or valid.size < window.size // 10:
        return None
    return float(np.median(valid)) * scale


@dataclass(frozen=True)
class SurfaceReading:
    z: float          # height of the surface on the spot, UR "base" frame
    depth_m: float    # the Z-depth it was measured at
    pixel: tuple      # (u, v) the depth was read around


def measure_pick_height(cfg, cam_from_base, camera, depth_at,
                        min_depth_m, max_depth_m,
                        scan_top_m=0.30, scan_bottom_m=-0.10, step_m=0.005):
    """Height of whatever sits on the fallback spot, checked for a usable pick.

    cam_from_base  RigidTransform, points in "base" -> the depth image's frame
    camera         Pinhole of the depth image (aligned to colour)
    depth_at       (u, v, half_px) -> median Z-depth in meters, or None

    Which pixel to read depends on the height being measured. Unless the spot
    is right under the lens the camera sees it along a slanted ray, so the top
    of a tall item appears displaced from its foot — read at the foot's pixel,
    a narrow item is missed and the table beside it is measured instead.

    So walk down the vertical line through the spot, from scan_top_m (the
    tallest item considered) to scan_bottom_m (well below any table), and at
    each height z read the depth where (x, y, z) would appear. While z is above
    the item, that ray passes over it and lands on something lower. The first z
    whose ray is stopped at or above z is inside the item, and what stopped the
    ray is its top. An empty spot stops at the table, which min_surface_z_m
    then rejects.

    Raises FallbackPickError, worded for the operator, if the spot is out of
    view, has no depth, holds nothing, or something is too close to the lens.
    """
    base_from_cam = cam_from_base.inverse()
    in_view = has_depth = False
    found = None
    steps = int(round((scan_top_m - scan_bottom_m) / step_m))
    for i in range(steps + 1):
        z = scan_top_m - i * step_m
        p = cam_from_base.apply((cfg.x, cfg.y, z))
        if p[2] <= 0.0:
            continue  # behind the lens at this height
        u, v = camera.project(p)
        if not (0.0 <= u < camera.width and 0.0 <= v < camera.height):
            continue  # a tall item near the image edge can leave the view
        in_view = True
        half_px = max(1, int(round(camera.fx * cfg.sample_size_m / 2.0 / p[2])))
        depth = depth_at(u, v, half_px)
        if depth is None:
            continue
        has_depth = True
        surface_z = base_from_cam.apply(camera.unproject(u, v, depth))[2]
        if surface_z >= z - step_m:
            found = (surface_z, depth, (u, v))
            break

    if not in_view:
        raise FallbackPickError(
            "The marked spot is outside the camera's view. Check "
            "fallback_pick.position (it is in the UR 'base' frame, not base_link).")
    if not has_depth:
        raise FallbackPickError(
            "The camera cannot measure the depth at the marked spot. Shiny or "
            "see-through items often have no depth — try another item.")
    if found is None:
        raise FallbackPickError(
            "The camera sees no surface at the marked spot. Check the camera "
            "calibration.",
            f"nothing between z={scan_top_m} and z={scan_bottom_m} m in base")
    z, depth, (u, v) = found

    detail = (f"surface z={z:.3f} m, depth {depth:.3f} m around pixel "
              f"({u:.0f}, {v:.0f})")
    if depth < min_depth_m:
        raise FallbackPickError(
            "Something is too close to the camera above the marked spot (a hand, "
            "or the robot?). Clear it and try again.",
            f"{detail}; min_depth_m is {min_depth_m}")
    if depth > max_depth_m:
        raise FallbackPickError(
            "The camera sees past the table at the marked spot. Check the camera.",
            f"{detail}; max_depth_m is {max_depth_m}")
    if z < cfg.min_surface_z_m:
        raise FallbackPickError(
            "Nothing on the marked spot. Put an item on it and try again.",
            f"{detail}; min_surface_z_m is {cfg.min_surface_z_m}")
    return SurfaceReading(z=z, depth_m=depth, pixel=(u, v))
