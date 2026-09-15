#!/usr/bin/env python3
"""Fast checks for the fallback pick's geometry — no ROS graph, no camera.

    python3 -m pytest test/test_fallback_pick.py -v

recycle_bot/fallback_pick.py finds a configured spot in the depth image and
measures how tall the item on it is. Everything is exercised here against a
synthetic depth image rendered from a known scene, so a sign error or a frame
mix-up shows up as a wrong height rather than as a robot pressing into the table.
"""

import math

import numpy as np
import pytest

from recycle_bot.fallback_pick import (
    FallbackPickConfig,
    FallbackPickError,
    Pinhole,
    RigidTransform,
    measure_pick_height,
    parse_config,
    sample_depth,
)

CAMERA = Pinhole(fx=920.0, fy=920.0, cx=640.0, cy=360.0, width=1280, height=720)
CAMERA_XY = (-0.30, -0.05)   # camera position in "base"
CAMERA_HEIGHT = 0.63


def looking_down(cx, cy, height):
    """cam_from_base for a camera at (cx, cy, height) looking straight down.

    180 deg about x: optical x = base x, optical y = -base y, optical z = down.
    """
    return RigidTransform((-cx, cy, height), (1.0, 0.0, 0.0, 0.0))


CAM_FROM_BASE = looking_down(*CAMERA_XY, CAMERA_HEIGHT)


def render_depth(boxes, table_depth=CAMERA_HEIGHT, invalid=0.0, seed=0):
    """uint16 millimetre depth image of a table plus flat-topped boxes.

    boxes: (x, y, half_size, height) in "base". Each pixel sees a box top if the
    ray at that box's height lands inside its footprint; box sides are ignored.
    """
    v, u = np.mgrid[0:CAMERA.height, 0:CAMERA.width].astype(float)
    depth = np.full(u.shape, table_depth)
    for bx, by, half, h in boxes:
        z_cam = CAMERA_HEIGHT - h
        x = (u - CAMERA.cx) / CAMERA.fx * z_cam + CAMERA_XY[0]
        y = -((v - CAMERA.cy) / CAMERA.fy * z_cam) + CAMERA_XY[1]
        inside = (np.abs(x - bx) <= half) & (np.abs(y - by) <= half)
        depth[inside] = np.minimum(depth[inside], z_cam)
    img = np.round(depth * 1000.0).astype(np.uint16)
    if invalid:
        rng = np.random.default_rng(seed)
        img[rng.random(img.shape) < invalid] = 0
    return img


def measure(cfg, img, min_depth=0.3, max_depth=0.7):
    return measure_pick_height(
        cfg, CAM_FROM_BASE, CAMERA,
        lambda u, v, half: sample_depth(img, u, v, half, 0.001),
        min_depth, max_depth,
    )


def spot(x, y, min_surface_z=0.005):
    return FallbackPickConfig(x=x, y=y, sample_size_m=0.03, min_surface_z_m=min_surface_z)


# -----------------------------------------------------------------------------
# building blocks
# -----------------------------------------------------------------------------

def test_transform_inverse_round_trips():
    tf = RigidTransform((0.3, -0.1, 0.6), (-0.5, 0.5, 0.5, 0.5))
    p = (0.12, -0.34, 0.05)
    back = tf.inverse().apply(tf.apply(p))
    assert all(abs(a - b) < 1e-12 for a, b in zip(back, p))


def test_looking_down_camera_sees_table_at_its_height():
    assert CAM_FROM_BASE.apply((*CAMERA_XY, 0.0)) == pytest.approx((0.0, 0.0, CAMERA_HEIGHT))


def test_pinhole_round_trips():
    p = CAMERA.unproject(900.0, 100.0, 0.55)
    assert CAMERA.project(p) == pytest.approx((900.0, 100.0))


def test_sample_depth_ignores_missing_readings_and_edges():
    img = np.full((50, 50), 600, dtype=np.uint16)
    img[20:30, 20:30] = 0
    assert sample_depth(img, 25, 25, 10, 0.001) == pytest.approx(0.600)
    assert sample_depth(img, 25, 25, 3, 0.001) is None          # nothing but holes
    assert sample_depth(img, 0, 0, 10, 0.001) == pytest.approx(0.600)   # clipped window


def test_sample_depth_float_images_skip_nan():
    img = np.full((20, 20), 0.5, dtype=np.float32)
    img[5:15, 5:15] = np.nan
    assert sample_depth(img, 2, 2, 2, 1.0) == pytest.approx(0.5)


# -----------------------------------------------------------------------------
# measuring the height
# -----------------------------------------------------------------------------

def test_item_right_under_the_camera():
    img = render_depth([(*CAMERA_XY, 0.04, 0.07)])
    reading = measure(spot(*CAMERA_XY), img)
    assert reading.z == pytest.approx(0.07, abs=0.002)
    assert reading.pixel == pytest.approx((640.0, 360.0))


def test_tall_item_off_axis_is_found_at_its_top():
    """Off the optical axis the top of a tall item sits pixels away from its foot.

    Reading depth at the pixel of the spot's foot would land on the table next
    to a narrow item; the re-projection must follow the ray up to its top.
    """
    x, y = CAMERA_XY[0] + 0.15, CAMERA_XY[1] + 0.12
    img = render_depth([(x, y, 0.015, 0.20)])
    reading = measure(spot(x, y), img)
    assert reading.z == pytest.approx(0.20, abs=0.003)

    foot = CAMERA.project(CAM_FROM_BASE.apply((x, y, 0.0)))
    assert math.dist(foot, reading.pixel) > 30, "test scene does not exercise parallax"


def test_noisy_depth_still_measures():
    img = render_depth([(*CAMERA_XY, 0.04, 0.05)], invalid=0.3)
    assert measure(spot(*CAMERA_XY), img).z == pytest.approx(0.05, abs=0.002)


def test_empty_spot_is_refused():
    with pytest.raises(FallbackPickError, match="Nothing on the marked spot") as err:
        measure(spot(*CAMERA_XY), render_depth([]))
    assert "min_surface_z_m" in err.value.detail


def test_spot_out_of_view_is_refused():
    # 0.6 m sideways at 0.63 m height is beyond a D415-like field of view
    with pytest.raises(FallbackPickError, match="outside the camera's view"):
        measure(spot(CAMERA_XY[0] + 0.6, CAMERA_XY[1]), render_depth([]))


def test_base_link_coordinates_are_caught():
    """The recurring foot-gun: a spot measured in base_link has x/y negated.

    Mirrored through the origin, it lands ~0.6 m from this camera — out of view.
    """
    with pytest.raises(FallbackPickError, match="not base_link"):
        measure(spot(-CAMERA_XY[0], -CAMERA_XY[1]), render_depth([]))


def test_something_near_the_camera_is_refused():
    img = render_depth([(*CAMERA_XY, 0.10, 0.45)])   # a hand 18 cm below the lens
    with pytest.raises(FallbackPickError, match="too close to the camera"):
        measure(spot(*CAMERA_XY), img)


def test_no_depth_is_refused():
    img = np.zeros((CAMERA.height, CAMERA.width), dtype=np.uint16)
    with pytest.raises(FallbackPickError, match="cannot measure the depth"):
        measure(spot(*CAMERA_XY), img)


# -----------------------------------------------------------------------------
# config
# -----------------------------------------------------------------------------

def test_parse_config():
    assert parse_config({}) is None
    cfg = parse_config({"fallback_pick": {"position": [-0.3, 0.05]}})
    assert (cfg.x, cfg.y) == (-0.3, 0.05)
    assert cfg.sample_size_m > 0.0


def test_parse_config_rejects_a_configured_height():
    with pytest.raises(ValueError, match="height is measured"):
        parse_config({"fallback_pick": {"position": [-0.3, 0.05, 0.1]}})
