#!/usr/bin/env python3
"""Run the YOLO model over image files — no camera, no ROS graph, no robot.

For answering "does this model actually see things correctly?" after a swap,
without needing the RealSense pointed at a scene or the arm powered.

rec_bot_vision_only.launch.py covers the live case. This covers the other one:
a fixed set of images, run repeatably, so two models can be compared on the same
input and a regression is visible rather than remembered.

It also cross-checks what it detects against bin_routing, because a class the
model emits but the config does not route is invisible at runtime — the object
is simply placed in default_bin, with no error anywhere.

    ros2 run recycle_bot test_model --image bottle.jpg
    ros2 run recycle_bot test_model --dir ~/samples --save-dir /tmp/annotated
    ros2 run recycle_bot test_model --list-classes
"""

import argparse
import os
import sys

IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def resolve_model(name=None):
    """Absolute path to a model in the installed pkg_resources directory."""
    from ament_index_python.packages import get_package_share_directory

    resources = os.path.join(
        get_package_share_directory("recycle_bot"), "pkg_resources"
    )
    if name and os.path.isabs(name):
        return name
    if name:
        return os.path.join(resources, name)

    # Default to whatever rec_bot_vision actually loads, read as text rather
    # than imported — importing it would pull in the whole camera/ROS stack.
    import importlib.util
    import re

    spec = importlib.util.find_spec("recycle_bot.rec_bot_vision")
    if spec and spec.origin:
        with open(spec.origin) as f:
            match = re.search(r'"([A-Za-z0-9_.\-]+\.pt)"', f.read())
        if match:
            return os.path.join(resources, match.group(1))
    raise RuntimeError("could not determine the active model; pass --model")


def load_routing(ur_type):
    """bin_routing and default_bin for an arm, or (None, None) if unavailable."""
    try:
        import yaml

        from recycle_bot.robot_profile import config_path

        with open(config_path(ur_type, "sorting_sequence.yaml")) as f:
            data = yaml.safe_load(f)
        return (data.get("bin_routing") or {}), data.get("default_bin")
    except Exception:
        return None, None


def gather_images(args):
    paths = []
    for image in args.image or []:
        paths.append(image)
    if args.dir:
        for entry in sorted(os.listdir(args.dir)):
            if entry.lower().endswith(IMAGE_SUFFIXES):
                paths.append(os.path.join(args.dir, entry))
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the YOLO model over image files, offline.",
    )
    parser.add_argument("--image", action="append", help="Image file (repeatable).")
    parser.add_argument("--dir", help="Directory of images to run over.")
    parser.add_argument("--model", help="Model filename in pkg_resources, or an "
                                        "absolute path. Defaults to the one "
                                        "rec_bot_vision loads.")
    parser.add_argument("--conf", type=float, default=None,
                        help="Confidence threshold. Defaults to the cell's "
                             "detection_filter.min_confidence.")
    parser.add_argument("--ur-type", default=None,
                        help="Arm whose config supplies the threshold and routing.")
    parser.add_argument("--save-dir", help="Write annotated images here.")
    parser.add_argument("--list-classes", action="store_true",
                        help="Print the model's classes and exit.")
    args = parser.parse_args(argv)

    try:
        model_path = resolve_model(args.model)
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    if not os.path.exists(model_path):
        print(f"ERROR: no such model: {model_path}", file=sys.stderr)
        return 2

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics is not available. This must run inside the "
              "container.", file=sys.stderr)
        return 2

    print(f"model: {model_path}")
    model = YOLO(model_path)
    names = (
        [model.names[k] for k in sorted(model.names)]
        if isinstance(model.names, dict) else list(model.names)
    )
    print(f"classes ({len(names)}): {names}\n")

    # Which arm's config to compare against.
    ur_type = args.ur_type
    if ur_type is None:
        try:
            from recycle_bot.robot_profile import resolve_ur_type
            ur_type = resolve_ur_type(None)
        except Exception:
            ur_type = None

    routing, default_bin = load_routing(ur_type) if ur_type else (None, None)
    if routing is not None:
        unrouted = [n for n in names if n not in routing]
        if unrouted:
            print(f"WARNING: {len(unrouted)} class(es) have no bin_routing rule "
                  f"in the {ur_type} config and would go to '{default_bin}':")
            for n in unrouted:
                print(f"    {n}")
            print()

    if args.list_classes:
        return 0

    threshold = args.conf
    if threshold is None and ur_type:
        try:
            import yaml
            from recycle_bot.robot_profile import config_path
            with open(config_path(ur_type, "calibration.yaml")) as f:
                threshold = float(
                    yaml.safe_load(f)["detection_filter"]["min_confidence"]
                )
        except Exception:
            threshold = None
    if threshold is None:
        threshold = 0.25
    print(f"confidence threshold: {threshold}"
          f"{' (from ' + str(ur_type) + ' calibration.yaml)' if args.conf is None and ur_type else ''}\n")

    paths = gather_images(args)
    if not paths:
        print("ERROR: no images given. Use --image and/or --dir.", file=sys.stderr)
        return 2

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    total = 0
    below = 0
    for path in paths:
        if not os.path.exists(path):
            print(f"{path}: MISSING")
            continue
        # Run with no threshold so detections that fall just under it are still
        # visible — "the model saw it but the filter dropped it" is a different
        # problem from "the model saw nothing", and they look identical if the
        # threshold is applied inside the model.
        results = model.predict(path, conf=0.01, verbose=False)
        print(f"{os.path.basename(path)}")
        found = False
        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                label = names[int(box.cls[0])]
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                keep = conf >= threshold
                if keep:
                    total += 1
                else:
                    below += 1
                found = True
                target = (routing or {}).get(label, default_bin) if routing is not None else "?"
                mark = "  " if keep else "  (below threshold) "
                print(f"   {mark}{label:<26} conf={conf:.2f}  "
                      f"bbox=({x1:.0f},{y1:.0f})-({x2:.0f},{y2:.0f})  -> {target}")
            if args.save_dir:
                out = os.path.join(args.save_dir, os.path.basename(path))
                result.save(filename=out)
        if not found:
            print("    no detections at all")
        print()

    if args.save_dir:
        print(f"annotated images written to {args.save_dir}")
    print(f"{total} detection(s) at or above {threshold}; "
          f"{below} below it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
