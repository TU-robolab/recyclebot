# System Imports
import os
import time

from collections import deque
from threading import Lock

# ROS2 imports
import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from std_srvs.srv import Trigger
from realsense2_camera_msgs.msg import RGBD
from ament_index_python.packages import get_package_share_directory


# vision imports
import cv2
import torch
import numpy as np

from ultralytics import YOLO
from cv_bridge import CvBridge



class VisionDetector(Node):
    def __init__(self):
        super().__init__("vision_detector")
        

        # initialize yolo model  (model in pkg_resources location)
        tmp_model_path = os.path.join(
            get_package_share_directory("recycle_bot"),
            "pkg_resources",
            "RecycleBotDIS3Obj_full.pt", #rb-lab-data-hannover-messe-v3-171025.pt, DIS.pt # The DIS is only the BOX
        )
        self.model = YOLO(tmp_model_path)

        # set hardware device for inference
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")
        self.model.to(self.device)

        # use model-provided labels to avoid mismatch
        if isinstance(self.model.names, dict):
            self.class_labels = [self.model.names[k] for k in sorted(self.model.names.keys())]
        else:
            self.class_labels = list(self.model.names)
        
        # RGBD data (protected by rgbd_lock)
        self.bridge = CvBridge()
        self.rgbd_lock = Lock()  # protects: last_rgbd_image
        self.last_rgbd_image = None
        self.camera_frame_id = ""

        # Detection bookkeeping (protected by detection_lock):
        #   pending_detections — new detections awaiting publication (drained by
        #                        process_deque at 10 Hz)
        #   recent_detections  — dedup memory; entries stay for dedup_window_s so
        #                        a stationary object is not re-published on every
        #                        capture. Keep the window short: downstream
        #                        consumers (viz, tests) expect a periodic stream,
        #                        and the control node dedups tasks by 3D pose
        #                        anyway — this window only rate-limits publishes.
        self.pending_detections = deque()
        self.recent_detections = deque(maxlen=128)
        self.detection_lock = Lock()
        self.dedup_window_s = (
            self.declare_parameter("dedup_window_s", 2.0)
            .get_parameter_value()
            .double_value
        )

        # threshold to weed out duplicate detections
        self.similarity_threshold = 0.5     #0.7

        # depth scale: converts raw depth values to meters
        # D415 default: 0.001 (raw values in mm, so mm * 0.001 = meters)
        self.depth_scale = (
            self.declare_parameter("depth_scale", 0.001)
            .get_parameter_value()
            .double_value
        )
        
        # create ROS2 interfaces to trigger capture of goals.
        # Service and the periodic auto-capture timer share one mutually-exclusive
        # callback group so two YOLO inferences can never run concurrently.
        self.capture_cb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(Trigger, "capture_detections",
                                       self.trigger_callback,
                                       callback_group=self.capture_cb_group
        )

        # Auto-capture: run detection periodically so it doesn't need a manual
        # /capture_detections call. Period in seconds; set to 0 to disable and
        # fall back to manual triggering only.
        self.auto_capture_period_s = (
            self.declare_parameter("auto_capture_period_s", 1.0)
            .get_parameter_value()
            .double_value
        )
        if self.auto_capture_period_s > 0.0:
            self.auto_capture_timer = self.create_timer(
                self.auto_capture_period_s,
                self.auto_capture_callback,
                callback_group=self.capture_cb_group,
            )
            self.get_logger().info(
                f"Auto-capture enabled every {self.auto_capture_period_s:.2f}s"
            )
        
        """
            subscribe to vision topic for rbg image from realsense: 
            depth(0)  Format:RGB8 , Width:1280, Height:720, FPS:30
        """
        # setup ROS quality of service for camera frames
        qos_camera_feed = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # keep only the latest frames
            depth=5,  # buffer up to 5 frames
            reliability=ReliabilityPolicy.BEST_EFFORT, # Drop frames if necessary for speed
            durability=DurabilityPolicy.VOLATILE  # no need to keep old frames
        )

        self.image_sub = self.create_subscription(
            RGBD,
            "/camera/camera/rgbd",
            self.image_callback,
            qos_camera_feed, 
            callback_group=ReentrantCallbackGroup()
        )

        # setup ROS quality of service for detections
        qos_detected_objects = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # store recent messages
            depth=10,  # buffer up to 10 detections
            reliability=ReliabilityPolicy.RELIABLE,  # ensure all detections arrive
            durability=DurabilityPolicy.VOLATILE  # no need to retain past detections
        )
        
        # publish an array of current 3D detections (with depth)
        self.detection_pub = self.create_publisher(Detection3DArray,
                                                  "object_detections",
                                                   qos_detected_objects
        )
        # timer for processing detections queue, 10Hz
        self.timer = self.create_timer(0.1, self.process_deque, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("vision detection node initialized")

    """ 
    thread-safe image callback, only 1 thread can update 
    last image at a time, with statement ensures lock lifecycle
    is automated (creation and release)
    image type is realsense2_msgs.msg (RGBD) -> cv image etc. 
    """
    def image_callback(self, msg):
        with self.rgbd_lock:
            self.last_rgbd_image = msg
        
    """ 
    thread-safe detection callback, runs the model on capture
    and publishes the detections 
    """
    def capture_detections(self):
        """Run YOLO on the latest RGBD frame and add new detections to the deque.

        Returns the number of newly added detections, or None if no image is
        available yet. Shared by the /capture_detections service and the
        periodic auto-capture timer.
        """
        cv_image = None
        depth_cv_image = None
        # keep lock on last image only as long as necessary
        with self.rgbd_lock:
            if self.last_rgbd_image is None:
                return None

            depth_cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.depth, "passthrough")

            # convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.rgb, self.last_rgbd_image.rgb.encoding)

            # camera frame for the published Detection3DArray header
            self.camera_frame_id = self.last_rgbd_image.rgb.header.frame_id


        # Uncomment for debug visualization (disables headless testing).
        # Runs in a separate thread to avoid blocking the inference callback
        # (requires: from threading import Thread)
        #Thread(target=self.show_rgbd, args=(cv_image, depth_cv_image)).start()

        # run inference with YOLO11 (outside of image lock, confidence threshold of 0.5)
        inf_results = self.model(cv_image, conf=0.5)
        self.get_logger().debug(f"NN output raw inference output: {inf_results}")
        self.get_logger().debug(f"NN output raw inference boxes: {inf_results[0].boxes}")


        # process detections
        detections = self.process_yolo_results(inf_results, cv_image, depth_cv_image)

        self.get_logger().debug(f"NN output raw detections: {detections}")
        # add unique detections (only alter detection state inside the lock)
        with self.detection_lock:
            self._expire_old_detections()
            added_count = 0
            for det in detections:
                if not self.is_duplicate(det):
                    self.recent_detections.append(det)
                    self.pending_detections.append(det)
                    added_count += 1

        return added_count

    def _expire_old_detections(self):
        """Drop dedup-memory entries older than the dedup window.

        Must be called with detection_lock held. recent_detections is ordered by
        insertion time, so popping from the left is sufficient.
        """
        cutoff = time.time() - self.dedup_window_s
        while self.recent_detections and self.recent_detections[0]["timestamp"] < cutoff:
            self.recent_detections.popleft()

    """
    service wrapper: manual /capture_detections trigger
    """
    def trigger_callback(self, request, response):
        added_count = self.capture_detections()
        if added_count is None:
            response.success = False
            response.message = "No image available"
            return response

        response.success = True
        response.message = f"Added {added_count} potential new detections"
        return response

    """
    timer wrapper: periodic auto-capture (no manual trigger needed)
    """
    def auto_capture_callback(self):
        added_count = self.capture_detections()
        if added_count is None:
            self.get_logger().debug("Auto-capture: no image available yet")
        elif added_count > 0:
            self.get_logger().debug(f"Auto-capture added {added_count} new detections")

    def show_rgbd(self, rgb_img, depth_img):
        # create colorized depth visualization
        # INVERT so close=255 (red/warm) and far=0 (blue/cool)
        valid_mask = depth_img > 0
        depth_normalized = np.zeros_like(depth_img, dtype=np.uint8)

        if np.any(valid_mask):
            valid_depth = depth_img[valid_mask]
            depth_min, depth_max = np.min(valid_depth), np.max(valid_depth)

            # normalize and INVERT: close objects → 255 (red), far objects → 0 (blue)
            depth_normalized[valid_mask] = (
                255 - ((depth_img[valid_mask] - depth_min) / (depth_max - depth_min) * 255)
            ).astype(np.uint8)

        # apply colormap (TURBO: red=255=close, blue=0=far)
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_TURBO)

        # mark invalid pixels as black
        depth_colormap[~valid_mask] = [0, 0, 0]

        depth_colormap = cv2.resize(depth_colormap, (rgb_img.shape[1], rgb_img.shape[0]))

        depth_colormap = cv2.resize(depth_colormap, (rgb_img.shape[1]// 2, rgb_img.shape[0]//2))
        rgb_img = cv2.resize(rgb_img, (rgb_img.shape[1]//2, rgb_img.shape[0]//2))
        combined_img = np.hstack((rgb_img, depth_colormap))  # horizontal stack

        # show both images
        cv2.imshow("RGB + depth (colourmap)", combined_img)
        cv2.waitKey(0) # delays for any key press
        # close both windows
        cv2.destroyAllWindows()
        
    def process_yolo_results(self, results, img, depth_img):
        detections = []

        # process YOLO results (first detection result if batched)
        result = results[0]

        # image dimensions for bounds checking
        img_h, img_w = depth_img.shape[:2]

        # get bounding boxes and format detections object list with req params
        boxes = result.boxes

        for box in boxes:
            # get box coordinates (in xywh format) (center of bbox)
            cx, cy, w, h = box.xywh[0].cpu().numpy()

            # get confidence and class ID
            confidence = float(box.conf.cpu().numpy()[0])
            class_id = int(box.cls.cpu().numpy()[0])

            # compute bounding box corners for depth extraction
            #
            #   (x1, y1) ────────────┐
            #      │                 │
            #      │    depth_bbox   │
            #      │                 │
            #      └──────────── (x2, y2)
            #
            x1 = int(max(0, cx - w / 2))
            y1 = int(max(0, cy - h / 2))
            x2 = int(min(img_w, cx + w / 2))
            y2 = int(min(img_h, cy + h / 2))

            # Estimate object depth from the central half of the bbox using the
            # median: bbox edges are mostly background (table), and a mean over
            # the full box systematically overestimates depth for small objects
            # (reported pick pose too low → gripper presses into the object).
            qw = (x2 - x1) // 4
            qh = (y2 - y1) // 4
            central = depth_img[y1 + qh:y2 - qh, x1 + qw:x2 - qw]
            valid_depth = central[central > 0]  # exclude invalid pixels (0 = no reading)

            if len(valid_depth) == 0:
                # fall back to the full bbox if the central region has no reading
                depth_bbox = depth_img[y1:y2, x1:x2]
                valid_depth = depth_bbox[depth_bbox > 0]

            if len(valid_depth) > 0:
                avg_depth_m = float(np.median(valid_depth)) * self.depth_scale
            else:
                avg_depth_m = 0.0  # no valid depth readings

            # convert to correct format for our pipeline
            detection = {
                "class_id": class_id,
                "label": self.class_labels[class_id] if class_id < len(self.class_labels) else f"class_{class_id}",
                "confidence": confidence,
                "bbox_uv": (
                    cx,            # center_x
                    cy,            # center_y
                    int(w),        # width
                    int(h)         # height
                ),
                "depth_m": avg_depth_m,  # average depth in meters
                "timestamp": time.time()
            }

            detections.append(detection)

        return detections
   
    def is_duplicate(self, new_det):
        """IoU check against the dedup memory. Call with detection_lock held."""
        for existing_det in self.recent_detections:
            # calculate IoU for duplicate detection check
            # bbox_uv format: (center_x, center_y, width, height)
            box_a = new_det["bbox_uv"]
            box_b = existing_det["bbox_uv"]

            # convert from center format to corner format
            #
            #            w
            #    ┌───────────────┐
            #    │   (cx, cy)    │
            #  h │       ·       │
            #    │               │
            #    └───────────────┘
            #
            #  becomes:
            #
            #   (x1, y1) ────────┐
            #      │             │
            #      │             │
            #      └──────── (x2, y2)
            #
            a_x1 = box_a[0] - box_a[2] / 2
            a_y1 = box_a[1] - box_a[3] / 2
            a_x2 = box_a[0] + box_a[2] / 2
            a_y2 = box_a[1] + box_a[3] / 2

            b_x1 = box_b[0] - box_b[2] / 2
            b_y1 = box_b[1] - box_b[3] / 2
            b_x2 = box_b[0] + box_b[2] / 2
            b_y2 = box_b[1] + box_b[3] / 2

            # calculate intersection
            #
            #   box_a (───)          box_b (━━━)
            #
            #    ┌─────────────┐
            #    │         ┏━━━┿━━━━━━━┓
            #    │         ┃///│///////┃
            #    │         ┃///│///////┃
            #    └─────────┃───┘///////┃
            #              ┃///////////┃
            #              ┗━━━━━━━━━━━┛
            #
            #   intersection = ///
            #   IoU = intersection / union
            #
            inter_x1 = max(a_x1, b_x1)
            inter_y1 = max(a_y1, b_y1)
            inter_x2 = min(a_x2, b_x2)
            inter_y2 = min(a_y2, b_y2)

            inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
            union_area = (box_a[2] * box_a[3] + box_b[2] * box_b[3] - inter_area)

            if union_area > 0 and inter_area / union_area > self.similarity_threshold:
                return True
        return False

    def process_deque(self):
        detection_array = Detection3DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()

        with self.detection_lock:
            # skip if nothing new (recent_detections is dedup memory only)
            if not self.pending_detections:
                return

            detection_array.header.frame_id = self.camera_frame_id

            while self.pending_detections:
                # fifo order
                det = self.pending_detections.popleft()

                d = Detection3D()
                # bbox.center.position:
                #   x, y = pixel coordinates of bbox center
                #   z = average valid depth in meters (0 if no valid depth)
                d.bbox.center.position.x = float(det["bbox_uv"][0])
                d.bbox.center.position.y = float(det["bbox_uv"][1])
                d.bbox.center.position.z = float(det["depth_m"])
                d.bbox.size.x = float(det["bbox_uv"][2])
                d.bbox.size.y = float(det["bbox_uv"][3])
                d.bbox.size.z = 0.0  # not used

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = det["label"]
                hypothesis.hypothesis.score = det["confidence"]
                d.results.append(hypothesis)

                detection_array.detections.append(d)

        self.detection_pub.publish(detection_array)

def main(args=None):
    rclpy.init(args=args)
    
    executor = MultiThreadedExecutor()
    node = VisionDetector()
    
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
