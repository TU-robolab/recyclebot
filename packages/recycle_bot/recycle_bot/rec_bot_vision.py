# System Imports
import yaml
import os
import time
import getpass

from collections import deque
from threading import Lock, Thread

# ROS2 imports
import rclpy
import tf2_ros

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from std_srvs.srv import Trigger
from realsense2_camera_msgs.msg import RGBD


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
        tmp_model_path = os.path.join(os.path.expanduser("~"), "ros2_ws/src/recycle_bot/pkg_resources", "rb-lab-data-hannover-messe-v3-171025.pt" )
        self.model = YOLO(tmp_model_path)

        # set hardware device for inference
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")
        self.model.to(self.device)

        # used labels
        self.class_labels = ["bottle_pet","box_pp","bucket","canister","cup_pp-ps","flower_pot","lid_pp-ps","non-food_bottle","other","watering_can"]
        
        # RGBD data (protected by rgbd_lock)
        self.bridge = CvBridge()
        self.rgbd_lock = Lock()  # protects: last_rgbd_image
        self.last_rgbd_image = None
                
        # create list to track detected trash (max 128 values ~4.2 secs at 30FPS)
        self.detection_deque = deque(maxlen=128)
        self.detection_lock = Lock()

        # threshold to weed out duplicate detections
        self.similarity_threshold = 0.7

        # depth scale: converts raw depth values to meters
        # D415 default: 0.001 (raw values in mm, so mm * 0.001 = meters)
        # TODO: consider extracting from /camera/camera/depth/camera_info or parameter server
        self.depth_scale = 0.001  
        
        # create ROS2 interfaces to triger capture of goals
        self.srv = self.create_service(Trigger, "capture_detections", 
                                       self.trigger_callback,
                                       callback_group= MutuallyExclusiveCallbackGroup()
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
    def trigger_callback(self, request, response):
        cv_image = None
        depth_cv_image = None
        # keep lock on last image only as long as necessary
        with self.rgbd_lock:
            if self.last_rgbd_image is None:
                response.success = False
                response.message = "No image available"
                return response
            
            depth_cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.depth, "passthrough")

            # convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.rgb, self.last_rgbd_image.rgb.encoding)

            # camera info for conversions
            camera_info = self.last_rgbd_image.rgb_camera_info
        

        # display debug images
        #self.show_rgbd(cv_image,depth_cv_image)
        #(comment this line for headless testing) Launch visualization in separate thread 
        #Thread(target=self.show_rgbd, args=(cv_image, depth_cv_image)).start()

        # run inference with YOLO11 (outside of image lock, confidence threshold of 0.5)
        inf_results = self.model(cv_image, conf=0.5)  
        print(f"NN output raw inference output: {inf_results}")
        print(f"NN output raw inference boxes: {inf_results[0].boxes}")
       
        
        # process detections
        detections = self.process_yolo_results(inf_results, cv_image, depth_cv_image)

        print(f"NN output raw detections: {detections}")
        # add unique detections to deque (only alter detections inside the lock)
        with self.detection_lock:
            added_count = 0
            for det in detections:
                if not self.is_duplicate(det):
                    self.detection_deque.append(det)
                    added_count += 1

        
        response.success = True
        response.message = f"Added {added_count} potential new detections"

        return response            

    def show_rgbd(self, rgb_img, depth_img):
        # create colorized depth visualization
        # INVERT so close=255 (red/warm) and far=0 (blue/cool)
        valid_mask = (depth_img > 0) & (depth_img < 65535)
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

            # extract depth region and compute average depth in meters
            depth_bbox = depth_img[y1:y2, x1:x2]
            valid_depth = depth_bbox[(depth_bbox > 0) & (depth_bbox < 65535)]  # exclude invalid pixels

            if len(valid_depth) > 0:
                avg_depth_m = float(np.mean(valid_depth)) * self.depth_scale
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
        for existing_det in self.detection_deque:
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
        # skip if empty
        if not self.detection_deque:
            return

        detection_array = Detection3DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()

        with self.detection_lock:
            while self.detection_deque:
                # fifo order
                det = self.detection_deque.popleft()

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
