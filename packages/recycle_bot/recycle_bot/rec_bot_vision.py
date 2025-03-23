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
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
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
        tmp_model_path = os.path.join(os.path.expanduser("~"), "ros2_ws/src/recycle_bot/pkg_resources", "rb-y11-v3.pt" )
        self.model = YOLO(tmp_model_path)

        # set hardware device for inference
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")
        self.model.to(self.device)

        # used labels
        self.class_labels = ["bottle_pet","box_pp","bucket","canister","cup_pp-ps","flower_pot","lid_pp-ps","non-food_bottle","other","watering_can"]
        
        # image configuration
        self.bridge = CvBridge()
        self.last_rgbd_image = None
        self.last_depth_image = None
        self.image_lock = Lock()
                
        # create list to track detected trash (max 128 values ~4.2 secs at 30FPS)
        self.detection_deque = deque(maxlen=128)
        self.detection_lock = Lock()

        # threshold to weed out duplicate detections
        self.similarity_threshold = 0.7  
        
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
        
        # publish an array of current detections     
        self.detection_pub = self.create_publisher(Detection2DArray, 
                                                  "object_detections",
                                                   qos_detected_objects
        )
        # timer for processing detections queue, 10Hz
        self.timer = self.create_timer(1, self.process_deque, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("vision detection node initialized")

    """ 
    thread-safe image callback, only 1 thread can update 
    last image at a time, with statement ensures lock lifecycle
    is automated (creation and release)
    image type is realsense2_msgs.msg (RGBD) -> cv image etc. 
    """
    def image_callback(self, msg):
        with self.image_lock:
            self.last_rgbd_image = msg
        
    """ 
    thread-safe detection callback, runs the model on capture
    and publishes the detections 
    """
    def trigger_callback(self, request, response):
        cv_image = None
        depth_cv_image = None
        # keep lock on last image only as long as necessary
        with self.image_lock:
            if self.last_rgbd_image is None:
                response.success = False
                response.message = "No image available"
                return response
            
            depth_cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.depth, "passthrough")

            # convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.last_rgbd_image.rgb, self.last_rgbd_image.rgb.encoding)

        # display debug images
        #self.show_rgbd(cv_image,depth_cv_image)
        # Launch visualization in separate thread
        Thread(target=self.show_rgbd, args=(cv_image, depth_cv_image)).start()

        # run inference with YOLO11 (outside of image lock, confidence threshold of 0.5)
        inf_results = self.model(cv_image, conf=0.5)  
        
        print("at trigger process")
        # process detections
        detections = self.process_yolo_results(inf_results, cv_image)
        
        print("at process")
        # add unique detections to deque (only alter detections inside the lock)
        with self.detection_lock:
            added_count = 0
            print("at detection lock")
            for det in detections:
                if not self.is_duplicate(det):
                    self.detection_deque.append(det)
                    added_count += 1

        
        response.success = True
        response.message = f"Added {added_count} potential new detections"

        print("message response for trigger")
        return response            

    def show_rgbd(self, rgb_img, depth_img):
        # normalize depth for visualization
        depth_display = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        depth_display = depth_display.astype(np.uint8)

        # apply colormap for better visibility
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
        depth_colormap = cv2.resize(depth_colormap, (rgb_img.shape[1], rgb_img.shape[0]))

        depth_colormap = cv2.resize(depth_colormap, (rgb_img.shape[1]// 2, rgb_img.shape[0]//2))
        rgb_img = cv2.resize(rgb_img, (rgb_img.shape[1]//2, rgb_img.shape[0]//2))
        combined_img = np.hstack((rgb_img, depth_colormap))  # horizontal stack

        # show both images
        cv2.imshow("RGB + depth (colourmap)", combined_img)
        cv2.waitKey(30)  #delays  second
        # close both windows
        cv2.destroyAllWindows()
        
    def process_yolo_results(self, results, img):
        detections = []
        
        # process YOLO results (first detection result if batched)
        result = results[0]
        
        # get bounding boxes and format detections object list with req params
        boxes = result.boxes
        
        for box in boxes:
            # get box coordinates (in xywh format)
            x, y, w, h = box.xywh[0].cpu().numpy()
            
            # get confidence and class ID
            confidence = float(box.conf.cpu().numpy()[0])
            class_id = int(box.cls.cpu().numpy()[0])
            
            # convert to correct format for our pipeline
            detection = {
                "class_id": class_id,
                "label": self.class_labels[class_id] if class_id < len(self.class_labels) else f"class_{class_id}",
                "confidence": confidence,
                "bbox": (
                    int(x - w/2),  # x1
                    int(y - h/2),  # y1
                    int(w),        # width
                    int(h)         # height
                ),
                "timestamp": time.time()
            }
            
            detections.append(detection)
        
        return detections
   
    def is_duplicate(self, new_det):
        for existing_det in self.detection_deque:
            # Calculate IoU for duplicate detection check
            box_a = new_det["bbox"]
            box_b = existing_det["bbox"]
            
            x_a = max(box_a[0], box_b[0])
            y_a = max(box_a[1], box_b[1])
            x_b = min(box_a[0]+box_a[2], box_b[0]+box_b[2])
            y_b = min(box_a[1]+box_a[3], box_b[1]+box_b[3])
            
            inter_area = max(0, x_b - x_a) * max(0, y_b - y_a)
            union_area = (box_a[2]*box_a[3] + box_b[2]*box_b[3] - inter_area)
            
            if inter_area / union_area > self.similarity_threshold:
                return True
        return False

    def process_deque(self):
        # skip if empty
        print("at beginning of process")
        if not self.detection_deque:
            print("at empty process deque")
            return
            
        print ("at non empty process deque")    
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        
        with self.detection_lock:

            print("with detection lock")

            while self.detection_deque:
                # fifo order 
                det = self.detection_deque.popleft()
                
                d = Detection2D()
                d.bbox.center.position.x = float(det["bbox"][0] + det["bbox"][2]/2)
                d.bbox.center.position.y = float(det["bbox"][1] + det["bbox"][3]/2)
                d.bbox.size_x = float(det["bbox"][2])
                d.bbox.size_y = float(det["bbox"][3])
                
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
