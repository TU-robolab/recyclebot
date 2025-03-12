# System Imports
import yaml
import os
import time

from collections import deque
from threading import Lock

# ROS2 imports
import rclpy
import tf2_ros
import rospy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_srvs.srv import Trigger

# vision imports
import cv2
import onnxruntime as ort
import numpy as np

from cv_bridge import CvBridge



class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        
        # initialize ONNX model initialization
        self.model = ort.InferenceSession('object_detection.onnx', 
                         providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
        )
        # used labels
        self.class_labels = ['class1', 'class2', ...]  
        
        # Image configuration
        self.bridge = CvBridge()
        self.last_image = None
        self.image_lock = Lock()
        
        # create list to track detected trash (max 128 values ~4.2 secs at 30FPS)
        self.detection_deque = deque(maxlen=128)
        self.detection_lock = Lock()

        # threshold to weed out duplicate detections
        self.similarity_threshold = 0.7  
        
        # create ROS2 interfaces to triger capture of goals
        self.srv = self.create_service(Trigger, 'capture_detections', 
                                       self.trigger_callback,
                                       callback_group=ReentrantCallbackGroup()
        )
        
        # subscribe to vision topic
        self.image_sub = self.create_subscription(
            Image,
            #'/camera/color/image_raw',
           # self.image_callback,
           # 10,
           # callback_group=ReentrantCallbackGroup()
        )

        # publish an array of current detections     
        self.detection_pub = self.create_publisher(Detection2DArray, 
                                                  'object_detections', 10
        )
        # timer for processing detections queue, every 100 ms 
        self.timer = self.create_timer(0.1, self.process_deque)
        
        self.get_logger().info("vision detection node initialized")

    """ 
    thread-safe image callback, only 1 thread can update 
    last image at a time, with statement ensures lock lifecycle
    is automated (creation and release)
    """
    def image_callback(self, msg):
        with self.image_lock:
            self.last_image = msg

    """ 
    thread-safe detection callback, runs the model on capture
    and publish the detection 
    """
    def trigger_callback(self, request, response):
        with self.image_lock:
            if self.last_image is None:
                response.success = False
                response.message = "No image available"
                return response
            
            cv_image = self.bridge.imgmsg_to_cv2(self.last_image, 'bgr8')
            
        # preprocess image for ONNX model
        input_tensor = self.preprocess_image(cv_image)
        
        # run inference
        outputs = self.model.run(None, {'input': input_tensor})
        
        # process detections
        detections = self.postprocess_output(outputs, cv_image.shape)
        
        # Add unique detections to deque
        with self.detection_lock:
            for det in detections:
                if not self.is_duplicate(det):
                    self.detection_deque.append(det)
        
        response.success = True
        response.message = f"Added {len(detections)} potential new detections"
        return response

    def preprocess_image(self, image):
        # model-specific preprocessing example
        
        resized = cv2.resize(image, (640, 640))
        normalized = resized.astype(np.float32) / 255.0
        return np.transpose(normalized, (2, 0, 1))[np.newaxis, ...]

    def postprocess_output(self, outputs, img_shape):
        detections = []
        boxes = outputs[0][0]
        scores = outputs[1][0]
        class_ids = outputs[2][0]
        
        for box, score, class_id in zip(boxes, scores, class_ids):
            if score < 0.5:  # confidence threshold to skip detection
                continue
                
            # Convert coordinates to image space
            y_min, x_min, y_max, x_max = box
            height, width = img_shape[:2]
            
            detection = {
                'class_id': int(class_id),
                'label': self.class_labels[int(class_id)],
                'confidence': float(score),
                'bbox': (
                    int(x_min * width),
                    int(y_min * height),
                    int((x_max - x_min) * width),
                    int((y_max - y_min) * height)
                ),
                'timestamp': time.time()
            }
            detections.append(detection)
        
        return detections

    def is_duplicate(self, new_det):
        for existing_det in self.detection_deque:
            # Calculate IoU for duplicate detection check
            box_a = new_det['bbox']
            box_b = existing_det['bbox']
            
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
        if not self.detection_deque:
            return
            
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        
        with self.detection_lock:
            while self.detection_deque:
                # fifo order 
                det = self.detection_deque.popleft()
                
                d = Detection2D()
                d.bbox.center.position.x = float(det['bbox'][0] + det['bbox'][2]/2)
                d.bbox.center.position.y = float(det['bbox'][1] + det['bbox'][3]/2)
                d.bbox.size_x = float(det['bbox'][2])
                d.bbox.size_y = float(det['bbox'][3])
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = det['label']
                hypothesis.hypothesis.score = det['confidence']
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

if __name__ == '__main__':
    main()
