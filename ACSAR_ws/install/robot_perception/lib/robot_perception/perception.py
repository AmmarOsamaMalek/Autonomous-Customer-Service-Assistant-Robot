#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
import tf2_ros
from math import atan2, cos, sin, pi
from rclpy.duration import Duration
from tf2_geometry_msgs import do_transform_pose_stamped


class HandDetector(Node):
    def __init__(self):
        super().__init__('hand_detector')
        
        
        self.bridge = CvBridge()
        
        # Parameters for processing control
        self.declare_parameter('processing_rate', 10.0)  
        self.declare_parameter('downscale_factor', 2)   
        self.declare_parameter('hand_raised_threshold', 0.25)  
        self.declare_parameter('detection_confidence', 0.6)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('model_complexity', 0)    
        
        
        self.processing_rate = self.get_parameter('processing_rate').value
        self.downscale_factor = self.get_parameter('downscale_factor').value
        self.hand_raised_threshold = self.get_parameter('hand_raised_threshold').value
        detection_confidence = self.get_parameter('detection_confidence').value
        tracking_confidence = self.get_parameter('tracking_confidence').value
        model_complexity = self.get_parameter('model_complexity').value
        
        # Initialize MediaPipe Hands 
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  
            min_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence,
            model_complexity=model_complexity  
        )
        
        # Initialize TF2 buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsic parameters 
        self.declare_parameter('camera_fx', 460.0)
        self.declare_parameter('camera_fy', 460.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        
        fx = self.get_parameter('camera_fx').value
        fy = self.get_parameter('camera_fy').value
        cx = self.get_parameter('camera_cx').value
        cy = self.get_parameter('camera_cy').value
        
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        
        # Hand detection state 
        self.hand_raised_streak = 0
        self.hand_lowered_streak = 0
        self.is_hand_raised = False
        self.last_hand_position = None
        self.last_process_time = self.get_clock().now()
        
        # Subscribers
        self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Publishers
        self.hand_raised_pub = self.create_publisher(Bool, '/hand_raised', 10)
        self.customer_pose_pub = self.create_publisher(PoseStamped, '/customer_pose', 10)
        
        # Debug publisher 
        self.debug_image_pub = self.create_publisher(Image, '/hand_detection_debug', 1)
        
        # Store latest lidar data
        self.latest_scan = None
        self.latest_scan_time = None
        
        # Rate limiter
        self.min_process_interval = 1.0 / self.processing_rate
        
        self.get_logger().info('Hand detector initialized with processing rate: '
                             f'{self.processing_rate}Hz, downscale factor: {self.downscale_factor}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion."""
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def lidar_callback(self, msg):
        """Store the latest lidar scan data and its timestamp."""
        self.latest_scan = msg
        self.latest_scan_time = msg.header.stamp

    def get_depth_from_lidar(self, angle_rad):
        """Get depth measurement from lidar at specific angle with averaging for stability."""
        if self.latest_scan is None:
            return None
            
        # Convert angle to lidar scan index
        angle_normalized = angle_rad
        while angle_normalized > pi:
            angle_normalized -= 2 * pi
        while angle_normalized < -pi:
            angle_normalized += 2 * pi
            
        index = int((angle_normalized - self.latest_scan.angle_min) / 
                   self.latest_scan.angle_increment)
        
        # Use a window of measurements for more stability
        window_size = 3  # Number of samples on each side
        valid_ranges = []
        
        for i in range(max(0, index - window_size), min(len(self.latest_scan.ranges), index + window_size + 1)):
            depth = self.latest_scan.ranges[i]
            if self.latest_scan.range_min <= depth <= self.latest_scan.range_max:
                valid_ranges.append(depth)
                
        if valid_ranges:
            # Return median for robustness against outliers
            return np.median(valid_ranges)
        
        return None

    def get_latest_transform_time(self):
        """Get the latest available transform time."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                tf2_ros.Time(),
                timeout=Duration(seconds=1.0)
            )
            return transform.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Failed to get transform time: {str(e)}')
            return None

    def calculate_customer_pose(self, hand_coords, image_width, image_height):
        """Calculate customer pose from hand coordinates and lidar data."""
        if self.latest_scan is None:
            return None
            
        # Adjust camera matrix for downscaled image if needed
        fx = self.camera_matrix[0, 0] / self.downscale_factor
        cx = self.camera_matrix[0, 2] / self.downscale_factor
        
        # Convert pixel coordinates to angle
        px, py = hand_coords
        
        # Calculate angle from camera center to hand
        angle = atan2(px - cx, fx)
        
        # Get depth from lidar
        depth = self.get_depth_from_lidar(angle)
        if depth is None or not np.isfinite(depth):
            return None
        
        # Get the latest available transform time
        latest_transform_time = self.get_latest_transform_time()
        if latest_transform_time is None:
            latest_transform_time = self.get_clock().now().to_msg()
        
        # Create customer pose
        pose = PoseStamped()
        pose.header.frame_id = 'camera_link'
        pose.header.stamp = latest_transform_time
        
        # Calculate 3D position
        pose.pose.position.x = depth * cos(angle)
        pose.pose.position.y = depth * sin(angle)
        pose.pose.position.z = 0.0  # Assume hand is at robot height
        
        # Set orientation to face the robot (opposite of the vector from robot to hand)
        angle_to_customer = atan2(pose.pose.position.y, pose.pose.position.x)
        pose.pose.orientation = self.euler_to_quaternion(0, 0, angle_to_customer + pi)
        
        try:
            # Transform pose to map frame using the correct method
            # For PoseStamped, we should use do_transform_pose_stamped
            transform = self.tf_buffer.lookup_transform(
                'map',
                pose.header.frame_id,
                pose.header.stamp,
                timeout=Duration(seconds=1.0)
            )
            
            # Import the required transformation function
            from tf2_geometry_msgs import do_transform_pose_stamped
            
            # Apply the transform
            transformed_pose = do_transform_pose_stamped(pose, transform)
            return transformed_pose
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {str(e)}')
            return None

    def image_callback(self, msg):
        # Rate limiting to control processing frequency
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_process_time).nanoseconds / 1e9
        
        if time_diff < self.min_process_interval:
            return  # Skip this frame to maintain processing rate
            
        self.last_process_time = current_time
        
        try:
            # Get image from message
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            original_height, original_width = cv_image.shape[:2]
            
            # Downscale image for faster processing
            if self.downscale_factor > 1:
                processed_image = cv2.resize(
                    cv_image, 
                    (original_width // self.downscale_factor, 
                     original_height // self.downscale_factor)
                )
            else:
                processed_image = cv_image.copy()
                
            # Process the image
            hand_raised, hand_coords, debug_image = self.detect_hand_and_position(
                processed_image, 
                original_width, 
                original_height
            )
            
            # Apply temporal filtering to reduce false positives/negatives
            if hand_raised:
                self.hand_raised_streak += 1
                self.hand_lowered_streak = 0
                if self.hand_raised_streak >= 2:  # Require 2 consecutive detections
                    self.is_hand_raised = True
                    self.last_hand_position = hand_coords
            else:
                self.hand_lowered_streak += 1
                self.hand_raised_streak = 0
                if self.hand_lowered_streak >= 3:  # Require 3 consecutive non-detections
                    self.is_hand_raised = False
                    
            # Publish hand raised status
            hand_raised_msg = Bool()
            hand_raised_msg.data = self.is_hand_raised
            self.hand_raised_pub.publish(hand_raised_msg)
            
            # If hand is consistently detected as raised, publish customer pose
            if self.is_hand_raised and self.last_hand_position is not None:
                customer_pose = self.calculate_customer_pose(
                    self.last_hand_position,
                    processed_image.shape[1],
                    processed_image.shape[0]
                )
                if customer_pose is not None:
                    self.customer_pose_pub.publish(customer_pose)
            
            # Publish debug image if needed
            if debug_image is not None:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_link')
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_hand_and_position(self, image, original_width, original_height):
        
        debug_image = image.copy()
        
        
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        
        results = self.hands.process(image_rgb)
        
        hand_raised = False
        hand_coords = None
        
        # Get key landmarks
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                middle_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
                pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
                
                
                vertical_distance = wrist.y - middle_finger_tip.y
                hand_span = abs(thumb_tip.x - pinky_tip.x)
                
                
                if vertical_distance > self.hand_raised_threshold and hand_span > 0.15:
                    hand_raised = True
                    
                    
                    h, w, _ = image.shape
                    px = int(wrist.x * w)
                    py = int(wrist.y * h)
                    hand_coords = (px, py)
                    
                   
                    mp.solutions.drawing_utils.draw_landmarks(
                        debug_image, 
                        hand_landmarks, 
                        self.mp_hands.HAND_CONNECTIONS
                    )
                    cv2.circle(debug_image, hand_coords, 10, (0, 255, 0), -1)
                    cv2.putText(
                        debug_image, 
                        f"HAND RAISED v_dist={vertical_distance:.2f}", 
                        (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (0, 255, 0), 
                        2
                    )
        
        if not hand_raised:
            cv2.putText(
                debug_image, 
                "NO HAND DETECTED", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.7, 
                (0, 0, 255), 
                2
            )
        
        return hand_raised, hand_coords, debug_image


def main(args=None):
    rclpy.init(args=args)
    node = HandDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()