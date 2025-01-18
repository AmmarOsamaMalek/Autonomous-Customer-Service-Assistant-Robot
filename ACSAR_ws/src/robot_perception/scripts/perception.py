#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import mediapipe as mp
import tf2_ros
import tf2_geometry_msgs
from math import atan2, cos, sin, pi
from rclpy.duration import Duration

class EnhancedHandDetector(Node):
    def __init__(self):
        super().__init__('enhanced_hand_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands with optimized settings for Raspberry Pi
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
            model_complexity=0  
        )
        
        # Initialize TF2 buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera intrinsic parameters (update these based on your camera)
        self.camera_matrix = np.array([
            [460.0, 0, 320.0],  # fx, 0, cx
            [0, 460.0, 240.0],  # 0, fy, cy
            [0, 0, 1]
        ])
        
        # Subscribers
        self.create_subscription(
            Image,
            '/image_raw',  # Updated topic for RPi camera
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
        
        # Store latest lidar data and its timestamp
        self.latest_scan = None
        self.latest_scan_time = None
        
        self.get_logger().info('Enhanced hand detector initialized')

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
        """Get depth measurement from lidar at specific angle."""
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
        
        if 0 <= index < len(self.latest_scan.ranges):
            depth = self.latest_scan.ranges[index]
            if depth >= self.latest_scan.range_min and depth <= self.latest_scan.range_max:
                return depth
        return None

    def get_latest_transform_time(self):
        """Get the latest available transform time."""
        try:
            # Look up the latest transform
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                tf2_ros.Time(),  # This will get the latest transform
                timeout=Duration(seconds=1.0)
            )
            return transform.header.stamp
        except Exception as e:
            self.get_logger().warn(f'Failed to get latest transform time: {str(e)}')
            return None

    def calculate_customer_pose(self, hand_coords):
        """Calculate customer pose from hand coordinates and lidar data."""
        if self.latest_scan is None:
            return None
            
        # Convert pixel coordinates to angle
        px, py = hand_coords
        fx = self.camera_matrix[0, 0]
        cx = self.camera_matrix[0, 2]
        
        # Calculate angle from camera center to hand
        angle = atan2(px - cx, fx)
        
        # Get depth from lidar
        depth = self.get_depth_from_lidar(angle)
        if depth is None:
            return None
        
        # Get the latest available transform time
        latest_transform_time = self.get_latest_transform_time()
        if latest_transform_time is None:
            return None
        
        # Create customer pose
        pose = PoseStamped()
        pose.header.frame_id = 'camera_link'
        pose.header.stamp = latest_transform_time  # Use the transform time
        
        # Calculate 3D position
        pose.pose.position.x = depth * cos(angle)
        pose.pose.position.y = depth * sin(angle)
        pose.pose.position.z = 0.0
        
        # Set orientation to face the customer
        angle_to_customer = atan2(pose.pose.position.y, pose.pose.position.x)
        pose.pose.orientation = self.euler_to_quaternion(0, 0, angle_to_customer)
        
        try:
            # Transform pose to map frame using the same timestamp
            transformed_pose = self.tf_buffer.transform(
                pose, 
                'map',
                timeout=Duration(seconds=1.0)
            )
            return transformed_pose
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {str(e)}')
            return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hand_raised, hand_coords = self.detect_hand_and_position(cv_image)
            
            if hand_raised and hand_coords is not None:
                # Calculate 3D position and publish customer pose
                customer_pose = self.calculate_customer_pose(hand_coords)
                if customer_pose is not None:
                    self.customer_pose_pub.publish(customer_pose)
            
            # Publish hand raised status
            hand_raised_msg = Bool()
            hand_raised_msg.data = hand_raised
            self.hand_raised_pub.publish(hand_raised_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_hand_and_position(self, image):
        """Detect raised hand and return its position in image coordinates."""
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                middle_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                
                if (wrist.y - middle_finger_tip.y) > 0.3:  # Hand raised threshold
                    # Convert normalized coordinates to pixel coordinates
                    h, w, _ = image.shape
                    px = int(wrist.x * w)
                    py = int(wrist.y * h)
                    return True, (px, py)
        
        return False, None


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedHandDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()