#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
import numpy as np
import mediapipe as mp

class HandDetectorCV(Node):
    def __init__(self):
        super().__init__('hand_detector_cv')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        self.hand_raised_pub = self.create_publisher(
            Bool,
            '/hand_raised',
            10
        )
        
        # Parameters
        self.raised_hand_threshold = 0.3  # Threshold for considering hand as raised
        
        self.get_logger().info('Hand detector node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hand_raised = self.detect_raised_hand(cv_image)
            
            # Publish result
            msg = Bool()
            msg.data = hand_raised
            self.hand_raised_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_raised_hand(self, image):
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process the image and detect hands
        results = self.hands.process(image_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Get wrist and middle finger tip landmarks
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                middle_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
                
                # Check if hand is raised (middle finger higher than wrist)
                if (wrist.y - middle_finger_tip.y) > self.raised_hand_threshold:
                    return True
                    
                # Draw landmarks for visualization
                self.draw_landmarks(image, hand_landmarks)
        
        return False

    def draw_landmarks(self, image, hand_landmarks):
        # Draw the hand landmarks
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            self.mp_hands.HAND_CONNECTIONS
        )
        
        # Display the image (for debugging)
        cv2.imshow('Hand Detection', image)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorCV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()