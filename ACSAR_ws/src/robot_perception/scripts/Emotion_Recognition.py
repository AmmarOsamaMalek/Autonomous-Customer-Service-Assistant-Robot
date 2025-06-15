#!/usr/bin/env python3

"""
Emotion REcognition Node:
============================
A Ros2 Node that performs real-time emotion recognition on images captured from a camera.
It uses a pre-trained Haar cascade for face detection and also a pre-trained deep learning model
for emotion calssification.
============================
This node is a part of a larger robot perception system that includes
hand detection and other perception tasks.

Author: AMmar Osama Malek
"""

import os 
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from tensorflow.keras.models import load_model # type: ignore
from ament_index_python.packages import get_package_share_directory


class EmotionRecognition(Node):
    
    EMOTION_LABELS_LOOKUP_TABLE = [
        "Anger",
        "Disgust",
        "Fear",
        "Happy",
        "Sad",
        "Surprise",
        "Neutral"
    ]
    
    def __init__(self):
        
        super().__init__("EmotionRecognition_node")
        
        
        default_model_path = "/home/ammar/Robotics/Graduation_Project/ACSAR_ws/models/emotion_model.h5"
        
        self.declare_parameter('cascade_path','')
        self.declare_parameter('model_path',default_model_path)
        self.declare_parameter('min_face_size',30)
        self.declare_parameter('min_confidence',0.5)
        self.declare_parameter('publish_rate',30.0)
        self.declare_parameter('display',False)
        
        
        self.cascade_path = self.get_parameter('cascade_path').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.min_face_size = self.get_parameter('min_face_size').get_parameter_value().integer_value
        self.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
        self.display = self.get_parameter('display').get_parameter_value().bool_value 
        
        if not self.cascade_path:
            self.cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            
        
        self._init_face_detector()
        self._init_emotion_classifier()
        
        self.bridge = CvBridge()
        
        self.last_face = None
        self.missed_frames = 0
        self.max_missed = 30
        
        self.images_pub = self.create_publisher(Image,'/emotion_detection/image',10)
        self.emotion_pub = self.create_publisher(String,'/emotion_detection/emotion',10)
        
        self.image_sub = self.create_subscription(Image,'/image_raw',self.image_callback,10)
        
        self.get_logger().info("Emotion Recognition Node INitialized")
        
    
    def _init_face_detector(self):
        
        if not os.path.exists(self.cascade_path):
            self.get_logger().error("Haar cascade file not found")
            rclpy.shutdown()
            return
        
        try:
            self.face_cascade = cv2.CascadeClassifier(self.cascade_path)
            if self.face_cascade.empty():
                self.get_logger().error("Failed to load Haar cascade classifier")
                rclpy.shutdown()
                return
            
            self.get_logger().info("Haar cascade classifier loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Error loading Haar cascade classifier: {e}")
            rclpy.shutdown()
            
            
    def _init_emotion_classifier(self):
        
        if not os.path.exists(self.model_path):
            self.get_logger().error("Model file not found")
            rclpy.shutdown()
            return
        
        try:
            self.model = load_model(self.model_path)
            self.get_logger().info("Emotion classification model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Error loading emotion classification model: {e}")
            rclpy.shutdown()
            
    def detect_face(self,frame):
        
        try:
        
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(self.min_face_size,self.min_face_size),
            )
            
            if len(faces) == 0:
                
                if self.last_face and self.missed_frames < self.max_missed:
                    self.missed_frames += 1
                    return self.last_face
                else:
                    self.last_face = None
                    return None
            
            best_face = None
            largest_area = 0
            
            for (x,y,w,h) in faces:
                area = w * h
                if area > largest_area:
                    largest_area = area
                    best_face = (x,y,w,h)
                    
            self.last_face = best_face
            self.missed_frames = 0
            return best_face
        except Exception as e:
            self.get_logger().error(f"Error in face detection: {e}")
            return None
        
    def predict_emotion(self,face_image):
        
        try:
            
            face_image = cv2.resize(face_image,(48,48))
            face_image = face_image/255.0
            face_image = face_image.reshape(1,48,48,1)  # Fixed: Changed from (1,48,48,3) to (1,48,48,1)
            
            prediction = self.model.predict(face_image,verbose=0)
            emotion_index = np.argmax(prediction[0])
            confidence = prediction[0][emotion_index]
            
            return self.EMOTION_LABELS_LOOKUP_TABLE[emotion_index],confidence
        except Exception as e:
            self.get_logger().error(f"Error in emotion prediction: {e}")
            return None, 0.0
        
    def process_frame(self, frame):
        
        processed_frame = frame.copy()
        emotion = None
        confidence = 0.0
        
        face = self.detect_face(frame)
        
        if face:
            x,y,w,h = face
            gray_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            face_image = gray_frame[y:y+h,x:x+w]
            
            if face_image.size > 0:
                
                emotion, confidence = self. predict_emotion(face_image)
                cv2.rectangle(processed_frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.putText(processed_frame,f"{emotion} ({confidence:.2f})",(x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,255,0),2)
                
        return processed_frame, emotion, confidence
    
    def image_callback(self,msg):
        
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            processed_frame, emotion, confidence = self.process_frame(cv_image)
            
            processed_msg = self.bridge.cv2_to_imgmsg(processed_frame,'bgr8')
            processed_msg.header = msg.header
            self.images_pub.publish(processed_msg)
            
            if emotion is not None:
                
                emotion_msg = String()
                emotion_msg.data = emotion
                self.emotion_pub.publish(emotion_msg)
                
            if self.display:
                
                try:
                    cv2.imshow("Emotion Recognition",processed_frame)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().error(f"Error displaying image: {e}")
                    cv2.destroyAllWindows()
                    self.display = False
                    
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")
            

def main(args=None):
    
    rclpy.init(args=args)
    node = EmotionRecognition()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Emotion Recognition Node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        node.get_logger().info("Emotion Recognition Node destroyed")                
        
if __name__ == "__main__":
    main()