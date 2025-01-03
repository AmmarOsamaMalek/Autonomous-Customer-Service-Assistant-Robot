#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sounddevice as sd
import vosk
import json
import queue
import time
import os  # Added missing import
from threading import Thread, Event

class SpeechRecorder(Node):
    def __init__(self):
        super().__init__('speech_recorder')
        
        # Initialize vosk model
        model_path = os.path.expanduser("~/Robotics/Graduation_Project/ACSAR_ws/vosk-model-small-en-us")
        model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(model, 16000)
        
        # Audio settings
        self.samplerate = 16000
        self.q = queue.Queue()
        
        # Create publishers and subscribers
        self.start_sub = self.create_subscription(
            String,
            '/start_record',
            self.start_recording_callback,
            10
        )
        
        self.return_pub = self.create_publisher(
            Bool,
            '/return_home',
            10
        )
        
        # Initialize variables
        self.recording = False
        self.last_speech_time = time.time()
        self.speech_timeout = 20.0
        self.recorded_text = []
        self.stop_event = Event()
        
        # Create directories for recordings and orders
        self.base_dir = os.path.expanduser('~/customer_data')
        self.recordings_dir = os.path.join(self.base_dir, 'recordings')
        self.orders_dir = os.path.join(self.base_dir, 'orders')
        os.makedirs(self.recordings_dir, exist_ok=True)
        os.makedirs(self.orders_dir, exist_ok=True)


    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        self.q.put(bytes(indata))

    def save_order(self):
        """Save the recorded text as a customer order."""
        if self.recorded_text:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            
            # Save full recording
            recording_file = os.path.join(self.recordings_dir, f'recording_{timestamp}.txt')
            with open(recording_file, 'w') as f:
                f.write('\n'.join(self.recorded_text))
            
            # Save order with timestamp
            order_file = os.path.join(self.orders_dir, f'order_{timestamp}.txt')
            with open(order_file, 'w') as f:
                f.write(f"Order Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("Customer Order:\n")
                f.write('\n'.join(self.recorded_text))
                f.write("\n\nStatus: Pending\n")
            
            self.get_logger().info(f'Saved order to {order_file}')


    def start_recording_callback(self, msg):
        if not self.recording:
            self.get_logger().info('Starting recording session')
            self.recording = True
            self.recorded_text = []
            self.stop_event.clear()
            self.last_speech_time = time.time()  # Reset the time when starting recording
            
            # Start recording thread
            Thread(target=self.record_audio).start()
            # Start timeout checker thread
            Thread(target=self.check_timeout).start()

    def record_audio(self):
        try:
            with sd.RawInputStream(samplerate=self.samplerate, 
                                 channels=1,
                                 dtype='int16',
                                 callback=self.audio_callback):
                
                while self.recording and not self.stop_event.is_set():
                    data = self.q.get()
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        if result['text']:
                            text = result['text']
                            self.get_logger().info(f'Recognized: {text}')
                            self.last_speech_time = time.time()
                            self.recorded_text.append(text)
                            
                            if 'finish' in text.lower():
                                self.stop_recording()
                                break
                                
        except Exception as e:
            self.get_logger().error(f'Error in recording: {str(e)}')
            self.stop_recording()

    def check_timeout(self):
        while self.recording and not self.stop_event.is_set():
            current_time = time.time()
            time_since_last_speech = current_time - self.last_speech_time
            
            self.get_logger().debug(f'Time since last speech: {time_since_last_speech:.1f} seconds')
            
            if time_since_last_speech > self.speech_timeout:
                self.get_logger().info(f'Speech timeout reached after {time_since_last_speech:.1f} seconds')
                self.stop_recording()
                break
            time.sleep(1.0)

    def stop_recording(self):
        """Stop recording and save the order."""
        if self.recording:
            self.recording = False
            self.stop_event.set()
            
            # Save the order
            self.save_order()
            
            # Wait a brief moment before sending return signal
            time.sleep(1.0)
            
            # Publish return home signal
            msg = Bool()
            msg.data = True
            self.return_pub.publish(msg)
            self.get_logger().info('Published return home signal')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()