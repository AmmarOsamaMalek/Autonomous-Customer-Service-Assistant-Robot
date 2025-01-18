#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sounddevice as sd
import vosk
import json
import queue
import time
import os
from threading import Thread, Event

class SpeechRecorder(Node):
    def __init__(self):
        super().__init__('speech_recorder')
        
        # Initialize vosk model
        model_path = os.path.expanduser("~/Robotics/Graduation_Project/ACSAR_ws/vosk-model-small-en-us")
        model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(model, 16000)
        
        self.samplerate = 16000
        self.q = queue.Queue()
        
        # Publishers and subscribers
        self.start_sub = self.create_subscription(String, '/start_record', 
                                                self.start_recording_callback, 10)
        self.voice_command_pub = self.create_publisher(String, '/voice_command', 10)
        
        # Initialize variables
        self.recording = False
        self.last_speech_time = time.time()
        self.speech_timeout = 20.0
        self.recorded_text = []
        self.stop_event = Event()
        
        # Create directories
        self.setup_directories()

    def setup_directories(self):
        self.base_dir = os.path.expanduser('~/customer_data')
        self.recordings_dir = os.path.join(self.base_dir, 'recordings')
        self.orders_dir = os.path.join(self.base_dir, 'orders')
        os.makedirs(self.recordings_dir, exist_ok=True)
        os.makedirs(self.orders_dir, exist_ok=True)

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        self.q.put(bytes(indata))

    def record_audio(self):
        try:
            with sd.RawInputStream(samplerate=self.samplerate, channels=1,
                                 dtype='int16', callback=self.audio_callback):
                while not self.stop_event.is_set():
                    data = self.q.get()
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        if result['text']:
                            text = result['text'].lower()
                            self.get_logger().info(f'Recognized: {text}')
                            self.last_speech_time = time.time()
                            
                            if self.recording:
                                self.recorded_text.append(text)
                                if 'finish' in text:
                                    self.stop_recording()
                                    msg = String()
                                    msg.data = 'finish'
                                    self.voice_command_pub.publish(msg)
                                elif 'go' in text:  
                                    msg = String()
                                    msg.data = 'go'
                                    self.voice_command_pub.publish(msg)
                                elif 'thank you' in text:  
                                    msg = String()
                                    msg.data = 'thank you'
                                    self.voice_command_pub.publish(msg)
                                
        except Exception as e:
            self.get_logger().error(f'Error in recording: {str(e)}')
            self.stop_event.set()


    def save_order(self):
        if self.recorded_text:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            
            # Save full recording
            recording_file = os.path.join(self.recordings_dir, f'recording_{timestamp}.txt')
            with open(recording_file, 'w') as f:
                f.write('\n'.join(self.recorded_text))
            
            # Save order
            order_file = os.path.join(self.orders_dir, f'order_{timestamp}.txt')
            with open(order_file, 'w') as f:
                f.write(f"Order Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("Customer Order:\n")
                f.write('\n'.join(self.recorded_text))
                f.write("\n\nStatus: Pending\n")

    def start_recording_callback(self, msg):
        self.get_logger().info(f'Received start recording command: {msg.data}')
        if not self.recording and msg.data == 'start':
            self.get_logger().info('Starting recording...')
            self.recording = True
            self.recorded_text = []
            self.stop_event.clear()
            self.last_speech_time = time.time()
            
            # Start recording threads
            recording_thread = Thread(target=self.record_audio)
            timeout_thread = Thread(target=self.check_timeout)
            
            recording_thread.start()
            timeout_thread.start()
            
            self.get_logger().info('Recording threads started')

    def check_timeout(self):
        while not self.stop_event.is_set():
            if time.time() - self.last_speech_time > self.speech_timeout:
                self.stop_recording()
                break
            time.sleep(1.0)

    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.save_order()
            self.stop_event.set()

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()