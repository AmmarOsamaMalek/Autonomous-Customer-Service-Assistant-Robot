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
         
        self.declare_parameter('device', '1') 
        self.declare_parameter('sample_rate', 48000)  
        self.device = self.get_parameter('device').value
        self.sample_rate = self.get_parameter('sample_rate').value
        
        
        model_path = "vosk-model-small-en-us"
        model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(model, self.sample_rate)
        
        self.q = queue.Queue()
        
       
        self.start_sub = self.create_subscription(String, '/start_record', 
                                                self.start_recording_callback, 10)
        self.voice_command_pub = self.create_publisher(String, '/voice_command', 10)
        self.customer_order_pub = self.create_publisher(String, '/customer_order', 10)
        self.delivery_status_pub = self.create_publisher(String, '/delivery_status', 10)
        
        
        self.recording = False
        self.last_speech_time = time.time()
        self.speech_timeout = 20.0
        self.recorded_text = []
        self.stop_event = Event()
        
        self.current_order = {
            "items": [],
            "total": 0.0,
            "status": "Pending"
        }
           
        self.menu_items = {
            "pizza": 12.99,
            "burger": 8.99,
            "salad": 7.99,
            "pasta": 10.99,
            "coffee": 3.99,
            "tea": 2.99,
            "juice": 4.99,
            "water": 1.99
        }
        
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
            with sd.RawInputStream(
                samplerate=self.sample_rate,  
                channels=1,
                dtype='int16',
                callback=self.audio_callback,
                device=self.device  
            ):
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
                                self.process_speech(text)
                                
        except Exception as e:
            self.get_logger().error(f'Error in recording: {str(e)}')
            self.stop_event.set()

    def process_speech(self, text):

        words = text.split()
        quantities = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5}
        quantity = 1
        
        for i, word in enumerate(words):
            if word.isdigit():
                quantity = int(word)
            elif word in quantities:
                quantity = quantities[word]
            
            for item_name in self.menu_items:
                if item_name in word:
                    item_price = self.menu_items[item_name]
                    item_exists = False
                    for order_item in self.current_order["items"]:
                        if order_item["name"].lower() == item_name:
                            order_item["quantity"] += quantity
                            item_exists = True
                            break
                    if not item_exists:
                        self.current_order["items"].append({
                            "name": item_name.capitalize(),
                            "quantity": quantity,
                            "price": item_price
                        })
                    self.current_order["total"] += quantity * item_price
                    self.publish_order()
                    quantity = 1
                    break
        
        
        if 'finish' in text:
            self.stop_recording()
            self.current_order["status"] = "Preparing"
            self.publish_order()
            self.publish_delivery_status("Preparing")
            msg = String()
            msg.data = 'finish'
            self.voice_command_pub.publish(msg)
        elif 'go' in text:
            msg = String()
            msg.data = 'go'
            self.voice_command_pub.publish(msg)
        elif 'thank you' in text:
            self.current_order["status"] = "Delivered"
            self.publish_order()
            self.publish_delivery_status("Delivered")
            msg = String()
            msg.data = 'thank you'
            self.voice_command_pub.publish(msg)

    def publish_order(self):
        msg = String()
        msg.data = json.dumps(self.current_order)
        self.customer_order_pub.publish(msg)
        self.get_logger().info(f'Published order: {msg.data}')

    def publish_delivery_status(self, status):
        msg = String()
        msg.data = status
        self.delivery_status_pub.publish(msg)
        self.get_logger().info(f'Published delivery status: {status}')

    def save_order(self):
        if self.recorded_text:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            
            
            recording_file = os.path.join(self.recordings_dir, f'recording_{timestamp}.txt')
            with open(recording_file, 'w') as f:
                f.write('\n'.join(self.recorded_text))
            
            
            order_file = os.path.join(self.orders_dir, f'order_{timestamp}.txt')
            with open(order_file, 'w') as f:
                f.write(f"Order Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("Customer Order:\n")
                f.write('\n'.join(self.recorded_text))
                f.write("\n\nItems:\n")
                for item in self.current_order["items"]:
                    f.write(f"{item['quantity']} x {item['name']} - ${item['price'] * item['quantity']:.2f}\n")
                f.write(f"\nTotal: ${self.current_order['total']:.2f}\n")
                f.write(f"Status: {self.current_order['status']}\n")

    def start_recording_callback(self, msg):
        self.get_logger().info(f'Received start recording command: {msg.data}')
        if not self.recording and msg.data == 'start':
            self.get_logger().info('Starting recording...')
            self.recording = True
            self.recorded_text = []
            self.stop_event.clear()
            self.last_speech_time = time.time()
            self.current_order = {
                "items": [],
                "total": 0.0,
                "status": "Pending"
            }
            self.publish_order()
            self.publish_delivery_status("Taking Order")
            
            
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