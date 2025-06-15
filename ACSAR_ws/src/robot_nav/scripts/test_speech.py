#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import sounddevice as sd
import vosk
import json
import queue
import threading
import time
import os

class MicrophoneTestNode(Node):
    def __init__(self):
        super().__init__('microphone_test_node')
        
        # Initialize Vosk model
        model_path = "vosk-model-small-en-us"  # Replace with the path to your Vosk model
        if not os.path.exists(model_path):
            self.get_logger().error(f"Vosk model not found at {model_path}")
            raise FileNotFoundError(f"Vosk model not found at {model_path}")
        
        self.model = vosk.Model(model_path)
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
        
        # Audio queue for streaming audio data
        self.audio_queue = queue.Queue()
        
        # Event to stop the audio processing thread
        self.stop_event = threading.Event()
        
        # Start the audio processing thread
        self.audio_thread = threading.Thread(target=self.process_audio)
        self.audio_thread.start()
        
        # Start the microphone stream
        self.start_microphone_stream()
        
        # Publisher to publish recognized text
        self.text_publisher = self.create_publisher(String, 'recognized_text', 10)
        
        # Timer to periodically check for recognized text
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def start_microphone_stream(self):
        """Start streaming audio from the microphone."""
        def audio_callback(indata, frames, time, status):
            if status:
                self.get_logger().warn(f"Audio stream status: {status}")
            self.audio_queue.put(bytes(indata))
        
        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=audio_callback
        )
        self.stream.start()
    
    def process_audio(self):
        """Process audio data from the queue and recognize speech."""
        while not self.stop_event.is_set():
            try:
                data = self.audio_queue.get(timeout=1)
                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(self.recognizer.Result())
                    recognized_text = result.get('text', '')
                    if recognized_text:
                        self.get_logger().info(f"Recognized: {recognized_text}")
                        # Publish the recognized text
                        msg = String()
                        msg.data = recognized_text
                        self.text_publisher.publish(msg)
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error processing audio: {e}")
    
    def timer_callback(self):
        """Timer callback to check for recognized text."""
        pass  # The processing is done in the audio thread
    
    def destroy_node(self):
        """Cleanup on node destruction."""
        self.stop_event.set()
        self.audio_thread.join()
        self.stream.stop()
        self.stream.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()