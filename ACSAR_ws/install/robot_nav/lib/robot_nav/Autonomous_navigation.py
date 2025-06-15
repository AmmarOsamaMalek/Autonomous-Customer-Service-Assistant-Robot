#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from math import cos, sin, pi, atan2
from typing import Callable
from rclpy.duration import Duration

class ServiceCoordinator(Node):
    def __init__(self):
        super().__init__('service_coordinator')
        
        # Define states
        self.states = {
            'IDLE': 0,
            'NAVIGATING_TO_CUSTOMER': 1,
            'TAKING_ORDER': 2,
            'NAVIGATING_TO_KITCHEN': 3,
            'WAITING_IN_KITCHEN': 4,
            'DELIVERING_TO_CUSTOMER': 5,
            'WAITING_FOR_THANKS': 6,
            'RETURNING_HOME': 7
        }
        
        
        self.current_state = self.states['IDLE']
        self.current_customer_pose = None
        self.is_navigating = False
        
        
        self.kitchen_pose = self.create_kitchen_pose()
        self.home_pose = self.create_home_pose()
        
        
        self.hand_raised_sub = self.create_subscription(
            Bool,
            '/hand_raised',
            self.hand_raised_callback,
            10
        )
        
        self.customer_pose_sub = self.create_subscription(
            PoseStamped,
            '/customer_pose',
            self.customer_pose_callback,
            10
        )
        
        self.voice_command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        
        
        self.start_record_pub = self.create_publisher(
            String,
            '/start_record',
            10
        )
        
        # Add velocity publisher to stop robot sliding
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )
        
        # Add initial pose publisher for AMCL
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
       
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Publish initial pose to AMCL after a short delay
        self.create_timer(1.0, self.publish_initial_pose)
        self.init_pose_timer = self.create_timer(1.0, self.publish_initial_pose)
        
        self.get_logger().info('Service Coordinator initialized')
        self.print_state()
        self.init_pose_count = 0

    def print_state(self):
        
        state_name = [k for k, v in self.states.items() if v == self.current_state][0]
        self.get_logger().info(f'Current State: {state_name}')

    def publish_initial_pose(self):
        
        if self.init_pose_count >= 10:
            self.init_pose_timer.cancel()
            return
            
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set initial position (0,0,0)
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
        initial_pose.pose.pose.position.z = 0.0
        
        # Set initial orientation 
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.0
        initial_pose.pose.pose.orientation.w = 1.0
        
        # Set covariance
        covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        
        initial_pose.pose.covariance = covariance
        
        self.initial_pose_pub.publish(initial_pose)
        self.init_pose_count += 1
        self.get_logger().info(f'Published initial pose to AMCL ({self.init_pose_count}/10)')
    
    
    def stop_robot(self):
        """Publish zero velocity command to stop robot movement."""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        
        self.stop_count = 0
        
        def publish_stop():
            self.stop_count += 1
            self.cmd_vel_pub.publish(stop_cmd)
            if self.stop_count >= 10:  
                self.stop_timer.cancel()
        
       
        self.stop_timer = self.create_timer(0.1, publish_stop)
    
    def create_kitchen_pose(self) -> PoseStamped:
       
        kitchen = PoseStamped()
        kitchen.header.frame_id = 'map'
        kitchen.pose.position.x = 17.9397  
        kitchen.pose.position.y = 8.45744  
        kitchen.pose.position.z = 0.0
        kitchen.pose.orientation = self.euler_to_quaternion(0, 0, 0.164469)
        return kitchen

    def create_home_pose(self) -> PoseStamped:
        
        home = PoseStamped()
        home.header.frame_id = 'map'
        home.header.stamp = self.get_clock().now().to_msg()
        home.pose.position.x = 0.0
        home.pose.position.y = 0.0
        home.pose.position.z = 0.0
        home.pose.orientation = self.euler_to_quaternion(0, 0, 0.0)
        return home

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def quaternion_to_euler(self, quaternion: Quaternion) -> tuple:
        
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def calculate_approach_pose(self, target_pose: PoseStamped, distance: float) -> PoseStamped:
       
        approach_pose = PoseStamped()
        approach_pose.header = target_pose.header
        
        _, _, yaw = self.quaternion_to_euler(target_pose.pose.orientation)
        
        dx = distance * cos(yaw)
        dy = distance * sin(yaw)
        
        approach_pose.pose.position.x = target_pose.pose.position.x - dx
        approach_pose.pose.position.y = target_pose.pose.position.y - dy
        approach_pose.pose.position.z = 0.0
        approach_pose.pose.orientation = target_pose.pose.orientation
        
        return approach_pose

    def customer_pose_callback(self, msg: PoseStamped):
        
        self.current_customer_pose = msg
        
        if self.current_state == self.states['NAVIGATING_TO_CUSTOMER']:
            self.navigate_to_customer()

    def hand_raised_callback(self, msg: Bool):
        
        if (msg.data and 
            self.current_state == self.states['IDLE'] and 
            self.current_customer_pose is not None):
            
            self.get_logger().info('Hand raised detected, navigating to customer')
            self.current_state = self.states['NAVIGATING_TO_CUSTOMER']
            self.navigate_to_customer()
            self.print_state()
            
    def start_recording_with_delay(self):
        
        self.start_record_count = 0
        
        def delayed_start():
            self.start_record_count += 1
            if self.start_record_count == 5:  # After 0.5 seconds (5 * 0.1)
                msg = String()
                msg.data = 'start'
                self.start_record_pub.publish(msg)
                self.get_logger().info('Published start recording command')
                self.start_record_timer.cancel()
        
        self.start_record_timer = self.create_timer(0.1, delayed_start)

    def voice_command_callback(self, msg: String):
        
        command = msg.data.lower()
        
        if command == 'finish' and self.current_state == self.states['TAKING_ORDER']:
            self.get_logger().info('Order finished, navigating to kitchen')
            self.navigate_to_kitchen()
            
        elif command == 'go' and self.current_state == self.states['WAITING_IN_KITCHEN']:
            self.get_logger().info('Received go command, delivering to customer')
            self.deliver_to_customer()
            
        elif command == 'thank you' and self.current_state == self.states['WAITING_FOR_THANKS']:
            self.get_logger().info('Received thank you, returning home')
            self.return_home()
            
        self.print_state()

    def navigate_to_customer(self):
        
        if self.current_customer_pose is None:
            self.get_logger().error('Cannot navigate: customer position unknown')
            return
            
        self.get_logger().info('Navigating to customer')
        goal_msg = NavigateToPose.Goal()
        approach_pose = self.calculate_approach_pose(self.current_customer_pose, 0.3)
        goal_msg.pose = approach_pose
        self.send_navigation_goal(goal_msg, self.order_taking_callback)

    def navigate_to_kitchen(self):
        
        self.get_logger().info('Navigating to kitchen')
        self.current_state = self.states['NAVIGATING_TO_KITCHEN']
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.kitchen_pose
        self.send_navigation_goal(goal_msg, self.kitchen_arrival_callback)
        self.print_state()

    def deliver_to_customer(self):
        
        if self.current_customer_pose is None:
            self.get_logger().error('Cannot deliver: customer position lost')
            return
            
        self.current_state = self.states['DELIVERING_TO_CUSTOMER']
        goal_msg = NavigateToPose.Goal()
        approach_pose = self.calculate_approach_pose(self.current_customer_pose, 0.3)
        goal_msg.pose = approach_pose
        self.send_navigation_goal(goal_msg, self.delivery_complete_callback)

    def return_home(self):
       
        self.current_state = self.states['RETURNING_HOME']
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.home_pose
        self.send_navigation_goal(goal_msg, self.home_arrival_callback)

    def send_navigation_goal(self, goal_msg: NavigateToPose.Goal, 
                           completion_callback: Callable[[bool], None]):
       
        self.is_navigating = True
        
        # Wait for navigation server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            self.is_navigating = False
            completion_callback(False)
            return

        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        
       
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, completion_callback))

    def goal_response_callback(self, future, completion_callback: Callable[[bool], None]):
       
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.is_navigating = False
            completion_callback(False)
            return

        self.get_logger().info('Navigation goal accepted')
        
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.goal_result_callback(future, completion_callback))

    def goal_result_callback(self, future, completion_callback: Callable[[bool], None]):
        
        result = future.result().result
        status = future.result().status
        success = status == 4  
        
        if success:
            self.get_logger().info('Navigation successful')
           
            self.stop_robot()
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            
        self.is_navigating = False
        completion_callback(success)

   
    def order_taking_callback(self, success: bool):
        
        if success:
            self.current_state = self.states['TAKING_ORDER']
            self.start_recording_with_delay()
            self.print_state() 
            
            
    def kitchen_arrival_callback(self, success: bool):
        
        if success:
            self.current_state = self.states['WAITING_IN_KITCHEN']
            self.get_logger().info('Waiting for "go" command in kitchen')
            self.start_recording_with_delay()
            self.print_state()


    def delivery_complete_callback(self, success: bool):
        
        if success:
            self.current_state = self.states['WAITING_FOR_THANKS']
            self.get_logger().info('Waiting for "thank you" from customer')
            self.start_recording_with_delay()
            self.print_state()

    def home_arrival_callback(self, success: bool):
        
        if success:
            self.current_state = self.states['IDLE']
            self.current_customer_pose = None
            self.get_logger().info('Ready for next customer')
            self.print_state()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServiceCoordinator()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()