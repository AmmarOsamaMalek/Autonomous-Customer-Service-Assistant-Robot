#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from math import cos, sin, pi, atan2

class ServiceCoordinator(Node):
    def __init__(self):
        super().__init__('service_coordinator')
        
        # State variables
        self.is_recording = False
        self.is_navigating = False
        self.is_returning = False
        self.current_customer_pose = None
        
        # Create subscribers
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
        
        self.return_home_sub = self.create_subscription(
            Bool,
            '/return_home',
            self.return_home_callback,
            10
        )
        
        # Create publishers
        self.start_record_pub = self.create_publisher(
            String,
            '/start_record',
            10
        )
        
        # Create Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info('Service Coordinator initialized')
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion."""
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def quaternion_to_euler(self, quaternion):
        """Convert quaternion to euler angles."""
        # Extract the values from quaternion
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

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

    def customer_pose_callback(self, msg):
        """Handle updates to customer position."""
        self.current_customer_pose = msg
        
        # If we're already navigating to a customer, update the goal
        if self.is_navigating and not self.is_returning:
            self.navigate_to_customer()

    def hand_raised_callback(self, msg):
        """Handle hand raised detection."""
        if msg.data and not self.is_recording and not self.is_navigating and not self.is_returning:
            if self.current_customer_pose is not None:
                self.get_logger().info('Hand raised detected, navigating to customer')
                self.navigate_to_customer()
            else:
                self.get_logger().warn('Hand raised detected but customer position unknown')

    def return_home_callback(self, msg):
        """Handle return home signal from speech recorder."""
        if msg.data and not self.is_returning:
            self.get_logger().info('Recording complete, returning to home position')
            self.is_recording = False
            self.is_returning = True
            self.navigate_to_home()

    def navigate_to_customer(self):
        """Navigate to the detected customer position."""
        if self.current_customer_pose is None:
            self.get_logger().error('Cannot navigate: customer position unknown')
            return
            
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        
        # Create approach pose (slightly before the customer)
        approach_pose = PoseStamped()
        approach_pose.header = self.current_customer_pose.header
        
        # Calculate position 1 meter before the customer
        distance = 0.3 # meters
        _, _, yaw = self.quaternion_to_euler(self.current_customer_pose.pose.orientation)
        
        dx = distance * cos(yaw)
        dy = distance * sin(yaw)
        
        approach_pose.pose.position.x = self.current_customer_pose.pose.position.x - dx
        approach_pose.pose.position.y = self.current_customer_pose.pose.position.y - dy
        approach_pose.pose.position.z = 0.0
        approach_pose.pose.orientation = self.current_customer_pose.pose.orientation
        
        goal_msg.pose = approach_pose
        self.send_navigation_goal(goal_msg)

    def navigate_to_home(self):
        """Navigate back to home position."""
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        
        # Create home position
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'map'
        home_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set home position coordinates
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.position.z = 0.0
        
        # Set orientation (facing forward)
        home_pose.pose.orientation = self.euler_to_quaternion(0, 0, 0)
        
        goal_msg.pose = home_pose
        self.send_navigation_goal(goal_msg)

    def send_navigation_goal(self, goal_msg):
        """Send a navigation goal and handle the response."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            self.is_navigating = False
            return

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation successful')
            if self.is_returning:
                self.is_returning = False
                self.is_navigating = False
                self.get_logger().info('Returned to home position, ready for new customers')
            elif not self.is_recording:
                self.start_recording()
        else:
            self.get_logger().error('Navigation failed')
        
        self.is_navigating = False

    def start_recording(self):
        """Start the speech recording process."""
        self.is_recording = True
        msg = String()
        msg.data = 'start'
        self.start_record_pub.publish(msg)
        self.get_logger().info('Started recording')

def main(args=None):
    rclpy.init(args=args)
    node = ServiceCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()