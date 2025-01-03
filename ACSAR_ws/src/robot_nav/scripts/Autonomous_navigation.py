#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf_transformations
import math

class ServiceCoordinator(Node):
    def __init__(self):
        super().__init__('service_coordinator')
        
        # State variables
        self.is_recording = False
        self.is_navigating = False
        self.is_returning = False
        self.customer_pose = None
        
        # Initialize the customer service position
        self.initialize_customer_pose()
        
        # Create subscribers
        self.hand_raised_sub = self.create_subscription(
            Bool,
            '/hand_raised',
            self.hand_raised_callback,
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

    def initialize_customer_pose(self):
        """Initialize the predefined customer service position."""
        self.customer_pose = PoseStamped()
        self.customer_pose.header.frame_id = 'map'
        
        # Set the customer service position (modify these coordinates as needed)
        self.customer_pose.pose.position.x = 14.5153
        self.customer_pose.pose.position.y = 0.0657412
        self.customer_pose.pose.position.z = 0.0
        
        # Set orientation to face the customer (90 degrees)
        q = tf_transformations.quaternion_from_euler(0, 0, -0.013126)
        self.customer_pose.pose.orientation.x = q[0]
        self.customer_pose.pose.orientation.y = q[1]
        self.customer_pose.pose.orientation.z = q[2]
        self.customer_pose.pose.orientation.w = q[3]

    def hand_raised_callback(self, msg):
        """Handle hand raised detection."""
        if msg.data and not self.is_recording and not self.is_navigating and not self.is_returning:
            self.get_logger().info('Hand raised detected, navigating to customer')
            self.navigate_to_customer()

    def return_home_callback(self, msg):
        """Handle return home signal from speech recorder."""
        if msg.data and not self.is_returning:
            self.get_logger().info('Recording complete, returning to home position')
            self.is_recording = False
            self.is_returning = True
            self.navigate_to_home()

    def navigate_to_customer(self):
        """Navigate to the customer service position."""
        self.is_navigating = True
        goal_msg = NavigateToPose.Goal()
        
        # Update timestamp
        self.customer_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = self.customer_pose
        
        # Send navigation goal
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
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        home_pose.pose.orientation.x = q[0]
        home_pose.pose.orientation.y = q[1]
        home_pose.pose.orientation.z = q[2]
        home_pose.pose.orientation.w = q[3]
        
        goal_msg.pose = home_pose
        self.send_navigation_goal(goal_msg)

    def send_navigation_goal(self, goal_msg):
        """Send a navigation goal and handle the response."""
        # Wait for navigation server
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
        
        # Get the result future
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