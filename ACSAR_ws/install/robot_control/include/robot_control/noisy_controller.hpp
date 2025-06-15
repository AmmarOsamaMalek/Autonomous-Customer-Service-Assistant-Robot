#ifndef NOISY_CONTROLLER_HPP
#define NOISY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>


class NoisyController : public rclcpp::Node
{
public:
    NoisyController(const std::string & name);

private:
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odm_pub_;

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    // Robot parameters
    double wheel_radius_;
    double wheel_width_;

    //double max_velocity_linear_;
    //double max_velocity_angular_;

    // Previous wheel positions
    double FR_wheel_prev_pos_;
    double FL_wheel_prev_pos_;
    double BR_wheel_prev_pos_;
    double BL_wheel_prev_pos_;

    // Robot pose
    double x_pos_;
    double y_pos_;
    double theta_;

    // Timing
    rclcpp::Time prev_time_;


    // Messages
    nav_msgs::msg::Odometry odm_msg_;
    geometry_msgs::msg::TransformStamped transform_stamp_;
};

#endif
