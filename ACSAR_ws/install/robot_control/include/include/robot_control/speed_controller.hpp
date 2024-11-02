#ifndef SPEED_CONTROLLER_HPP
#define SPEED_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Core>
#include <Eigen/LU> 

class SpeedController : public rclcpp::Node
{
public:
    SpeedController(const std::string & name);

private:
    void velCallback(const geometry_msgs::msg::TwistStamped & msg);
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    double wheel_radius_;
    double wheel_width_;
    Eigen::MatrixXd forward_kinematics_matrix_;
    Eigen::MatrixXd inverse_kinematics_matrix_;

    double FR_wheel_prev_pos_;
    double FL_wheel_prev_pos_;
    double BR_wheel_prev_pos_;
    double BL_wheel_prev_pos_;

    rclcpp::Time prev_time_;
};

#endif
