#include "robot_control/noisy_controller.hpp"
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <random>
using std::placeholders::_1;

NoisyController::NoisyController(const std::string & name) : Node(name),
    FR_wheel_prev_pos_(0.0),FL_wheel_prev_pos_(0.0),
    BR_wheel_prev_pos_(0.0),BL_wheel_prev_pos_(0.0),
    x_pos_(0.0),y_pos_(0.0),theta_(0.0)
{
    declare_parameter("wheel_radius", 0.07);
    declare_parameter("wheel_width", 0.35);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_width_ = get_parameter("wheel_width").as_double();
    prev_time_ = get_clock()->now();


    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",10,std::bind(&NoisyController::jointCallback,this,_1));

    odm_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "service_robot/odm_noisy",10);
    
    
    odm_msg_.header.frame_id = "odm";
    odm_msg_.child_frame_id = "base_link_ekf";
    odm_msg_.pose.pose.orientation.x = 0.0;
    odm_msg_.pose.pose.orientation.y = 0.0;
    odm_msg_.pose.pose.orientation.z = 0.0;
    odm_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamp_.header.frame_id = "odm";
    transform_stamp_.child_frame_id = "base_link_noisy";
}

   
void NoisyController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> left_encoders_noise(0.0,0.005);
    std::normal_distribution<double> right_encoders_noise(0.0,0.005);
    double wheel_encoder_front_left = msg.position.at(0) + left_encoders_noise(noise_generator);
    double wheel_encoder_back_left = msg.position.at(1) + left_encoders_noise(noise_generator);
    double wheel_encoder_front_right = msg.position.at(2) + right_encoders_noise(noise_generator);
    double wheel_encoder_back_right = msg.position.at(3) + right_encoders_noise(noise_generator);

    double dp_FL_wheel = wheel_encoder_front_left- FL_wheel_prev_pos_;
    double dp_BL_wheel = wheel_encoder_back_left- BL_wheel_prev_pos_;
    double dp_FR_wheel = wheel_encoder_front_right- FR_wheel_prev_pos_;
    double dp_BR_wheel = wheel_encoder_back_right- BR_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    FL_wheel_prev_pos_ = msg.position.at(0);
    BL_wheel_prev_pos_ = msg.position.at(1);
    FR_wheel_prev_pos_ = msg.position.at(2);
    BR_wheel_prev_pos_ = msg.position.at(3);

    prev_time_ = msg_time;

    double delta_FL_wheel = dp_FL_wheel / dt.seconds();
    double delta_FR_wheel = dp_FR_wheel / dt.seconds();
    double delta_BL_wheel = dp_BL_wheel / dt.seconds();
    double delta_BR_wheel = dp_BR_wheel / dt.seconds();

    double robot_linear_velocity = (wheel_radius_  * (delta_FL_wheel + delta_FR_wheel + delta_BL_wheel + delta_BR_wheel)) / 2.0;
    double robot_angular_velocity = (wheel_radius_ * (delta_BR_wheel - delta_FR_wheel + delta_BL_wheel - delta_FL_wheel )) / (wheel_width_);

    double d_s = (wheel_radius_  * (dp_FL_wheel + dp_FR_wheel + dp_BL_wheel + dp_BR_wheel)) / 2.0;
    double d_theta = (wheel_radius_ * ( dp_BR_wheel - dp_FR_wheel + dp_BL_wheel - dp_FL_wheel )) / ( wheel_width_);

    theta_ += d_theta;
    x_pos_ += d_s * cos(theta_);
    y_pos_ += d_s * sin(theta_);

    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    odm_msg_.pose.pose.orientation.x = q.x();
    odm_msg_.pose.pose.orientation.y = q.y();
    odm_msg_.pose.pose.orientation.z = q.z();
    odm_msg_.pose.pose.orientation.w = q.w();
    odm_msg_.header.stamp = get_clock()->now();
    odm_msg_.pose.pose.position.x = x_pos_;
    odm_msg_.pose.pose.position.y = y_pos_;
    odm_msg_.twist.twist.linear.x = robot_linear_velocity;
    odm_msg_.twist.twist.angular.z = robot_angular_velocity;

    transform_stamp_.transform.translation.x = x_pos_;
    transform_stamp_.transform.translation.y = y_pos_;
    transform_stamp_.transform.rotation.x = q.x();
    transform_stamp_.transform.rotation.y = q.y();
    transform_stamp_.transform.rotation.z = q.z();
    transform_stamp_.transform.rotation.w = q.w();
    transform_stamp_.header.stamp = get_clock()->now();

    odm_pub_->publish(odm_msg_);
    transform_broadcaster_->sendTransform(transform_stamp_);

}   

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NoisyController>("noisy_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}