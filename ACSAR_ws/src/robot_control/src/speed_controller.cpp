#include "robot_control/speed_controller.hpp"
#include <cstring>
using std::placeholders::_1;

SpeedController::SpeedController(const std::string & name) : Node(name),
    FR_wheel_prev_pos_(0.0),FL_wheel_prev_pos_(0.0),BR_wheel_prev_pos_(0.0),BL_wheel_prev_pos_(0.0)
{
    declare_parameter("wheel_radius", 0.07);
    declare_parameter("wheel_width", 0.35);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_width_ = get_parameter("wheel_width").as_double();
    prev_time_ = get_clock()->now();

    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/service_robot_controller/cmd_vel", 10,
        std::bind(&SpeedController::velCallback, this, _1));
    
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/four_wheel_base_controller/commands", 10);

    joint_state_pub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",10,std::bind(&SpeedController::jointCallback,this,_1));
    
    
        // For the equations:
        // x˙ = 4/r(ωFL + ωBL + ωFR + ωBR)
        // θ˙ = R/2W(ωFR + ωBR - ωFL - ωBL)
        
        // We need to solve for [ωFL, ωBL, ωFR, ωBR]
        // Given [x˙, θ˙]
        
        // The matrix A in the equation [x˙, θ˙]ᵀ = A[ωFL, ωBL, ωFR, ωBR]ᵀ is:
        // Initialize the matrices with fixed sizes
        forward_kinematics_matrix_ = Eigen::MatrixXd(2, 4);
        inverse_kinematics_matrix_ = Eigen::MatrixXd(4, 2);
        
        double r = wheel_radius_;
        double W = wheel_width_;
        
        forward_kinematics_matrix_ << 
            r/4,    r/4,    r/4,    r/4,      // x˙ equation
            -r/(2*W), -r/(2*W), r/(2*W),  r/(2*W);    // θ˙ equation
            
        // Calculate inverse kinematics matrix (pseudo-inverse since we have more outputs than inputs)
        // Calculate pseudo-inverse using SVD
         // Calculate the pseudo-inverse explicitly
    Eigen::Matrix2d temp = (forward_kinematics_matrix_ * forward_kinematics_matrix_.transpose());
    inverse_kinematics_matrix_ = forward_kinematics_matrix_.transpose() * temp.inverse();
    }
   

void SpeedController::velCallback(const geometry_msgs::msg::TwistStamped & msg)
{
    // Create velocity vector [x˙, θ˙]
        Eigen::Vector2d desired_velocities;
        desired_velocities << 
            msg.twist.linear.x,    // x˙
            msg.twist.angular.z;   // θ˙

        // Calculate wheel velocities using the inverse kinematics matrix
        Eigen::Vector4d wheel_velocities = inverse_kinematics_matrix_ * desired_velocities;

        // Create and publish message
        auto wheel_velocities_msg = std_msgs::msg::Float64MultiArray();

        wheel_velocities_msg.data = {
            wheel_velocities(0),  // ωFL
            wheel_velocities(1),  // ωBL
            wheel_velocities(2),  // ωFR
            wheel_velocities(3) };  // ωBR
    RCLCPP_INFO(this->get_logger(),"Wheel Velocities  %f  %f  %f  %f",wheel_velocities(0),wheel_velocities(1),wheel_velocities(2),wheel_velocities(3));
    wheel_cmd_pub_->publish(wheel_velocities_msg);
}

void SpeedController::jointCallback(const sensor_msgs::msg::JointState & msg)
{
    double dp_FL_wheel = msg.position.at(0)- FL_wheel_prev_pos_;
    double dp_BL_wheel = msg.position.at(1)- BL_wheel_prev_pos_;
    double dp_FR_wheel = msg.position.at(2)- FR_wheel_prev_pos_;
    double dp_BR_wheel = msg.position.at(3)- BR_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;
//RCLCPP_INFO(this->get_logger(),"msg_time = %f  prev_time_ = %f",msg_time,prev_time_);
    FL_wheel_prev_pos_ = msg.position.at(0);
    BL_wheel_prev_pos_ = msg.position.at(1);
    FR_wheel_prev_pos_ = msg.position.at(2);
    BR_wheel_prev_pos_ = msg.position.at(3);

    prev_time_ = msg_time;

    double delta_FL_wheel = dp_FL_wheel / dt.seconds();
    double delta_FR_wheel = dp_FR_wheel / dt.seconds();
    double delta_BL_wheel = dp_BL_wheel / dt.seconds();
    double delta_BR_wheel = dp_BR_wheel / dt.seconds();

    double robot_linear_velocity = (wheel_radius_  * (delta_FL_wheel + delta_FR_wheel + delta_BL_wheel + delta_BR_wheel)) / 4.0;
    double robot_angular_velocity = (wheel_radius_ * (delta_FR_wheel - delta_BR_wheel + delta_FL_wheel - delta_BL_wheel )) / (2.0 * wheel_width_);

    RCLCPP_INFO(this->get_logger(), "Linear: %f Angular: %f", robot_linear_velocity, robot_angular_velocity);
}   

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeedController>("speed_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}