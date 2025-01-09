#ifndef HARDWARE_INTERFACE_HPP_
#define HARDWARE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <termios.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <map>

class WheelVelocityBridge : public rclcpp::Node {
public:
    WheelVelocityBridge();
    virtual ~WheelVelocityBridge();

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    bool initialize_serial();
    bool send_packet(const std::vector<uint8_t>& packet);
    uint8_t convert_velocity_to_command(double velocity);
    double normalize_velocity(double velocity);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    int serial_fd_;
    const uint8_t START_BYTE = 0xFF;
    const uint8_t END_BYTE = 0xFE;
    std::string serial_port_;
    int baudrate_;
    
    // Maximum expected velocity in rad/s for normalization
    const double MAX_VELOCITY = 10.0;  
    
    // Mapping for wheel indices
    const std::map<std::string, int> wheel_indices = {
        {"wheel_front_left_joint", 0},
        {"wheel_front_right_joint", 1},
        {"wheel_back_left_joint", 2},
        {"wheel_back_right_joint", 3}
    };
};

#endif // WHEEL_VELOCITY_BRIDGE_HPP_