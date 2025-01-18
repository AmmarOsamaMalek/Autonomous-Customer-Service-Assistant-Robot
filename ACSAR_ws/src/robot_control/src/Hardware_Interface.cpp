#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"  // Include this header
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

namespace robot_control
{

class SimpleHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(SimpleHardwareInterface)

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Check parameters
        if (info_.hardware_parameters.count("serial_port") == 0 || 
            info_.hardware_parameters.count("baud_rate") == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Missing required parameters 'serial_port' or 'baud_rate'");
            return CallbackReturn::ERROR;
        }

        // Get parameters
        port_ = info_.hardware_parameters["serial_port"];
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

        // Initialize the storage vectors with the correct size
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);

        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), 
                    "Initialized with port: %s, baud: %d", 
                    port_.c_str(), baud_rate_);
        
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), 
                    "Configured for %zu joints", info_.joints.size());

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        // Get joint names from hardware info
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            // Export position interface
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name, 
                    hardware_interface::HW_IF_POSITION,  // Use the constant
                    &hw_positions_[state_interfaces.size() / 2]));
                    
            // Export velocity interface
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name, 
                    hardware_interface::HW_IF_VELOCITY,  // Use the constant
                    &hw_velocities_[state_interfaces.size() / 2]));
                    
            RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Exported state interfaces for joint: %s", joint.name.c_str());
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        // Get joint names from hardware info
        for (const hardware_interface::ComponentInfo& joint : info_.joints) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint.name, 
                    hardware_interface::HW_IF_VELOCITY,  // Use the constant
                    &hw_commands_[command_interfaces.size()]));
                    
            RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Exported command interface for joint: %s", joint.name.c_str());
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override
    {
        // Open serial port
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Failed to open serial port: %s", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Failed to get serial attributes");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Set baud rate
        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        // 8N1 mode
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        
        // No flow control
        tty.c_cflag &= ~CRTSCTS;
        
        // Turn on READ & ignore control lines
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Raw mode
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        // Wait for 1 byte for up to 0.1 seconds
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Failed to set serial attributes");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Successfully activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Successfully deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        // Update position based on velocity for basic odometry
        double dt = period.seconds();

        for (size_t i = 0; i < hw_positions_.size(); ++i) {
            hw_positions_[i] += hw_velocities_[i] * dt;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        if (serial_fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        // Format for AVR: START_BYTE + 4 PWM values + END_BYTE
        uint8_t packet[6] = {0xFF, 0, 0, 0, 0, 0xFE};
        
        // Convert wheel velocities to PWM values (0-255)
        for (size_t i = 0; i < 4; ++i) {
            // Scale velocity to PWM range and add direction bit
            double scaled_vel = hw_commands_[i] * 127.0 / 0.5; // max velocity from URDF
            if (scaled_vel > 0) {
                packet[i + 1] = static_cast<uint8_t>(std::min(127.0, scaled_vel)) + 128;
            } else {
                packet[i + 1] = static_cast<uint8_t>(std::min(127.0, -scaled_vel));
            }
            // Update stored velocity for position estimation
            hw_velocities_[i] = hw_commands_[i];
        }

        // Send to AVR
        ssize_t bytes_written = ::write(serial_fd_, packet, 6);  // Using global scope operator
        if (bytes_written != 6) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Failed to write complete packet: %s", strerror(errno));
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    
    std::string port_;
    int serial_fd_ = -1;
    int baud_rate_;
};

} // namespace robot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robot_control::SimpleHardwareInterface,
    hardware_interface::SystemInterface)

