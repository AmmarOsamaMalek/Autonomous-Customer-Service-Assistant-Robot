#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <geometry_msgs/msg/twist.hpp>

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

    /**
     * Initializes the hardware interface for a differential drive robot.
     * 
     * Sets up serial communication parameters (port, baud rate) and wheel specs from the provided HardwareInfo.
     * Configures encoder resolution (11 PPR, 87:1 gear ratio, 3828 counts/rev) and initializes storage for joint
     * positions, velocities, and commands, assuming 2 encoder channels for 4 joints.
     * returns SUCCESS on proper initialization, or ERROR if parameters are missing or invalid.
     */

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }

        // Check parameters
        if (info_.hardware_parameters.count("serial_port") == 0 || 
            info_.hardware_parameters.count("baud_rate") == 0 ||
            info_.hardware_parameters.count("wheel_diameter") == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), 
                        "Missing required parameters");
            return CallbackReturn::ERROR;
        }

        // Get parameters
        port_ = info_.hardware_parameters["serial_port"];
        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);

        // Fixed hardware specifications
        pulses_per_rev_ = 11;  // Motor encoder PPR
        gear_ratio_ = 87.0;    // 1:87 gear ratio
        
        // Calculate total counts for one wheel revolution
        // PPR * 4 (quadrature) * gear ratio
        counts_per_wheel_rev_ = pulses_per_rev_ * 4 * gear_ratio_;

        // Initialize the storage vectors
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);
        
        // Initialize encoder counts
        last_encoder_counts_.resize(4, 0);
        current_encoder_counts_.resize(4, 0);
        last_read_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), 
                    "Initialized with port: %s, baud: %d", 
                    port_.c_str(), baud_rate_);
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"),
                    "Encoder configuration: PPR: %d, Gear ratio: 1:%.1f, Counts per wheel rev: %.1f",
                    pulses_per_rev_, gear_ratio_, counts_per_wheel_rev_);

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
                    hardware_interface::HW_IF_VELOCITY,  
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


    int Write_Serial(const unsigned char* buf, int len)
    {
        return ::write(serial_fd_,const_cast<unsigned char*>(buf),len);
    }

    int Read_Serial(unsigned char* buf,int len)
    {
        auto start = std::chrono::steady_clock::now();
        ssize_t bytes_read = 0;
        while(bytes_read < len)
        {
            ssize_t res = ::read(serial_fd_,&buf[bytes_read],1);
            if(res < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"),"Failed to read from serial port: %s",strerror(errno));
                return res;
            }
            
            bytes_read += res;
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(now-start).count() > 3000)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"),"Timeout while reading from serial port");
                return -1;
            }

        }
        return bytes_read;
    }

    /**
     * Reads encoder data from the hardware interface.
     * 
     * Reads raw encoder counts from the hardware via the serial port and calculates the joint positions and velocities
     * for all four motors. The function also handles invalid packets, calculates the time difference between reads, 
     * and updates the internal state of the hardware interface. 
     * 
     * @param time The current time.
     * @param period The time period since the last read.
     * @return hardware_interface::return_type::OK on success, or hardware_interface::return_type::ERROR on failure.
     */

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), "Serial port not open");
            return hardware_interface::return_type::ERROR;
        }
    
        // Read encoder packet (18 bytes: 0xFF + 4 encoders * 4 bytes + 0xFE)
        uint8_t packet[18];
        ssize_t bytes_read = 0;
        bool valid_packet = false;
    
        while (bytes_read < 18 && !valid_packet) {
            uint8_t byte;
            ssize_t result = ::read(serial_fd_, &byte, 1);
            if (result > 0) {
                if (bytes_read == 0 && byte == 0xFF) {
                    packet[bytes_read++] = byte;
                } else if (bytes_read > 0) {
                    packet[bytes_read++] = byte;
                }
                if (bytes_read == 18) {
                    valid_packet = (packet[0] == 0xFF && packet[17] == 0xFE);
                }
            } else if (result < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("SimpleHardwareInterface"), "Serial read failed: %s", strerror(errno));
                return hardware_interface::return_type::ERROR;
            }
        }
    
        if (!valid_packet) {
            RCLCPP_WARN(rclcpp::get_logger("SimpleHardwareInterface"), "Invalid encoder packet");
            return hardware_interface::return_type::OK;
        }
    
        // Parse 4 encoder counts
        int32_t counts[4];
        counts[0] = (packet[1] << 24) | (packet[2] << 16) | (packet[3] << 8) | packet[4];   // FL
        counts[1] = (packet[5] << 24) | (packet[6] << 16) | (packet[7] << 8) | packet[8];   // BL
        counts[2] = (packet[9] << 24) | (packet[10] << 16) | (packet[11] << 8) | packet[12]; // FR
        counts[3] = (packet[13] << 24) | (packet[14] << 16) | (packet[15] << 8) | packet[16]; // BR
    
        for (size_t i = 0; i < 4; ++i) {
            current_encoder_counts_[i] = counts[i];
        }
    
        // Calculate time difference
        auto current_time = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(current_time - last_read_time_).count();
        last_read_time_ = current_time;
        if (dt <= 0 || dt > 1.0) {
            dt = period.seconds();
        }
    
        // Update positions and velocities for 4 joints
        for (size_t i = 0; i < 4; ++i) {
            // Position: θ = (2π * counts) / counts_per_wheel_rev
            hw_positions_[i] = (2.0 * M_PI * current_encoder_counts_[i]) / counts_per_wheel_rev_;
    
            // Velocity: ω = (2π * Δcounts) / (counts_per_wheel_rev * Δt)
            double count_diff = current_encoder_counts_[i] - last_encoder_counts_[i];
            hw_velocities_[i] = (2.0 * M_PI * count_diff) / (counts_per_wheel_rev_ * dt);
    
            last_encoder_counts_[i] = current_encoder_counts_[i];
        }
    
        // Log all 4 joints
        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                    "FL pos: %.2f rad, vel: %.2f rad/s", hw_positions_[0], hw_velocities_[0]);
        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                    "BL pos: %.2f rad, vel: %.2f rad/s", hw_positions_[1], hw_velocities_[1]);
        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                    "FR pos: %.2f rad, vel: %.2f rad/s", hw_positions_[2], hw_velocities_[2]);
        RCLCPP_INFO(rclcpp::get_logger("arduino_actuator_interface"), 
                    "BR pos: %.2f rad, vel: %.2f rad/s", hw_positions_[3], hw_velocities_[3]);
    
        return hardware_interface::return_type::OK;
    }


    /**
     * Writes velocity commands to the hardware interface.
     * 
     * Retrieves the velocity commands for all four motors, determines the direction for each motor (forward or reverse),
     * and constructs a command string. The command string is then sent to the hardware via the serial port.
     * The function also logs the commands for debugging purposes and throttles the data transfer to avoid overwhelming the Arduino.
     * 
     * @param time The current time.
     * @param period The time period since the last write.
     * @return hardware_interface::return_type::OK on success, or hardware_interface::return_type::ERROR on failure.
     */

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override
    {
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Writing");

        try {
          // Get the velocity commands for all 4 motors
          float rpmFront_left = static_cast<float>(hw_commands_.at(0));  // Front left
          int dirFront_left = (rpmFront_left >= 0) ? 0 : 1;  // 0 for forward, 1 for reverse
      
          float rpmBack_left = static_cast<float>(hw_commands_.at(1));  // Back left
          int dirBack_left = (rpmBack_left >= 0) ? 0 : 1;  
      
          float rpmFront_right = static_cast<float>(hw_commands_.at(2));  // Front right
          int dirFront_right = (rpmFront_right >= 0) ? 0 : 1;  
      
          float rpmBack_right = static_cast<float>(hw_commands_.at(3));  // Back right
          int dirBack_right = (rpmBack_right >= 0) ? 0 : 1;  
      
          // Create a string with the command data for all 4 motors
          std::string data = std::to_string(rpmFront_left) + " " + std::to_string(dirFront_left) + " " +
                             std::to_string(rpmBack_left) + " " + std::to_string(dirBack_left) + " " +
                             std::to_string(rpmFront_right) + " " + std::to_string(dirFront_right) + " " +
                             std::to_string(rpmBack_right) + " " + std::to_string(dirBack_right) + "\n";
      
          // Write the command data to the serial port
          Write_Serial(reinterpret_cast<const unsigned char*>(data.c_str()), data.length());
          RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Writing %s", data.c_str());
      
          // Throttle the data transfer to avoid overwhelming the Arduino
          std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }
        catch (const std::exception& e) {
          RCLCPP_FATAL(rclcpp::get_logger("SimpleHardwareInterface"), "Error: %s", e.what());
          return hardware_interface::return_type::ERROR;
        }
      
        // Log the commands for all 4 joints for debugging purposes
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Front left joint command: %.2f, Direction: %d", 
                    hw_commands_.at(0), (hw_commands_.at(0) >= 0) ? 0 : 1);
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Back left joint command: %.2f, Direction: %d", 
                    hw_commands_.at(1), (hw_commands_.at(1) >= 0) ? 0 : 1);
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Front right joint command: %.2f, Direction: %d", 
                    hw_commands_.at(2), (hw_commands_.at(2) >= 0) ? 0 : 1);
        RCLCPP_INFO(rclcpp::get_logger("SimpleHardwareInterface"), "Back right joint command: %.2f, Direction: %d", 
                    hw_commands_.at(3), (hw_commands_.at(3) >= 0) ? 0 : 1);
      
        return hardware_interface::return_type::OK;
    }

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    
    std::string port_;
    int serial_fd_ = -1;
    int baud_rate_;

    // Encoder-related members
    std::vector<int32_t> current_encoder_counts_;
    std::vector<int32_t> last_encoder_counts_;
    std::chrono::steady_clock::time_point last_read_time_;
    int pulses_per_rev_;          // Raw encoder PPR (11)
    double gear_ratio_;           // Gear reduction ratio (87:1)
    double counts_per_wheel_rev_; // Total encoder counts per wheel revolution
    double wheel_diameter_;
};

} // namespace robot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    robot_control::SimpleHardwareInterface,
    hardware_interface::SystemInterface)

