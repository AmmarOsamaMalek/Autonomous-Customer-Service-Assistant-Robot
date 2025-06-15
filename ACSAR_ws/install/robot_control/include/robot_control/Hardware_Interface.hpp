#ifndef HARDWARE_INTERFACE_HPP
#define HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <algorithm>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <thread>
#include <libserial/SerialPort.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>

namespace ACSAR_firmware
{
    class ACSARInterface : public hardware_interface::SystemInterface
    {
    public:
        ACSARInterface();
        ~ACSARInterface();

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        
        void publishOdometry(const rclcpp::Time &time, const rclcpp::Duration &period);

        LibSerial::SerialPort AVR_;
        std::string port_name_;
        int baud_rate_;
        rclcpp::Time last_read_time_;

        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        
        double wheel_radius_ = 0.1;  
        double wheel_separation_ = 0.5;  
        
        // Odometry state
        double x_ = 0.0;
        double y_ = 0.0;
        double theta_ = 0.0;
        
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
        std::thread executor_thread_;
        
        std::string odom_frame_ = "odom";
        std::string base_frame_ = "base_link";
        std::string lidar_frame_ = "lidar_link";
        
        double lidar_x_offset_ = 0.1;  // meters
        double lidar_y_offset_ = 0.0;  // meters
        double lidar_z_offset_ = 0.2;  // meters
    };
}


#endif // SIMPLE_HARDWARE_INTERFACE_HPP