#include <robot_control/Hardware_Interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace ACSAR_firmware
{
    ACSARInterface::ACSARInterface(){}
    
    ACSARInterface::~ACSARInterface()
    {
        if (AVR_.IsOpen())
        {
            try
            {
                AVR_.Close();
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Failed to close serial port: %s", e.what());
            }
        }
    }

    hardware_interface::CallbackReturn ACSARInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        hardware_interface::CallbackReturn result = hardware_interface::SystemInterface::on_init(info);
        if (result != hardware_interface::CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_name_ = info_.hardware_parameters.at("serial_port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Port name not found in hardware parameters");
            return hardware_interface::CallbackReturn::ERROR;
        }
        try
        {
            std::string baud_str = info_.hardware_parameters.at("baud_rate");
            baud_rate_ = std::stoi(baud_str);
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Baud rate not found in hardware parameters");
            baud_rate_ = 9600;
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Invalid baud rate value: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        
        try {
            odom_frame_ = info_.hardware_parameters.at("odom_frame");
        } catch (const std::out_of_range &e) {
            odom_frame_ = "odom";
        }

        try {
            base_frame_ = info_.hardware_parameters.at("base_frame");
        } catch (const std::out_of_range &e) {
            base_frame_ = "base_link";
        }

        try {
            lidar_frame_ = info_.hardware_parameters.at("lidar_frame");
        } catch (const std::out_of_range &e) {
            lidar_frame_ = "lidar_link";
        }


        try {
            wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
        } catch (const std::out_of_range &e) {
            wheel_radius_ = 0.05; 
        }

        try {
            wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
        } catch (const std::out_of_range &e) {
            wheel_separation_ = 0.3; 
        }

        // Get LiDAR mounting position
        try {
            lidar_x_offset_ = std::stod(info_.hardware_parameters.at("lidar_x_offset"));
        } catch (const std::out_of_range &e) {
            lidar_x_offset_ = -0.15; 
        }

        try {
            lidar_y_offset_ = std::stod(info_.hardware_parameters.at("lidar_y_offset"));
        } catch (const std::out_of_range &e) {
            lidar_y_offset_ = 0.0; 
        }

        try {
            lidar_z_offset_ = std::stod(info_.hardware_parameters.at("lidar_z_offset"));
        } catch (const std::out_of_range &e) {
            lidar_z_offset_ = 0.37; // 
        }

        velocity_commands_.resize(info_.joints.size(), 0.0);
        position_states_.resize(info_.joints.size(), 0.0);   
        velocity_states_.resize(info_.joints.size(), 0.0);

        last_read_time_ = rclcpp::Clock().now();

        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "ACSARInterface initialized with port: %s, baud rate: %d", port_name_.c_str(), baud_rate_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ACSARInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "position", &position_states_[i]));
            
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "velocity", &velocity_states_[i]));         
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ACSARInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "velocity", &velocity_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn ACSARInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Activating Robot Hardware Interface");

        std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
        std::fill(position_states_.begin(), position_states_.end(), 0.0);   
        std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        try
        {
            AVR_.Open(port_name_);

            switch (baud_rate_)
            {
              case 9600:
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
                break;
              case 19200:
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_19200);
                break;
              case 38400:
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
                break;
              case 57600:
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
                break;
              case 115200:
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
                break;
              default:
                RCLCPP_WARN(rclcpp::get_logger("ACSARInterface"), 
                          "Unsupported baud rate: %d, using 9600 instead", baud_rate_);
                AVR_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
                break;
            }

            AVR_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            AVR_.SetParity(LibSerial::Parity::PARITY_NONE);
            AVR_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            AVR_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

            AVR_.FlushIOBuffers();
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Failed to open serial port: %s", e.what());
            return hardware_interface::CallbackReturn::FAILURE;
        }
        
        node_ = std::make_shared<rclcpp::Node>("acsar_hw_interface");
        
        odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() {
            while (rclcpp::ok()) {
                executor_->spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        
        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Serial port opened successfully, Ready to take commands");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ACSARInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Deactivating Robot Hardware Interface");
        std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
        
        write(rclcpp::Time(0), rclcpp::Duration(std::chrono::nanoseconds(0)));

        if (AVR_.IsOpen())
        {
            try
            {
                AVR_.Close();
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Failed to close serial port: %s", e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        
       
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        
        executor_->remove_node(node_);
        odom_publisher_.reset();
        tf_broadcaster_.reset();
        node_.reset();

       RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Serial port closed successfully, Hardware Stopped");
       return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ACSARInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    if (AVR_.IsDataAvailable())
    {
        try
        {
            auto dt = (rclcpp::Clock().now() - last_read_time_).seconds();
            std::string message;
            AVR_.ReadLine(message);

            // Clean up message by removing carriage returns and newlines
            message.erase(std::remove(message.begin(), message.end(), '\r'), message.end());
            message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());

            if (message.empty())
            {
                return hardware_interface::return_type::OK;
            }

            if (!message.empty() && message.back() == ',') {
                message.pop_back();
            }

            RCLCPP_DEBUG(rclcpp::get_logger("ACSARInterface"), "Received message: %s", message.c_str());

            std::stringstream ss(message);
            std::string part;

            while (std::getline(ss, part, ','))
            {
                if (part.length() < 3)
                {
                    RCLCPP_WARN(rclcpp::get_logger("ACSARInterface"), "Skipping too short part: %s", part.c_str());
                    continue;
                }

                char motor_id = part.at(0);
                char direction = part.at(1);

                int sign = (direction == 'p') ? 1 : -1;

                try {
                   
                    std::string value_str = part.substr(2);
                   
                    value_str.erase(0, value_str.find_first_not_of(" \t"));
                    value_str.erase(value_str.find_last_not_of(" \t") + 1);
                    
                    if (value_str.empty()) {
                        RCLCPP_WARN(rclcpp::get_logger("ACSARInterface"), "Empty value for motor %c", motor_id);
                        continue;
                    }
                    
                    double velocity = sign * std::stod(value_str);

                    switch (motor_id)
                    {
                        case 'a': // front right wheel
                            if (0 < info_.joints.size())
                            {
                                velocity_states_[0] = velocity;
                                position_states_[0] += velocity * dt;  
                            }
                            break;

                        case 'b': // Front left wheel
                            if (1 < info_.joints.size())
                            {
                                velocity_states_[1] = velocity;
                                position_states_[1] += velocity_states_[1] * dt;
                            }
                            break;
                            
                        case 'c':  // Rear left wheel
                            if (2 < info_.joints.size())
                            {
                                velocity_states_[2] = velocity;
                                position_states_[2] += velocity_states_[2] * dt;
                            }
                            break;
                            
                        case 'd':  // Rear right wheel
                            if (3 < info_.joints.size())
                            {
                                velocity_states_[3] = velocity;
                                position_states_[3] += velocity_states_[3] * dt;
                            }
                            break;

                        default:
                            RCLCPP_WARN(rclcpp::get_logger("ACSARInterface"), "Invalid motor ID: %c", motor_id);
                            break;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(rclcpp::get_logger("ACSARInterface"), 
                        "Failed to parse velocity for motor %c: %s (value: %s)", 
                        motor_id, e.what(), part.substr(2).c_str());
                    continue; 
                }
            }

            last_read_time_ = rclcpp::Clock().now();
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), 
                "Failed to read from serial port: %s", e.what());
            // Don't return error here - try to continue operation
        }
    }
    
    // Calculate and publish odometry based on wheel velocities
    publishOdometry(time, period);
    
    // Reduce the log level to DEBUG for routine operations
    RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Velocity states: [%f, %f, %f, %f]",
        velocity_states_[0], velocity_states_[1], velocity_states_[2], velocity_states_[3]);
    RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Position states: [%f, %f, %f, %f]",
        position_states_[0], position_states_[1], position_states_[2], position_states_[3]);
        
    return hardware_interface::return_type::OK;
}


 hardware_interface::return_type ACSARInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    std::stringstream command_stream;

    // Format is: a[p/n]XX.XX,b[p/n]XX.XX,c[p/n]XX.XX,d[p/n]XX.XX
    const std::array<char, 4> motor_ids = {'a', 'b', 'c', 'd'};

    for (size_t i = 0; i < info_.joints.size() && i < 4; i++)
    {
        char direction = (velocity_commands_[i] >= 0) ? 'p' : 'n';
        command_stream << motor_ids[i] << direction;
        command_stream << std::fixed << std::setprecision(2);

        if (std::abs(velocity_commands_[i]) < 10.0)
        {
            command_stream << "0";
        }

        command_stream << std::abs(velocity_commands_[i]);
        command_stream << ",";
    }

    try
    {
        std::string message = command_stream.str();
        message += "\r\n";
        RCLCPP_DEBUG(rclcpp::get_logger("ACSARInterface"), "Sending: %s", message.c_str());
        AVR_.Write(message);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ACSARInterface"), "Error writing to serial port: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("hardware_interface"), "Velocity commands: [%f, %f, %f, %f]",
        velocity_commands_[0], velocity_commands_[1], velocity_commands_[2], velocity_commands_[3]);

    return hardware_interface::return_type::OK;
}
   

 
    void ACSARInterface::publishOdometry(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Calculate linear and angular velocities using differential drive kinematics
        // For a 4-wheeled robot (differential drive with independent wheels)
        double front_right_velocity = velocity_states_[0] * wheel_radius_;
        double front_left_velocity = velocity_states_[1] * wheel_radius_;
        double rear_left_velocity = velocity_states_[2] * wheel_radius_;
        double rear_right_velocity = velocity_states_[3] * wheel_radius_;
        
        // Calculate overall left and right side velocities
        double left_wheel_velocity = (front_left_velocity + rear_left_velocity) / 2.0;
        double right_wheel_velocity = (front_right_velocity + rear_right_velocity) / 2.0;
        
        double linear_vel_x = (left_wheel_velocity + right_wheel_velocity) / 2.0;
        double angular_vel_z = (right_wheel_velocity - left_wheel_velocity) / wheel_separation_;
        
        // Calculate position change
        double dt = period.seconds();
        double delta_x = linear_vel_x * std::cos(theta_) * dt;
        double delta_y = linear_vel_x * std::sin(theta_) * dt;
        double delta_theta = angular_vel_z * dt;
        
        // Update position
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;
        
        // Normalize theta
        while (theta_ > M_PI) theta_ -= 2 * M_PI;
        while (theta_ < -M_PI) theta_ += 2 * M_PI;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        
        // Publish transform from odom to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = odom_frame_;
        transform_stamped.child_frame_id = base_frame_;
        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform_stamped);
        
        // Publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        // Set position
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // Set velocity
        odom_msg.twist.twist.linear.x = linear_vel_x;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_vel_z;
        
        // Set covariance
        // position uncertainty
        odom_msg.pose.covariance[0] = 0.01; 
        odom_msg.pose.covariance[7] = 0.01;  
        odom_msg.pose.covariance[35] = 0.01; 
        
        //velocity uncertainty
        odom_msg.twist.covariance[0] = 0.01;  
        odom_msg.twist.covariance[35] = 0.01; 
        
       
        odom_publisher_->publish(odom_msg);
        
        
        geometry_msgs::msg::TransformStamped lidar_transform;
        lidar_transform.header.stamp = time;
        lidar_transform.header.frame_id = base_frame_;
        lidar_transform.child_frame_id = lidar_frame_;
        
        lidar_transform.transform.translation.x = lidar_x_offset_;
        lidar_transform.transform.translation.y = lidar_y_offset_;
        lidar_transform.transform.translation.z = lidar_z_offset_;
        
        lidar_transform.transform.rotation.x = 0.0;
        lidar_transform.transform.rotation.y = 0.0;
        lidar_transform.transform.rotation.z = 0.0;
        lidar_transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(lidar_transform);
        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "FL: %.2f FR: %.2f RL: %.2f RR: %.2f", 
            front_left_velocity, front_right_velocity, rear_left_velocity, rear_right_velocity);
        RCLCPP_INFO(rclcpp::get_logger("ACSARInterface"), "Left avg: %.2f, Right avg: %.2f", 
            left_wheel_velocity, right_wheel_velocity);

    }
}

PLUGINLIB_EXPORT_CLASS(ACSAR_firmware::ACSARInterface, hardware_interface::SystemInterface)