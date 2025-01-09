#include "robot_control/Hardware_Interface.hpp"
#include <unistd.h>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cstring>

// Constants matching AVR implementation
const uint8_t START_BYTE = 0xFF;
const uint8_t END_BYTE = 0xFE;
const uint8_t PACKET_SIZE = 6;  // START + 4 motors + END
const double MAX_VELOCITY = 10.0;  // Maximum velocity in rad/s

WheelVelocityBridge::WheelVelocityBridge()
: Node("wheel_velocity_bridge"),
  serial_fd_(-1),
  wheel_indices({
    {"front_left_wheel", 0},
    {"front_right_wheel", 1},
    {"back_left_wheel", 2},
    {"back_right_wheel", 3}
  })
{
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();

    RCLCPP_INFO(this->get_logger(), "Initializing wheel velocity bridge...");
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);

    if (!initialize_serial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port");
        return;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&WheelVelocityBridge::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Successfully subscribed to /joint_states");
    RCLCPP_INFO(this->get_logger(), "Wheel Velocity Bridge Node is ready!");
}

WheelVelocityBridge::~WheelVelocityBridge() {
    if (serial_fd_ >= 0) {
        // Send stop command before closing
        std::vector<uint8_t> stop_packet(PACKET_SIZE, 128);
        stop_packet[0] = START_BYTE;
        stop_packet[PACKET_SIZE-1] = END_BYTE;
        send_packet(stop_packet);
        
        RCLCPP_INFO(this->get_logger(), "Closing serial port");
        close(serial_fd_);
    }
}

bool WheelVelocityBridge::initialize_serial() {
    RCLCPP_INFO(this->get_logger(), "Opening serial port...");
    
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get serial attributes: %s", strerror(errno));
        return false;
    }

    // Set baudrate
    speed_t baud;
    switch (baudrate_) {
        case 9600: baud = B9600; break;
        case 115200: baud = B115200; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d", baudrate_);
            return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // Match AVR UART settings
    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver and set local mode
    tty.c_cflag &= ~PARENB;             // No parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8 data bits
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode
    tty.c_oflag &= ~OPOST;              // Raw output
    tty.c_cc[VMIN] = 0;                 // Non-blocking read
    tty.c_cc[VTIME] = 1;                // 0.1 second read timeout

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set serial attributes: %s", strerror(errno));
        return false;
    }

    // Flush any existing data
    tcflush(serial_fd_, TCIOFLUSH);

    RCLCPP_INFO(this->get_logger(), "Serial port successfully configured");
    return true;
}

uint8_t WheelVelocityBridge::convert_velocity_to_command(double velocity) {
    // Convert rad/s to motor command (0-255)
    // 128 is stop
    // 129-255 is forward (increasing speed)
    // 0-127 is reverse (decreasing speed)
    double normalized_vel = std::clamp(velocity / MAX_VELOCITY, -1.0, 1.0);
    
    uint8_t command;
    if (normalized_vel >= 0) {
        command = static_cast<uint8_t>(128 + (normalized_vel * 127));
    } else {
        command = static_cast<uint8_t>(128 - (std::abs(normalized_vel) * 127));
    }

    RCLCPP_DEBUG(this->get_logger(), 
                 "Velocity: %.3f rad/s, Normalized: %.3f, Command: %d", 
                 velocity, normalized_vel, command);
    return command;
}

bool WheelVelocityBridge::send_packet(const std::vector<uint8_t>& packet) {
    if (packet.size() != PACKET_SIZE) {
        RCLCPP_ERROR(this->get_logger(), "Invalid packet size: %zu (expected %d)", 
                     packet.size(), PACKET_SIZE);
        return false;
    }

    std::stringstream hex_stream;
    hex_stream << std::hex << std::setfill('0');
    for (uint8_t byte : packet) {
        hex_stream << "0x" << std::setw(2) << static_cast<int>(byte) << " ";
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Sending packet: %s", hex_stream.str().c_str());

    ssize_t bytes_written = write(serial_fd_, packet.data(), packet.size());
    if (bytes_written != static_cast<ssize_t>(packet.size())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write complete packet: %s", strerror(errno));
        return false;
    }
    
    // Small delay to ensure packet is sent completely
    usleep(1000);  // 1ms delay
    
    RCLCPP_DEBUG(this->get_logger(), "Successfully wrote %zd bytes", bytes_written);
    return true;
}

void WheelVelocityBridge::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Create packet structure: [START_BYTE, M1, M2, M3, M4, END_BYTE]
    std::vector<uint8_t> packet(PACKET_SIZE, 128);  // Initialize with stop commands
    packet[0] = START_BYTE;
    packet[PACKET_SIZE-1] = END_BYTE;
    
    // Extract velocities based on joint names
    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = wheel_indices.find(msg->name[i]);
        if (it != wheel_indices.end() && i < msg->velocity.size()) {
            packet[it->second + 1] = convert_velocity_to_command(msg->velocity[i]);
        }
    }

    // Log motor commands
    RCLCPP_INFO(this->get_logger(), 
                "Motor commands - FL: %d, FR: %d, BL: %d, BR: %d",
                packet[1], packet[2], packet[3], packet[4]);
    
    // Send packet
    if (!send_packet(packet)) {
        RCLCPP_WARN(this->get_logger(), "Failed to send packet");
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelVelocityBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}