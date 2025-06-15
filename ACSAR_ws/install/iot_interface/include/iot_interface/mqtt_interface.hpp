#ifndef MQTT_INTERFACE_HPP
#define MQTT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <MQTTClient.h>
#include <mqtt/client.h>
#include <mqtt/exception.h>
#include <mqtt/async_client.h>
#include <string>

class MQTTInterface : public rclcpp::Node   
{
    public:
        MQTTInterface();
        ~MQTTInterface();
    
    private:
         void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
         void customer_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
         void timer_callback();

         void mqtt_connect();
         void mqtt_publish(const std::string &topic, const std::string &message);
         
         rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
         rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr customer_subscriber_;
         rclcpp::TimerBase::SharedPtr timer_;

         double x_,y_,theta_;
         std::string order_id_ = "777";
         bool delivered_ = false;   
         bool customer_detected_ = false;

         MQTTClient mqtt_client_;

};
#endif 