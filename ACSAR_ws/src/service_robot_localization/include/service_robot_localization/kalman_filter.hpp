#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>


class KalmanFilter : public rclcpp::Node 
{

public:
    KalmanFilter(const std::string & name);

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odm_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odm_pub_;
    
    // Odometry guassian distribution
    double mean_;
    double variance_;

    double imu_angular_z_;
    bool is_first_odm_;
    double last_angular_z_;
    double motion_;

    double motion_variance_;
    double measurement_variance_;

    nav_msgs::msg::Odometry kalman_odm_;

    void odmCallBack(const nav_msgs::msg::Odometry & odm_msg);
    void imuCallBack(const sensor_msgs::msg::Imu & imu_msg);
    void measurement_Update(void);
    void state_Prediction(void);



};

#endif