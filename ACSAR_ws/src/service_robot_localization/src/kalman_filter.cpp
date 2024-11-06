#include "service_robot_localization/kalman_filter.hpp"

using std::placeholders::_1;

KalmanFilter::KalmanFilter(const std::string & name) : 
    Node(name),mean_(0.0),variance_(1000.0),
    imu_angular_z_(0.0),is_first_odm_(true),
    last_angular_z_(0.0),motion_(0.0),
    motion_variance_(4.0),measurement_variance_(0.5)
{
   odm_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "service_robot/odm",10,std::bind(&KalmanFilter::odmCallBack,this,_1));

   imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu/data",10,std::bind(&KalmanFilter::imuCallBack,this,_1)); 

   odm_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        "service_robot/odm_kalman",10);
}

void KalmanFilter::odmCallBack(const nav_msgs::msg::Odometry & odm_msg)
{
    kalman_odm_ = odm_msg;

    if(is_first_odm_)
    {
        mean_ = odm_msg.twist.twist.angular.z;
        last_angular_z_ = odm_msg.twist.twist.angular.z;;
        is_first_odm_ = false;
    }

    motion_ = odm_msg.twist.twist.angular.z - last_angular_z_;

    measurement_Update();
    state_Prediction();

    kalman_odm_.twist.twist.angular.z = mean_;
    odm_pub_->publish(kalman_odm_);

}
void KalmanFilter::imuCallBack(const sensor_msgs::msg::Imu & imu_msg)
{
    imu_angular_z_ = imu_msg.angular_velocity.z;
}

void KalmanFilter::measurement_Update(void)
{
    mean_ = ((measurement_variance_* mean_) + (imu_angular_z_*variance_))/(variance_+measurement_variance_);
    variance_ = (variance_*measurement_variance_)/(variance_+measurement_variance_);
}

void KalmanFilter::state_Prediction(void)
{
    mean_ = mean_ + motion_;
    variance_ = variance_ + motion_variance_;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}