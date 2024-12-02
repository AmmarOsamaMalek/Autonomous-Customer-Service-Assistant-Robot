#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <queue>
#include <mutex>

class CircleNavigator : public rclcpp::Node 
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    CircleNavigator() : Node("circle_navigator"), is_action_server_ready_(false)
    {
        // Declare parameters
        this->declare_parameter("goal_offset", 0.5);
        this->declare_parameter("base_frame", std::string("base_link"));
        this->declare_parameter("map_frame", std::string("odom"));
        this->declare_parameter("max_retry_attempts", 5);

        // Get parameter values
        goal_offset_ = this->get_parameter("goal_offset").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        max_retry_attempts_ = this->get_parameter("max_retry_attempts").as_int();

        // Create action client for navigation
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "/navigate_to_pose"
        );

        // Subscribe to circle position topic
        circle_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "circle_position_with_depth", 10,
            std::bind(&CircleNavigator::circlePositionCallback, this, std::placeholders::_1)
        );

        // Timer to check action server and process queued goals
        action_server_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            std::bind(&CircleNavigator::checkActionServerAndProcessQueue, this)
        );

        RCLCPP_INFO(this->get_logger(), "Circle Navigator Node Initialized");
    }

private:
    void checkActionServerAndProcessQueue()
    {
        // Check action server availability
        if (!nav_action_client_) {
            RCLCPP_ERROR(this->get_logger(), "Action client is null");
            return;
        }

        // Wait for a short time to check action server
        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Navigation action server not ready");
            is_action_server_ready_ = false;
            return;
        }

        // Mark action server as ready
        is_action_server_ready_ = true;

        // Process queued goals if any
        processQueuedGoals();
    }

    void processQueuedGoals()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // Process goals in the queue
        while (!goal_queue_.empty()) {
            auto goal_pose = goal_queue_.front();
            goal_queue_.pop();

            try {
                sendNavigationGoal(goal_pose);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Error processing queued goal: %s", e.what());
            }
        }
    }

    void circlePositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received circle position: x=%.2f, y=%.2f, z=%.2f", 
            msg->x, msg->y, msg->z
        );

        // Create navigation goal
        auto goal_pose = createNavigationGoal(*msg);
        
        if (!goal_pose) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create navigation goal");
            return;
        }

        // If action server is not ready, queue the goal
        if (!is_action_server_ready_) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            goal_queue_.push(*goal_pose);
            RCLCPP_WARN(this->get_logger(), 
                "Action server not ready. Queuing navigation goal.");
            return;
        }

        // Try to send goal directly
        try {
            sendNavigationGoal(*goal_pose);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Error sending navigation goal: %s", e.what());
            
            // Enqueue the goal for later retry
            std::lock_guard<std::mutex> lock(queue_mutex_);
            goal_queue_.push(*goal_pose);
        }
    }

    std::optional<geometry_msgs::msg::PoseStamped> createNavigationGoal(const geometry_msgs::msg::Point& point)
    {
        geometry_msgs::msg::PoseStamped goal_pose;
        
        // Set frame and timestamp
        goal_pose.header.frame_id = map_frame_;
        goal_pose.header.stamp = this->get_clock()->now();
        
        // Set position from input point
        goal_pose.pose.position.x = point.x + goal_offset_;
        goal_pose.pose.position.y = point.y;
        goal_pose.pose.position.z = point.z;
        
        // Set default orientation (identity quaternion)
        goal_pose.pose.orientation.w = 1.0;
        
        return goal_pose;
    }

    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal_pose)
    {
        // Ensure action server is ready
        if (!is_action_server_ready_) {
            throw std::runtime_error("Action server not ready");
        }

        // Prepare navigation goal
        auto goal = NavigateToPose::Goal();
        goal.pose = goal_pose;

        RCLCPP_INFO(this->get_logger(), 
            "Sending navigation goal: x=%.2f, y=%.2f", 
            goal_pose.pose.position.x, 
            goal_pose.pose.position.y
        );

        // Send goal with callbacks
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = 
            std::bind(&CircleNavigator::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = 
            std::bind(&CircleNavigator::navResultCallback, this, std::placeholders::_1);

        nav_action_client_->async_send_goal(goal, send_goal_options);
    }

    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr goal_handle)
    {
        if (!goal_handle) 
        {
            RCLCPP_WARN(this->get_logger(), "Navigation goal was rejected!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");
    }

    void navResultCallback(const GoalHandleNavigateToPose::WrappedResult& result)
    {
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Successfully navigated to the goal!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(this->get_logger(), "Navigation was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    // Node members
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr circle_subscription_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::TimerBase::SharedPtr action_server_timer_;
    
    // Goal queuing mechanism
    std::queue<geometry_msgs::msg::PoseStamped> goal_queue_;
    std::mutex queue_mutex_;
    std::atomic<bool> is_action_server_ready_{false};
    
    // Configuration parameters
    double goal_offset_;
    std::string base_frame_;
    std::string map_frame_;
    int max_retry_attempts_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CircleNavigator>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}