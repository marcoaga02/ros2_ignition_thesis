#ifndef MY_SIM_TESI_ROS2_NODES_ORCHESTRATOR_NODE_HPP_
#define MY_SIM_TESI_ROS2_NODES_ORCHESTRATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace my_sim_tesi_ros2_nodes
{
class OrchestratorNode : public rclcpp::Node
{
public:
    OrchestratorNode(const rclcpp::NodeOptions &options);
    
private:
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_drone_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr end_drone_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_drone_start_navigation_;
    bool drone_has_already_navigated_ = false;
    bool is_simulation_running_ = false;

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void publish_drone_start();
    void publish_initial_pose(); 
    void publish_goal_pose();
    void drone_arrived_callback(const std_msgs::msg::String & msg);
};
}  // namespace my_sim_tesi_ros2_nodes
#endif // MY_SIM_TESI_ROS2_NODES_ORCHESTRATOR_NODE_HPP_