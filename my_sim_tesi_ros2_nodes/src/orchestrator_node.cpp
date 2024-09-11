#include <functional>
#include <chrono>

#include "my_sim_tesi_ros2_nodes/orchestrator_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


namespace my_sim_tesi_ros2_nodes
{
    OrchestratorNode::OrchestratorNode(const rclcpp::NodeOptions &options)
        : Node("orchestrator_node", options)
    {
        using namespace std::placeholders;
        
        start_drone_publisher_ = this->create_publisher<std_msgs::msg::String>("/quadcopter/start_navigation", 10);
        end_drone_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/quadcopter/end_navigation",
            10,
            std::bind(&OrchestratorNode::drone_arrived_callback, this, _1));
        initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        timer_drone_start_navigation_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&OrchestratorNode::publish_drone_start, this));
        clock_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, std::bind(&OrchestratorNode::clock_callback, this, std::placeholders::_1));
    }

    void OrchestratorNode::clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        auto time = msg->clock.sec;
        if(time > 0) {
            clock_subscriber_.reset(); // per evitare di effettuare callback dopo che il simulatore è stato avviato
            is_simulation_running_ = true;
        }
    }

    void OrchestratorNode::publish_drone_start() {
        if(!is_simulation_running_) {
            return;
        }
        this->timer_drone_start_navigation_->cancel(); // invia una volta sola il messaggio sul topic
        auto start_msg = std_msgs::msg::String();
        start_msg.data = "start_drone_navigation";
        start_drone_publisher_->publish(start_msg);
        this->publish_initial_pose();
    }

    void OrchestratorNode::drone_arrived_callback(const std_msgs::msg::String & msg){
        auto message = msg.data;
        if(message != "end_drone_navigation" || drone_has_already_navigated_) {
            return;
        }
        drone_has_already_navigated_ = true;
        RCLCPP_INFO(this->get_logger(), "The quadcopter has finished navigation, preparing to start autonomous driving");
        this->publish_goal_pose();
    }
 
    void OrchestratorNode::publish_initial_pose()
    {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->get_clock()->now();

        /*i valori della della covarianza li ho letti con una echo sul topic /initial_pose
          e poi impostandola manualmente da GUI RViz2*/
        pose_msg.pose.pose.position.x = -13.10;
        pose_msg.pose.pose.position.y = 2.64;
        pose_msg.pose.pose.position.z = 0.0;

        double yaw = M_PI * (-1) / 2;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);

        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        pose_msg.pose.covariance[0] = 0.25;
        pose_msg.pose.covariance[7] = 0.25;
        pose_msg.pose.covariance[35] = 0.06853891909122467;

        initial_pose_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Initial pose published!");
    }

    void OrchestratorNode::publish_goal_pose()
    {
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        
        goal_msg.header.frame_id = "map";
        goal_msg.header.stamp = this->get_clock()->now();

        goal_msg.pose.position.x = -0.25;
        goal_msg.pose.position.y = -0.92;
        double yaw = M_PI * (-1) / 2;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal_msg.pose.orientation.x = q.x();
        goal_msg.pose.orientation.y = q.y();
        goal_msg.pose.orientation.z = q.z();
        goal_msg.pose.orientation.w = q.w();
        goal_pose_publisher_->publish(goal_msg);
        RCLCPP_INFO(this->get_logger(), "Goal published! Navigating to [x, y, yaw] = [%f, %f, %f]",
            goal_msg.pose.position.x, goal_msg.pose.position.y, yaw);
    }

} // namespace my_sim_tesi_ros2_nodes
RCLCPP_COMPONENTS_REGISTER_NODE(my_sim_tesi_ros2_nodes::OrchestratorNode)
