#ifndef MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_SERVER_HPP_
#define MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_SERVER_HPP_
#include <functional> // std::bind
#include <memory> // std::shared_ptr

#include "my_sim_tesi_ros2_interfaces/action/drone_pose_control.hpp" // my_sim_tesi_ros2_interfaces::action::DronePoseControl
#include "rclcpp/rclcpp.hpp" // rclcpp::Node
#include "rclcpp_action/rclcpp_action.hpp" // rclcpp_action::ServerGoalHandle
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_sim_tesi_ros2_nodes {
    /**
    * @class PoseControlActionServer
    * @brief Gestisce il controllo della posa del drone attraverso una action ROS 2 (action server).
    * 
    * Questa classe implementa un server di action che permette di controllare la posa del drone
    * (posizione, orientamento, altitudine) in risposta agli obiettivi inviati tramite l'action client ROS 2.
    */
    class PoseControlActionServer : public rclcpp::Node {

    public:
        using PoseControl = my_sim_tesi_ros2_interfaces::action::DronePoseControl;
        using GoalHandlePoseControl = rclcpp_action::ServerGoalHandle<PoseControl>;
        enum class Axis {
            X,
            Y,
            Z
        };

        explicit PoseControlActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        rclcpp_action::Server<PoseControl>::SharedPtr action_server_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr altimeter_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        double current_x_;
        double current_y_;
        double current_yaw_;
        double current_altitude_;

        void odometry_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void altimeter_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const PoseControl::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePoseControl> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandlePoseControl> goal_handle);

        void execute(const std::shared_ptr<GoalHandlePoseControl> goal_handle);

        bool moveToGoal(
            std::shared_ptr<my_sim_tesi_ros2_interfaces::action::DronePoseControl_Feedback> feedback,
            const std::shared_ptr<GoalHandlePoseControl> goal_handle, Axis axis, bool is_linear,
            double &current_coordinate, double target_value, double tollerance, double velocity);

        double reduceVelocity(double initial_velocity, double distance_to_goal, Axis axis, bool is_linear);

        double normalizeAngle (double var_required);

    }; // class PoseControlActionServer
}
#endif // MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_SERVER_HPP_