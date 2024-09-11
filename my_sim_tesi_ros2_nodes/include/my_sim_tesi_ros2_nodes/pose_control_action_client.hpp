#ifndef MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_CLIENT_HPP_
#define MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_CLIENT_HPP_

#include <memory> // std::shared_ptr
#include <tuple>
#include <vector>

#include "my_sim_tesi_ros2_interfaces/action/drone_pose_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace my_sim_tesi_ros2_nodes
{

  /**
   * @class PoseControlActionClient
   * @brief Gestisce il controllo della posa del drone attraverso una action ROS 2 (action client).
   *
   * Questa classe implementa un client di action che permette di controllare la posa del drone
   * (posizione, orientamento, altitudine) inviando obiettivi all'action server ROS 2.
   * Gli obiettivi sono definiti nel file che viene passato come parametro all'avvio del nodo.
   * In tale file, essi sono organizzati in 4 valori double per riga separati ciascuno da uno spazio
   * e scritti nel seguente ordine: target_x target_y target_yaw target_altitude.
   * Inoltre, un obiettivo può variare rispetto al precedente rispetto alla target_altitude
   * ed uno solo tra i restanti tre valori (poiché i moviementi sono solo lungo l'asse x
   * del drone, è necessario effettuare prima le rotazioni e dopodiché, quando il drone è
   * orientato correttamente, procedere con i movimenti lineari).
   */
  class PoseControlActionClient : public rclcpp::Node
  {
  public:
    using PoseControl = my_sim_tesi_ros2_interfaces::action::DronePoseControl;
    using GoalHandlePoseControl = rclcpp_action::ClientGoalHandle<PoseControl>;

    explicit PoseControlActionClient(const rclcpp::NodeOptions &options);

  private:
    rclcpp_action::Client<PoseControl>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr end_drone_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_drone_subscriber_;
    std::vector<std::tuple<double, double, double, double>> coordinates_vector_;
    std::vector<std::tuple<double, double, double, double>>::iterator current_goal_;
    bool is_goal_active_ = false;
    bool drone_has_already_navigated_ = false;

    void send_goal();
    void drone_start_callback(const std_msgs::msg::String &msg);
    void goal_response_callback(const GoalHandlePoseControl::SharedPtr &goal_handle);
    void feedback_callback(GoalHandlePoseControl::SharedPtr, const std::shared_ptr<const PoseControl::Feedback> feedback);
    void result_callback(const GoalHandlePoseControl::WrappedResult &result);
    void publish_end_of_navigation();
  }; // class PoseControlActionClient
} // namespace my_sim_tesi_ros2_nodes
#endif // MY_SIM_TESI_ROS2_NODES_POSE_CONTROL_ACTION_CLIENT_HPP_