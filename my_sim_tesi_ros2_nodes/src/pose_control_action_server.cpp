#include <thread>
#include <cmath>

#include "my_sim_tesi_ros2_nodes/pose_control_action_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_sim_tesi_ros2_nodes
{
PoseControlActionServer::PoseControlActionServer(const rclcpp::NodeOptions& options)
  : Node("pose_control_action_server", options)
{
  this->declare_parameter<double>("x_start", 0.0);
  this->declare_parameter<double>("y_start", 0.0);
  this->declare_parameter<double>("yaw_start", 0.0);

  current_x_ = this->get_parameter("x_start").as_double();
  current_y_ = this->get_parameter("y_start").as_double();
  current_yaw_ = this->get_parameter("yaw_start").as_double();
  current_altitude_ = 0.0;

  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<PoseControl>(
      this, "drone_pose_control", std::bind(&PoseControlActionServer::handle_goal, this, _1, _2),
      std::bind(&PoseControlActionServer::handle_cancel, this, _1),
      std::bind(&PoseControlActionServer::handle_accepted, this, _1));

  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/quadcopter/odometry", 10, std::bind(&PoseControlActionServer::odometry_topic_callback, this, _1));

  altimeter_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/quadcopter/altimeter", 10, std::bind(&PoseControlActionServer::altimeter_topic_callback, this, _1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/quadcopter/cmd_vel", 10);
}

void PoseControlActionServer::odometry_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  auto orientation = msg->pose.pose.orientation;
  tf2::Quaternion quat(orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;
}

void PoseControlActionServer::altimeter_topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (msg->ranges.size() > 0)
  {
    double altitude = msg->ranges[0];  // Legge il valore del raggio

    // Controlla che l'altitudine sia un valore valido diverso da infinito e NaN
    if (std::isfinite(altitude))
    {
      current_altitude_ = altitude;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid distance measurement.");
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No range data available.");
  }
}

rclcpp_action::GoalResponse PoseControlActionServer::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                                 std::shared_ptr<const PoseControl::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with target pose [x, y, yaw, altitude] ="
              "[%f, %f ,%f ,%f]",
              goal->target_x, goal->target_y, goal->target_yaw, goal->target_altitude);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
PoseControlActionServer::handle_cancel(const std::shared_ptr<GoalHandlePoseControl> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PoseControlActionServer::handle_accepted(const std::shared_ptr<GoalHandlePoseControl> goal_handle)
{
  using namespace std::placeholders;
  // nuovo thread poiché per evitare di bloccare l'executor bisogna terminare questo metodo velocemente
  std::thread{ std::bind(&PoseControlActionServer::execute, this, _1), goal_handle }.detach();
}

void PoseControlActionServer::execute(const std::shared_ptr<GoalHandlePoseControl> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  bool goal_deleted = false;
  double target_x = goal->target_x;
  double target_y = goal->target_y;
  double target_yaw = goal->target_yaw;
  double target_altitude = goal->target_altitude;

  auto feedback = std::make_shared<PoseControl::Feedback>();
  feedback->current_x = current_x_;
  feedback->current_y = current_y_;
  feedback->current_yaw = current_yaw_;
  feedback->current_altitude = current_altitude_;
  auto result = std::make_shared<PoseControl::Result>();

  // ogni volta si controlla se va aggiustata l'altezza poiché il drone tende ad abbassarsi ad ogni variazione di
  // velocità
  if (std::abs(current_altitude_ - target_altitude) > 0.1)
  {
    goal_deleted = moveToGoal(feedback, goal_handle, Axis::Z, true, current_altitude_, target_altitude, 0.05, 0.4);
    if (goal_deleted)
    {
      result->res_x = current_x_;
      result->res_y = current_y_;
      result->res_yaw = current_yaw_;
      result->res_altitude = current_altitude_;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
  }
  if (std::abs(normalizeAngle(current_yaw_ - target_yaw)) > 0.2)
  {
    goal_deleted = moveToGoal(feedback, goal_handle, Axis::Z, false, current_yaw_, target_yaw, 0.03, 0.2);
  }
  else if (std::abs(current_x_ - target_x) > 0.2)
  {
    goal_deleted = moveToGoal(feedback, goal_handle, Axis::X, true, current_x_, target_x, 0.1, 0.8);
  }
  else if (std::abs(current_y_ - target_y) > 0.2)
  {
    goal_deleted = moveToGoal(feedback, goal_handle, Axis::Y, true, current_y_, target_y, 0.1, 0.8);
  }

  result->res_x = current_x_;
  result->res_y = current_y_;
  result->res_yaw = current_yaw_;
  result->res_altitude = current_altitude_;

  if (goal_deleted)
  {
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return;
  }

  if (rclcpp::ok())
  {
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

double PoseControlActionServer::normalizeAngle(double var_required)
{
  if (var_required > M_PI)
  {
    var_required -= 2 * M_PI;
  }
  else if (var_required < -M_PI)
  {
    var_required += 2 * M_PI;
  }
  return var_required;
}

/**
 * @brief Esegue mediante messaggi di Twist il movimento verso l'obiettivo specificato
 *
 * Questo metodo controlla i movimenti del drone verso un obiettivo specificato in base all'asse di movimento
 * ed alla tolleranza specficata. Si occupa anche della riduzione della velocità quando si avvicina all'obiettivo.
 *
 * @param feedback Puntatore al feedback condiviso con il client della action.
 * @param goal_handle Puntatore al goal handle, obiettivo dell'action.
 * @param axis L'asse lungo il quale il drone deve muoversi (X, Y, Z).
 * @param is_linear Indica se il movimento è lineare (true) o angolare (false).
 * @param current_coordinate Riferimento alla coordinata attuale lungo l'asse specificato dai parametri 'axis' e
 * 'isLinear'.
 * @param target_value Il valore della posizione obiettivo da raggiungere lungo l'asse specificato.
 * @param tolerance La tolleranza entro la quale considerare raggiunto l'obiettivo.
 * @param velocity La velocità iniziale da assegnare al movimento.
 *
 * @return true se l'obiettivo è stato annullato, altrimenti false.
 */
bool PoseControlActionServer::moveToGoal(
    std::shared_ptr<my_sim_tesi_ros2_interfaces::action::DronePoseControl_Feedback> feedback,
    const std::shared_ptr<GoalHandlePoseControl> goal_handle, Axis axis, bool is_linear, double& current_coordinate,
    double target_value, double tollerance, double velocity)
{
  rclcpp::Rate rate(5);
  geometry_msgs::msg::Twist twist_msg;
  bool goal_deleted = false;
  double var_required = current_coordinate - target_value;
  if (!is_linear)
  {
    var_required = normalizeAngle(var_required);
  }
  int direction = (var_required < 0) ? 1 : -1;
  while (std::abs(var_required) > tollerance && rclcpp::ok())
  {
    if (goal_handle->is_canceling())
    {
      goal_deleted = true;
      break;  // esce dal while ed azzera la velocità
    }
    velocity = reduceVelocity(velocity, var_required, axis, is_linear);
    if (is_linear)
    {
      switch (axis)
      {
        case Axis::X:
          twist_msg.linear.x = velocity;
          feedback->current_x = current_coordinate;
          break;
        case Axis::Y:
          twist_msg.linear.x = velocity;
          feedback->current_y = current_coordinate;
          break;
        case Axis::Z:
          twist_msg.linear.z = velocity * direction;
          feedback->current_altitude = current_coordinate;
          break;
      }
    }
    else
    {
      twist_msg.angular.z = velocity * direction;
      feedback->current_yaw = current_coordinate;
    }
    cmd_vel_publisher_->publish(twist_msg);
    goal_handle->publish_feedback(feedback);
    rate.sleep();
    if (is_linear)
    {
      var_required = current_coordinate - target_value;
    }
    else
    {
      var_required = normalizeAngle(current_coordinate - target_value);
    }
    direction = (var_required < 0) ? 1 : -1;
  }
  twist_msg.linear.x = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.z = 0;
  cmd_vel_publisher_->publish(twist_msg);
  return goal_deleted;
}

/**
 * @brief Calcola la riduzione della velocità in base alla vicinanza dell'obiettivo.
 *
 * Questo metodo si occupa di calcolare la nuova velocità sulla base della vicinanza della posizione
 * obiettivo in modo da rendere fluida la decelerazione del drone e di conseguenza più accurata la posizione
 * finale di arrivo.
 *
 * @param initial_velocity Velocità corrente del drone.
 * @param distance_to_goal Distanza residua da percorrere per raggiungere l'obiettivo.
 * @param axis L'asse lungo il quale il drone deve muoversi (X, Y, Z).
 * @param is_linear Indica se il movimento è lineare (true) o angolare (false).
 *
 * @return la nuova velocità (double) da assegnare al movimento del drone al prossimo messaggio Twist.
 */
double PoseControlActionServer::reduceVelocity(double initial_velocity, double distance_to_goal, Axis axis,
                                               bool is_linear)
{
  double final_velocity = initial_velocity;
  distance_to_goal = std::abs(distance_to_goal);
  if (is_linear)
  {
    if (axis == Axis::Z)
    {
      if (distance_to_goal < 0.6)
      {  // riduce la velocità lineare se siamo vicini all'obiettivo
        final_velocity = 0.15;
      }
    }
    else
    {
      if (distance_to_goal < 0.5)
      {  // riduce la velocità lineare se siamo vicini all'obiettivo
        final_velocity = 0.1;
      }
      else if (distance_to_goal < 2.0)
      {
        final_velocity = 0.5;
      }
    }
  }
  else
  {
    if (distance_to_goal < 0.35)
    {  // riduce la velocità lineare se siamo vicini all'obiettivo
      final_velocity = 0.07;
    }
  }
  return final_velocity;
}
}  // namespace my_sim_tesi_ros2_nodes

RCLCPP_COMPONENTS_REGISTER_NODE(my_sim_tesi_ros2_nodes::PoseControlActionServer)