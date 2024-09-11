#include <fstream>
#include <string>
#include <chrono>

#include "my_sim_tesi_ros2_nodes/pose_control_action_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace my_sim_tesi_ros2_nodes {
    PoseControlActionClient::PoseControlActionClient(const rclcpp::NodeOptions &options)
        : Node("pose_control_action_client", options) {
            this->declare_parameter<std::string>("file_path", "");
            std::string file_path = this->get_parameter("file_path").as_string();

            using namespace std::placeholders;

        if (!file_path.empty()) {
            auto file_ptr_ = std::make_unique<std::ifstream>(file_path);
            if (!file_ptr_->is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
                rclcpp::shutdown();
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "File opened successfully: %s", file_path.c_str());
            }
            double x, y, yaw, z;
            while (*file_ptr_ >> x >> y >> yaw >> z) {
                coordinates_vector_.emplace_back(x, y, yaw, z);
            }
            file_ptr_->close();
            current_goal_ = coordinates_vector_.begin();
        } else {
            RCLCPP_ERROR(this->get_logger(), "File path not provided.");
        }

        this->client_ptr_ = rclcpp_action::create_client<PoseControl>(
            this,
            "drone_pose_control");
        
        start_drone_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/quadcopter/start_navigation",
            10,
            std::bind(&PoseControlActionClient::drone_start_callback, this, _1));

        end_drone_publisher_ = this->create_publisher<std_msgs::msg::String>("quadcopter/end_navigation", 10);
    }

    void PoseControlActionClient::drone_start_callback(const std_msgs::msg::String & msg) {
        auto message = msg.data;
        if(message != "start_drone_navigation" || drone_has_already_navigated_) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "The quadcopter is starting navigation");
        drone_has_already_navigated_ = true;
        
        /* ogni 'timer_period_ms' ms si esegue la callback con cui si controlla se il goal precedente è stato
        completato ed in tal caso si invia il successivo */
        this->timer_ = this->create_wall_timer( // inizializzo il timer delle callback dopo il messaggio di start
            std::chrono::milliseconds(500),
            std::bind(&PoseControlActionClient::send_goal, this));
    }

    void PoseControlActionClient::send_goal() {
        using namespace std::placeholders;
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
        if (is_goal_active_) {
            return;
        } else if (current_goal_ == coordinates_vector_.end()) {
            this->timer_->cancel(); // finiti i goal terminano le callback
            this->publish_end_of_navigation();
            return;
        }

        auto goal_msg = PoseControl::Goal();
        goal_msg.target_x = std::get<0>(*current_goal_);
        goal_msg.target_y = std::get<1>(*current_goal_);
        goal_msg.target_yaw = std::get<2>(*current_goal_);
        goal_msg.target_altitude = std::get<3>(*current_goal_);
        
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<PoseControl>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PoseControlActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&PoseControlActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&PoseControlActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        is_goal_active_ = true;
    }

    void PoseControlActionClient::goal_response_callback(const GoalHandlePoseControl::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void PoseControlActionClient::feedback_callback(
        GoalHandlePoseControl::SharedPtr,
        const std::shared_ptr<const PoseControl::Feedback> feedback) {
        double current_x = feedback->current_x;
        double current_y = feedback->current_y;
        double current_yaw = feedback->current_yaw;
        double current_altitude = feedback->current_altitude;
        RCLCPP_INFO(this->get_logger(), "Action Feedback: current pose is [x=%f, y=%f, yaw=%f,altitude=%f]",
            current_x, current_y, current_yaw, current_altitude);
    }

    void PoseControlActionClient::result_callback(const GoalHandlePoseControl::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        double current_x = result.result->res_x;
        double current_y = result.result->res_y;
        double current_yaw = result.result->res_yaw;
        double current_altitude = result.result->res_altitude;
        RCLCPP_INFO(this->get_logger(), "Goal succeeded: current pose is [x=%f, y=%f, yaw=%f,altitude=%f]",
            current_x, current_y, current_yaw, current_altitude);
        is_goal_active_ = false;
        current_goal_++; // per passare al goal successivo
    }

    void PoseControlActionClient::publish_end_of_navigation() {
        auto end_msg = std_msgs::msg::String();
        end_msg.data = "end_drone_navigation";
        this->end_drone_publisher_->publish(end_msg);
    }
} // namespace my_sim_tesi_ros2_nodes

// Registrazione del nodo
RCLCPP_COMPONENTS_REGISTER_NODE(my_sim_tesi_ros2_nodes::PoseControlActionClient)
