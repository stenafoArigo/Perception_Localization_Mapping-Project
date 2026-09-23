#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Goal {
  double x, y, theta;
};

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher() : Node("goal_publisher"), current_goal_idx_(0), goal_active_(false)
  {
    this->declare_parameter<std::string>("csv_path", "");

    std::string csv_path = this->get_parameter("csv_path").as_string();
    if (csv_path.empty()) {
      RCLCPP_ERROR(this->get_logger(),
        "Parameter 'csv_path' not set. Pass it via launch file.");
      return;
    }

    load_goals(csv_path);

    if (goals_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No goals loaded from CSV: %s", csv_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu goals from %s",
      goals_.size(), csv_path.c_str());

   
    rclcpp::QoS qos_profile(10);
    qos_profile.transient_local();

    all_goals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/all_goals", qos_profile);

    publish_all_goals();
   

    action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GoalPublisher::try_send_goal, this));
  }

private:

  void publish_all_goals()
  {
    if (goals_.empty()) return;

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = this->now();

    for (const auto & g : goals_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = g.x;
      pose.position.y = g.y;
      pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, g.theta);
      pose.orientation = tf2::toMsg(q);

      pose_array.poses.push_back(pose);
    }

    all_goals_pub_->publish(pose_array);
  }
  

  void load_goals(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", path.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') continue;  
      std::istringstream ss(line);
      std::string tok;
      std::vector<double> vals;
      while (std::getline(ss, tok, ',')) {
        try { vals.push_back(std::stod(tok)); }
        catch (...) { break; }
      }
      if (vals.size() >= 3) {
        goals_.push_back({vals[0], vals[1], vals[2]});
      }
    }
  }

  void try_send_goal()
  {
    if (goal_active_) return;
    if (current_goal_idx_ >= goals_.size()) {
      RCLCPP_INFO(this->get_logger(), "All goals completed!");
      timer_->cancel();
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Waiting for navigate_to_pose action server...");
      return;
    }

    send_next_goal();
  }

  void send_next_goal()
  {
    const auto & g = goals_[current_goal_idx_];

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x = g.x;
    goal_msg.pose.pose.position.y = g.y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g.theta);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    RCLCPP_INFO(this->get_logger(),
      "Sending goal %zu/%zu  →  x=%.2f  y=%.2f  theta=%.2f",
      current_goal_idx_ + 1, goals_.size(), g.x, g.y, g.theta);

    auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
          goal_active_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted");
          goal_active_ = true;
        }
      };

    send_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        goal_active_ = false;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "Goal %zu SUCCEEDED", current_goal_idx_ + 1);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(),
              "Goal %zu ABORTED – moving to next", current_goal_idx_ + 1);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
              "Goal %zu CANCELED – moving to next", current_goal_idx_ + 1);
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        current_goal_idx_++;
      };

    action_client_->async_send_goal(goal_msg, send_options);
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Goal> goals_;
  size_t current_goal_idx_;
  bool   goal_active_;
  

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_goals_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}
