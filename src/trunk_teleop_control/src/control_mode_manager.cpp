#include <algorithm>
#include <chrono>
#include <cctype>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "trunk_teleop_control/srv/set_control_mode.hpp"

namespace
{

std::string normalizeMode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return mode;
}

class ControlModeManager : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using SetControlMode = trunk_teleop_control::srv::SetControlMode;

  ControlModeManager()
  : Node("control_mode_manager")
  {
    state_topic_ = declare_parameter<std::string>("state_topic", "control_mode_state");
    set_mode_service_ = declare_parameter<std::string>("set_mode_service", "set_control_mode");
    current_mode_ = normalizeMode(declare_parameter<std::string>("initial_mode", "manual_teleop"));
    manual_stop_wait_sec_ = declare_parameter<double>("manual_stop_wait_sec", 0.25);
    cancel_action_on_takeover_ = declare_parameter<bool>("cancel_action_on_takeover", true);
    follow_joint_trajectory_action_ = declare_parameter<std::string>(
      "follow_joint_trajectory_action", "trunk_group_controller/follow_joint_trajectory");

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, qos);
    set_mode_srv_ = create_service<SetControlMode>(
      set_mode_service_,
      std::bind(
        &ControlModeManager::setModeCallback, this, std::placeholders::_1,
        std::placeholders::_2));

    if (cancel_action_on_takeover_) {
      action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, follow_joint_trajectory_action_);
    }

    if (!isValidMode(current_mode_)) {
      RCLCPP_WARN(
        get_logger(), "Invalid initial_mode '%s'; falling back to idle.", current_mode_.c_str());
      current_mode_ = "idle";
    }
    publishMode();

    RCLCPP_INFO(
      get_logger(),
      "Control mode manager ready. service='%s' state_topic='%s' initial_mode='%s'",
      set_mode_service_.c_str(), state_topic_.c_str(), current_mode_.c_str());
  }

private:
  bool isValidMode(const std::string & mode) const
  {
    static const std::unordered_set<std::string> valid_modes = {
      "idle", "manual_teleop", "auto_plan_execute", "estop"};
    return valid_modes.count(mode) > 0;
  }

  void publishMode()
  {
    std_msgs::msg::String msg;
    msg.data = current_mode_;
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Control mode: %s", current_mode_.c_str());
  }

  void cancelControllerGoalIfNeeded(const std::string & requested_mode)
  {
    if (!cancel_action_on_takeover_ || !action_client_) {
      return;
    }
    if (current_mode_ != "auto_plan_execute") {
      return;
    }
    if (requested_mode != "manual_teleop" && requested_mode != "idle" && requested_mode != "estop") {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Controller action '%s' is not ready; cannot request cancel during mode switch.",
        follow_joint_trajectory_action_.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Requesting cancellation of active FollowJointTrajectory goals before switching to %s.",
      requested_mode.c_str());
    (void)action_client_->async_cancel_all_goals();
  }

  void setModeCallback(
    const std::shared_ptr<SetControlMode::Request> request,
    std::shared_ptr<SetControlMode::Response> response)
  {
    const std::string requested_mode = normalizeMode(request->mode);
    if (!isValidMode(requested_mode)) {
      response->success = false;
      response->message =
        "Invalid mode '" + request->mode +
        "'. Expected idle, manual_teleop, auto_plan_execute, or estop.";
      return;
    }

    if (requested_mode == current_mode_) {
      response->success = true;
      response->message = "Control mode already " + current_mode_ + ".";
      publishMode();
      return;
    }

    cancelControllerGoalIfNeeded(requested_mode);

    if (current_mode_ == "manual_teleop" && requested_mode == "auto_plan_execute") {
      current_mode_ = "idle";
      publishMode();
      if (manual_stop_wait_sec_ > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(manual_stop_wait_sec_));
      }
    }

    current_mode_ = requested_mode;
    publishMode();
    response->success = true;
    response->message = "Control mode switched to " + current_mode_ + ".";
  }

  std::string state_topic_;
  std::string set_mode_service_;
  std::string current_mode_;
  double manual_stop_wait_sec_{0.25};
  bool cancel_action_on_takeover_{true};
  std::string follow_joint_trajectory_action_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<SetControlMode>::SharedPtr set_mode_srv_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlModeManager>());
  rclcpp::shutdown();
  return 0;
}
