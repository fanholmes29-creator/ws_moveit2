#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class TrunkJoystickTeleop : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  TrunkJoystickTeleop()
  : Node("trunk_joystick_teleop")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "joy");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "joint_states");
    controller_state_topic_ = declare_parameter<std::string>(
      "controller_state_topic", "trunk_group_controller/controller_state");
    use_controller_state_ = declare_parameter<bool>("use_controller_state", true);
    follow_joint_trajectory_action_ = declare_parameter<std::string>(
      "follow_joint_trajectory_action", "trunk_group_controller/follow_joint_trajectory");
    joint_trajectory_topic_ = declare_parameter<std::string>(
      "joint_trajectory_topic", "trunk_group_controller/joint_trajectory");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"trunk_joint1", "trunk_joint2", "trunk_joint3", "trunk_joint4"});

    control_mode_ = declare_parameter<std::string>("control_mode", "step");
    control_rate_ = declare_parameter<double>("control_rate", 20.0);
    max_velocity_rad_s_ = declare_parameter<double>("max_velocity_rad_s", 0.25);
    command_duration_ = declare_parameter<double>("command_duration", 0.15);
    axis_deadzone_ = declare_parameter<double>("axis_deadzone", 0.15);
    command_state_max_error_rad_ = declare_parameter<double>(
      "command_state_max_error_rad", 0.2);
    step_rad_ = declare_parameter<double>("step_rad", 0.034906585);
    trajectory_duration_ = declare_parameter<double>("trajectory_duration", 0.5);
    joy_timeout_ = declare_parameter<double>("joy_timeout", 0.5);
    require_deadman_ = declare_parameter<bool>("require_deadman", true);

    button_a_ = declare_parameter<int>("button_a", 0);
    button_b_ = declare_parameter<int>("button_b", 1);
    button_x_ = declare_parameter<int>("button_x", 3);
    button_y_ = declare_parameter<int>("button_y", 4);
    button_l1_ = declare_parameter<int>("button_l1", 6);
    axis_step_ = declare_parameter<int>("axis_step", 1);
    axis_step_threshold_ = declare_parameter<double>("axis_step_threshold", 0.5);
    invert_axis_step_ = declare_parameter<bool>("invert_axis_step", false);

    joint_min_limits_.reserve(joint_names_.size());
    joint_max_limits_.reserve(joint_names_.size());
    for (const auto & joint_name : joint_names_) {
      joint_min_limits_.push_back(declare_parameter<double>(joint_name + ".min", -3.14));
      joint_max_limits_.push_back(declare_parameter<double>(joint_name + ".max", 3.14));
    }

    if (joint_names_.empty()) {
      RCLCPP_WARN(get_logger(), "Parameter joint_names is empty; no motion goals can be sent.");
    }

    q_current_.assign(joint_names_.size(), 0.0);
    selected_joint_index_ = 0;

    if (control_mode_ != "step" && control_mode_ != "continuous") {
      RCLCPP_WARN(
        get_logger(),
        "Invalid control_mode '%s'; falling back to 'step'. Expected 'step' or 'continuous'.",
        control_mode_.c_str());
      control_mode_ = "step";
    }
    if (control_rate_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid control_rate %.3f; falling back to 20.0 Hz.", control_rate_);
      control_rate_ = 20.0;
    }
    if (command_duration_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid command_duration %.3f; falling back to 0.15 s.",
        command_duration_);
      command_duration_ = 0.15;
    }
    if (max_velocity_rad_s_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid max_velocity_rad_s %.3f; using its absolute value.",
        max_velocity_rad_s_);
      max_velocity_rad_s_ = std::abs(max_velocity_rad_s_);
    }
    if (axis_deadzone_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid axis_deadzone %.3f; using its absolute value.", axis_deadzone_);
      axis_deadzone_ = std::abs(axis_deadzone_);
    }
    if (command_state_max_error_rad_ < 0.0) {
      RCLCPP_WARN(
        get_logger(), "Invalid command_state_max_error_rad %.3f; using its absolute value.",
        command_state_max_error_rad_);
      command_state_max_error_rad_ = std::abs(command_state_max_error_rad_);
    }

    if (control_mode_ == "step") {
      action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, follow_joint_trajectory_action_);
      RCLCPP_INFO(
        get_logger(), "Using step control mode with FollowJointTrajectory action '%s'.",
        follow_joint_trajectory_action_.c_str());
    } else {
      // Continuous teleop streams short position trajectories directly to the controller topic.
      // This avoids waiting on action results while the joystick is held.
      trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        joint_trajectory_topic_, rclcpp::SystemDefaultsQoS());
      const auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
      control_timer_ = create_wall_timer(
        control_period, std::bind(&TrunkJoystickTeleop::controlTimerCallback, this));
      RCLCPP_INFO(
        get_logger(),
        "Using continuous control mode at %.3f Hz, publishing JointTrajectory to '%s'.",
        control_rate_, joint_trajectory_topic_.c_str());
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TrunkJoystickTeleop::joyCallback, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TrunkJoystickTeleop::jointStateCallback, this, std::placeholders::_1));

    if (use_controller_state_) {
      controller_state_sub_ =
        create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
          controller_state_topic_, rclcpp::SystemDefaultsQoS(),
          std::bind(
            &TrunkJoystickTeleop::controllerStateCallback, this, std::placeholders::_1));
    }

    logParameters();
  }

private:
  static std::string vectorToString(const std::vector<double> & values)
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      stream << values[i];
      if (i + 1 < values.size()) {
        stream << ", ";
      }
    }
    stream << "]";
    return stream.str();
  }

  static std::string vectorToStringNames(const std::vector<std::string> & values)
  {
    std::ostringstream stream;
    stream << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      stream << values[i];
      if (i + 1 < values.size()) {
        stream << ", ";
      }
    }
    stream << "]";
    return stream.str();
  }

  static builtin_interfaces::msg::Duration secondsToDurationMsg(double seconds)
  {
    const double clamped_seconds = std::max(0.0, seconds);
    const double whole_seconds = std::floor(clamped_seconds);
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<std::int32_t>(whole_seconds);
    duration.nanosec = static_cast<std::uint32_t>(
      (clamped_seconds - whole_seconds) * 1000000000.0);
    return duration;
  }

  static bool isButtonPressed(const sensor_msgs::msg::Joy & msg, int index)
  {
    return index >= 0 &&
           static_cast<std::size_t>(index) < msg.buttons.size() &&
           msg.buttons[static_cast<std::size_t>(index)] != 0;
  }

  static double axisValue(const sensor_msgs::msg::Joy & msg, int index, bool & valid)
  {
    valid = index >= 0 && static_cast<std::size_t>(index) < msg.axes.size();
    if (!valid) {
      return 0.0;
    }
    return msg.axes[static_cast<std::size_t>(index)];
  }

  static bool isRisingEdge(bool current, bool previous)
  {
    return current && !previous;
  }

  std::string selectedJointName() const
  {
    if (selected_joint_index_ < joint_names_.size()) {
      return joint_names_[selected_joint_index_];
    }
    return "<invalid>";
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    last_joy_time_ = now();
    latest_joy_stamp_ = last_joy_time_;
    // Continuous mode consumes the latest joystick sample from the control timer.
    // Step mode still uses this callback's rising-edge detection below.
    latest_joy_ = msg;
    has_joy_ = true;

    const bool deadman_pressed = isButtonPressed(*msg, button_l1_);
    latest_deadman_pressed_ = deadman_pressed;
    const bool motion_enabled = !require_deadman_ || deadman_pressed;

    const bool a_pressed = isButtonPressed(*msg, button_a_);
    const bool b_pressed = isButtonPressed(*msg, button_b_);
    const bool x_pressed = isButtonPressed(*msg, button_x_);
    const bool y_pressed = isButtonPressed(*msg, button_y_);

    bool axis_valid = false;
    double axis_step_value = axisValue(*msg, axis_step_, axis_valid);
    if (!axis_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy axis_step index %d is out of range for axes size %zu.",
        axis_step_, msg->axes.size());
    }
    if (invert_axis_step_) {
      axis_step_value = -axis_step_value;
    }
    const bool axis_positive = axis_valid && axis_step_value > axis_step_threshold_;
    const bool axis_negative = axis_valid && axis_step_value < -axis_step_threshold_;

    RCLCPP_DEBUG(
      get_logger(),
      "Joy state: deadman_active=%s A=%d B=%d X=%d Y=%d axis_step_value=%.3f "
      "axis_positive=%d axis_negative=%d selected_joint_index=%zu selected_joint_name=%s",
      motion_enabled ? "true" : "false",
      a_pressed ? 1 : 0, b_pressed ? 1 : 0, x_pressed ? 1 : 0, y_pressed ? 1 : 0,
      axis_step_value, axis_positive ? 1 : 0, axis_negative ? 1 : 0,
      selected_joint_index_, selectedJointName().c_str());

    if (motion_enabled) {
      if (isRisingEdge(a_pressed, previous_a_pressed_)) {
        selectJoint(0);
      }
      if (isRisingEdge(b_pressed, previous_b_pressed_)) {
        selectJoint(1);
      }
      if (isRisingEdge(x_pressed, previous_x_pressed_)) {
        selectJoint(2);
      }
      if (isRisingEdge(y_pressed, previous_y_pressed_)) {
        selectJoint(3);
      }

      if (control_mode_ == "step") {
        // Compatibility path: one joystick edge still sends one FollowJointTrajectory goal.
        if (isRisingEdge(axis_positive, previous_axis_positive_)) {
          sendStepGoal(1.0);
        }
        if (isRisingEdge(axis_negative, previous_axis_negative_)) {
          sendStepGoal(-1.0);
        }
      }
    }

    previous_a_pressed_ = a_pressed;
    previous_b_pressed_ = b_pressed;
    previous_x_pressed_ = x_pressed;
    previous_y_pressed_ = y_pressed;
    previous_axis_positive_ = axis_positive;
    previous_axis_negative_ = axis_negative;
  }

  void controlTimerCallback()
  {
    if (control_mode_ != "continuous") {
      return;
    }
    if (!latest_joy_) {
      return;
    }

    const rclcpp::Time tick_time = now();
    const double joy_age_sec = (tick_time - latest_joy_stamp_).seconds();
    if (joy_age_sec > joy_timeout_) {
      q_command_initialized_ = false;
      last_control_time_ = tick_time;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy input timeout in continuous mode: latest Joy is %.3f seconds old, timeout is %.3f.",
        joy_age_sec, joy_timeout_);
      return;
    }

    if (require_deadman_ && !latest_deadman_pressed_) {
      q_command_initialized_ = false;
      last_control_time_ = tick_time;
      return;
    }
    if (!has_joint_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot publish continuous command before receiving valid joint states.");
      last_control_time_ = tick_time;
      return;
    }
    if (selected_joint_index_ >= joint_names_.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot publish continuous command because selected joint index is out of range.");
      last_control_time_ = tick_time;
      return;
    }

    bool axis_valid = false;
    double axis_step_value = axisValue(*latest_joy_, axis_step_, axis_valid);
    if (!axis_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy axis_step index %d is out of range for axes size %zu.",
        axis_step_, latest_joy_->axes.size());
      last_control_time_ = tick_time;
      return;
    }
    if (invert_axis_step_) {
      axis_step_value = -axis_step_value;
    }

    if (std::abs(axis_step_value) < axis_deadzone_) {
      // Keep the next continuous command anchored to controller feedback after the stick rests.
      q_command_ = q_current_;
      q_command_initialized_ = false;
      last_control_time_ = tick_time;
      return;
    }

    if (!q_command_initialized_) {
      q_command_ = q_current_;
      q_command_initialized_ = true;
    }

    if (q_command_.size() != joint_names_.size()) {
      q_command_ = q_current_;
    }

    const double command_state_error =
      std::abs(q_command_[selected_joint_index_] - q_current_[selected_joint_index_]);
    if (command_state_error > command_state_max_error_rad_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Continuous command state error %.3f rad exceeds limit %.3f rad; resetting command state.",
        command_state_error, command_state_max_error_rad_);
      q_command_ = q_current_;
    }

    // Use measured timer spacing so short scheduling delays do not change the commanded speed.
    double dt = 1.0 / control_rate_;
    if (last_control_time_.nanoseconds() > 0) {
      const double measured_dt = (tick_time - last_control_time_).seconds();
      if (measured_dt > 0.0) {
        dt = measured_dt;
      }
    }
    last_control_time_ = tick_time;

    // Integrate joystick deflection into a bounded position command for the selected joint.
    q_command_[selected_joint_index_] += axis_step_value * max_velocity_rad_s_ * dt;
    q_command_[selected_joint_index_] = std::clamp(
      q_command_[selected_joint_index_],
      joint_min_limits_[selected_joint_index_],
      joint_max_limits_[selected_joint_index_]);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;
    trajectory.points.resize(1);
    trajectory.points[0].positions = q_command_;
    trajectory.points[0].time_from_start = secondsToDurationMsg(command_duration_);

    trajectory_pub_->publish(trajectory);
    RCLCPP_DEBUG(
      get_logger(),
      "Published continuous command: selected_joint=%s axis=%.3f dt=%.3f q_command=%s",
      joint_names_[selected_joint_index_].c_str(), axis_step_value, dt,
      vectorToString(q_command_).c_str());
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (msg->name.size() != msg->position.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "JointState name size (%zu) does not match position size (%zu); ignoring message.",
        msg->name.size(), msg->position.size());
      return;
    }

    std::unordered_map<std::string, std::size_t> joint_index;
    joint_index.reserve(msg->name.size());
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      joint_index[msg->name[i]] = i;
    }

    std::vector<double> q_current(joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = joint_index.find(joint_names_[i]);
      if (it == joint_index.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "JointState on '%s' is missing required joint '%s'; ignoring this message. "
          "Available joints: %s",
          joint_states_topic_.c_str(), joint_names_[i].c_str(),
          vectorToStringNames(msg->name).c_str());
        return;
      }
      q_current[i] = msg->position[it->second];
    }

    q_current_ = std::move(q_current);
    has_joint_state_ = true;
  }

  void controllerStateCallback(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (msg->joint_names.size() != msg->actual.positions.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Controller state joint_names size (%zu) does not match actual.positions size (%zu); "
        "ignoring message.",
        msg->joint_names.size(), msg->actual.positions.size());
      return;
    }

    std::unordered_map<std::string, std::size_t> joint_index;
    joint_index.reserve(msg->joint_names.size());
    for (std::size_t i = 0; i < msg->joint_names.size(); ++i) {
      joint_index[msg->joint_names[i]] = i;
    }

    std::vector<double> q_current(joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto it = joint_index.find(joint_names_[i]);
      if (it == joint_index.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Controller state on '%s' is missing required joint '%s'; ignoring this message. "
          "Available joints: %s",
          controller_state_topic_.c_str(), joint_names_[i].c_str(),
          vectorToStringNames(msg->joint_names).c_str());
        return;
      }
      q_current[i] = msg->actual.positions[it->second];
    }

    q_current_ = std::move(q_current);
    has_joint_state_ = true;
  }

  void selectJoint(std::size_t index)
  {
    if (index >= joint_names_.size()) {
      RCLCPP_WARN(
        get_logger(), "Cannot select joint index %zu because joint_names has size %zu.",
        index, joint_names_.size());
      return;
    }

    selected_joint_index_ = index;
    RCLCPP_INFO(get_logger(), "Selected joint: %s", joint_names_[selected_joint_index_].c_str());
    RCLCPP_DEBUG(get_logger(), "Selected joint index: %zu", selected_joint_index_);
  }

  bool canSendMotionGoal() const
  {
    if (joint_names_.empty()) {
      RCLCPP_WARN(get_logger(), "Cannot send goal because joint_names is empty.");
      return false;
    }
    if (!has_joy_) {
      RCLCPP_WARN(get_logger(), "Cannot send goal before receiving Joy input.");
      return false;
    }

    const double age_sec = (now() - last_joy_time_).seconds();
    if (age_sec > joy_timeout_) {
      RCLCPP_WARN(
        get_logger(), "Joy input timeout: latest Joy is %.3f seconds old, timeout is %.3f.",
        age_sec, joy_timeout_);
      return false;
    }
    if (!has_joint_state_) {
      RCLCPP_WARN(get_logger(), "Cannot send goal before receiving valid joint states.");
      return false;
    }
    if (selected_joint_index_ >= joint_names_.size()) {
      RCLCPP_WARN(get_logger(), "Selected joint index is out of range.");
      return false;
    }
    if (goal_active_) {
      RCLCPP_WARN(get_logger(), "Previous FollowJointTrajectory goal is still active; skipping.");
      return false;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(
        get_logger(), "Action server '%s' is not available; skipping goal.",
        follow_joint_trajectory_action_.c_str());
      return false;
    }
    return true;
  }

  void sendStepGoal(double direction)
  {
    if (!canSendMotionGoal()) {
      return;
    }

    std::vector<double> q_target = q_current_;
    q_target[selected_joint_index_] += direction * step_rad_;
    q_target[selected_joint_index_] = std::clamp(
      q_target[selected_joint_index_],
      joint_min_limits_[selected_joint_index_],
      joint_max_limits_[selected_joint_index_]);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;
    trajectory.points.resize(1);
    trajectory.points[0].positions = q_target;
    trajectory.points[0].time_from_start = secondsToDurationMsg(trajectory_duration_);

    FollowJointTrajectory::Goal goal_msg;
    goal_msg.trajectory = trajectory;

    RCLCPP_INFO(
      get_logger(),
      "Sending step goal: selected_joint=%s direction=%.1f q_current=%s q_target=%s",
      joint_names_[selected_joint_index_].c_str(), direction,
      vectorToString(q_current_).c_str(), vectorToString(q_target).c_str());
    RCLCPP_DEBUG(
      get_logger(),
      "Goal detail: selected_joint_index=%zu selected_joint_name=%s direction=%.1f "
      "q_current=%s q_target=%s",
      selected_joint_index_, selectedJointName().c_str(), direction,
      vectorToString(q_current_).c_str(), vectorToString(q_target).c_str());

    goal_active_ = true;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
      std::bind(&TrunkJoystickTeleop::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(
        &TrunkJoystickTeleop::feedbackCallback, this, std::placeholders::_1,
        std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&TrunkJoystickTeleop::resultCallback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goalResponseCallback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      goal_active_ = false;
      RCLCPP_WARN(get_logger(), "FollowJointTrajectory goal was rejected by the action server.");
      return;
    }

    RCLCPP_INFO(get_logger(), "FollowJointTrajectory goal accepted by the action server.");
  }

  void feedbackCallback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback>)
  {
    RCLCPP_DEBUG(get_logger(), "Received FollowJointTrajectory feedback.");
  }

  void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
  {
    goal_active_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "FollowJointTrajectory goal succeeded.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "FollowJointTrajectory goal failed/aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "FollowJointTrajectory goal was canceled.");
        break;
      default:
        RCLCPP_WARN(get_logger(), "FollowJointTrajectory goal returned an unknown result code.");
        break;
    }
  }

  void logParameters() const
  {
    RCLCPP_INFO(get_logger(), "trunk_joystick_teleop parameters:");
    RCLCPP_INFO(get_logger(), "  joy_topic: %s", joy_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  joint_states_topic: %s", joint_states_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  use_controller_state: %s", use_controller_state_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  controller_state_topic: %s", controller_state_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "  follow_joint_trajectory_action: %s",
      follow_joint_trajectory_action_.c_str());
    RCLCPP_INFO(get_logger(), "  joint_trajectory_topic: %s", joint_trajectory_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  control_mode: %s", control_mode_.c_str());
    RCLCPP_INFO(get_logger(), "  control_rate: %.3f", control_rate_);
    RCLCPP_INFO(get_logger(), "  max_velocity_rad_s: %.3f", max_velocity_rad_s_);
    RCLCPP_INFO(get_logger(), "  command_duration: %.3f", command_duration_);
    RCLCPP_INFO(get_logger(), "  axis_deadzone: %.3f", axis_deadzone_);
    RCLCPP_INFO(
      get_logger(), "  command_state_max_error_rad: %.3f", command_state_max_error_rad_);
    RCLCPP_INFO(get_logger(), "  joint_names: %s", vectorToStringNames(joint_names_).c_str());
    RCLCPP_INFO(get_logger(), "  step_rad: %.9f", step_rad_);
    RCLCPP_INFO(get_logger(), "  trajectory_duration: %.3f", trajectory_duration_);
    RCLCPP_INFO(get_logger(), "  joy_timeout: %.3f", joy_timeout_);
    RCLCPP_INFO(get_logger(), "  require_deadman: %s", require_deadman_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "  mapping: A=%d B=%d X=%d Y=%d L1=%d axis_step=%d axis_step_threshold=%.3f "
      "invert_axis_step=%s",
      button_a_, button_b_, button_x_, button_y_, button_l1_,
      axis_step_, axis_step_threshold_, invert_axis_step_ ? "true" : "false");

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      RCLCPP_INFO(
        get_logger(), "  limit %s: [%.6f, %.6f]",
        joint_names_[i].c_str(), joint_min_limits_[i], joint_max_limits_[i]);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string joy_topic_;
  std::string joint_states_topic_;
  std::string controller_state_topic_;
  std::string follow_joint_trajectory_action_;
  std::string joint_trajectory_topic_;
  std::string control_mode_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;

  double control_rate_{20.0};
  double max_velocity_rad_s_{0.25};
  double command_duration_{0.15};
  double axis_deadzone_{0.15};
  double command_state_max_error_rad_{0.2};
  double step_rad_{0.034906585};
  double trajectory_duration_{0.5};
  double joy_timeout_{0.5};
  bool require_deadman_{true};
  bool use_controller_state_{true};

  int button_a_{0};
  int button_b_{1};
  int button_x_{3};
  int button_y_{4};
  int button_l1_{6};
  int axis_step_{1};
  double axis_step_threshold_{0.5};
  bool invert_axis_step_{false};

  std::vector<double> q_current_;
  std::vector<double> q_command_;
  std::size_t selected_joint_index_{0};
  bool q_command_initialized_{false};

  bool previous_a_pressed_{false};
  bool previous_b_pressed_{false};
  bool previous_x_pressed_{false};
  bool previous_y_pressed_{false};
  bool previous_axis_positive_{false};
  bool previous_axis_negative_{false};

  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_joy_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  bool latest_deadman_pressed_{false};
  bool has_joy_{false};
  bool has_joint_state_{false};
  bool goal_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrunkJoystickTeleop>());
  rclcpp::shutdown();
  return 0;
}
