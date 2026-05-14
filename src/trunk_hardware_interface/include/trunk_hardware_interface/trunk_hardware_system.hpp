#pragma once

#include <array>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace trunk_hardware_interface
{

class TrunkHardwareSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TrunkHardwareSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  struct JointConfig
  {
    std::string name;
    int motor_id = -1;
    double gear_ratio = 1.0;
    double zero_offset = 0.0;
    double direction_sign = 1.0;
    double min_position = -3.14;
    double max_position = 3.14;
    double max_velocity = 0.2;
    double max_position_error = 0.2;
    double state_position = 0.0;
    double state_velocity = 0.0;
    double command_position = 0.0;
    double previous_command_position = 0.0;
  };

  static constexpr std::size_t kJointCount = 4;

  bool parseHardwareParameters();
  bool parseJointParameters();
  bool validateInterfaces() const;
  bool hasTimedOut(const rclcpp::Time& time) const;
  bool isCommandSafe(const JointConfig& joint, double period_sec) const;

  hardware_interface::return_type readHardware(
    const rclcpp::Time& time,
    const rclcpp::Duration& period);
  hardware_interface::return_type writeHardware(
    const rclcpp::Time& time,
    const rclcpp::Duration& period);

  static std::string getStringParam(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& name,
    const std::string& default_value);
  static double getDoubleParam(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& name,
    double default_value);
  static int getIntParam(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& name,
    int default_value);
  static bool getBoolParam(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& name,
    bool default_value);

  std::array<JointConfig, kJointCount> joints_;
  std::string transport_ = "dry_run";
  std::string port_;
  std::string can_interface_;
  std::string ip_;
  int baudrate_ = 1000000;
  int command_timeout_ms_ = 100;
  bool require_homing_ = true;
  bool require_absolute_encoder_ = true;
  bool dry_run_ = true;
  bool active_ = false;
  bool warned_dry_run_ = false;
  rclcpp::Time last_write_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace trunk_hardware_interface
