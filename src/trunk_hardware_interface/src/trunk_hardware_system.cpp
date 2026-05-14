#include "trunk_hardware_interface/trunk_hardware_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace trunk_hardware_interface
{

namespace
{

const rclcpp::Logger LOGGER = rclcpp::get_logger("TrunkHardwareSystem");

bool containsInterface(
  const std::vector<hardware_interface::InterfaceInfo>& interfaces,
  const std::string& name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo& interface) {
      return interface.name == name;
    });
}

}  // namespace

hardware_interface::CallbackReturn TrunkHardwareSystem::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(
      LOGGER, "Expected %zu trunk joints, got %zu.", kJointCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!parseHardwareParameters() || !parseJointParameters() || !validateInterfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    LOGGER,
    "Initialized trunk hardware interface. transport=%s dry_run=%s port=%s can_interface=%s ip=%s",
    transport_.c_str(), dry_run_ ? "true" : "false", port_.c_str(),
    can_interface_.c_str(), ip_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TrunkHardwareSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount * 2);
  for (auto& joint : joints_) {
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position);
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
TrunkHardwareSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);
  for (auto& joint : joints_) {
    interfaces.emplace_back(
      joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
  }
  return interfaces;
}

hardware_interface::CallbackReturn TrunkHardwareSystem::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  for (auto& joint : joints_) {
    joint.command_position = joint.state_position;
    joint.previous_command_position = joint.command_position;
  }
  active_ = true;
  warned_dry_run_ = false;
  RCLCPP_INFO(LOGGER, "Trunk hardware activated; commands synchronized to current state.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TrunkHardwareSystem::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  active_ = false;
  RCLCPP_WARN(LOGGER, "Trunk hardware deactivated; motor outputs should be disabled by driver.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TrunkHardwareSystem::read(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  return readHardware(time, period);
}

hardware_interface::return_type TrunkHardwareSystem::write(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  if (hasTimedOut(time)) {
    RCLCPP_ERROR(
      LOGGER, "Command timeout exceeded (%d ms); refusing to write motor commands.",
      command_timeout_ms_);
    return hardware_interface::return_type::ERROR;
  }

  const double period_sec = std::max(1e-6, period.seconds());
  for (const auto& joint : joints_) {
    if (!isCommandSafe(joint, period_sec)) {
      return hardware_interface::return_type::ERROR;
    }
  }

  const auto result = writeHardware(time, period);
  if (result == hardware_interface::return_type::OK) {
    last_write_time_ = time;
    for (auto& joint : joints_) {
      joint.previous_command_position = joint.command_position;
    }
  }
  return result;
}

bool TrunkHardwareSystem::parseHardwareParameters()
{
  transport_ = getStringParam(info_.hardware_parameters, "transport", transport_);
  port_ = getStringParam(info_.hardware_parameters, "port", "");
  can_interface_ = getStringParam(info_.hardware_parameters, "can_interface", "");
  ip_ = getStringParam(info_.hardware_parameters, "ip", "");
  baudrate_ = getIntParam(info_.hardware_parameters, "baudrate", baudrate_);
  command_timeout_ms_ = getIntParam(
    info_.hardware_parameters, "command_timeout_ms", command_timeout_ms_);
  require_homing_ = getBoolParam(info_.hardware_parameters, "require_homing", require_homing_);
  require_absolute_encoder_ = getBoolParam(
    info_.hardware_parameters, "require_absolute_encoder", require_absolute_encoder_);
  dry_run_ = getBoolParam(info_.hardware_parameters, "dry_run", transport_ == "dry_run");

  if (command_timeout_ms_ <= 0) {
    RCLCPP_ERROR(LOGGER, "command_timeout_ms must be positive.");
    return false;
  }
  if (!dry_run_ && transport_ == "dry_run") {
    RCLCPP_ERROR(LOGGER, "transport=dry_run cannot be used with dry_run=false.");
    return false;
  }
  return true;
}

bool TrunkHardwareSystem::parseJointParameters()
{
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const auto& joint_info = info_.joints[i];
    auto& joint = joints_[i];
    joint.name = joint_info.name;
    joint.motor_id = getIntParam(joint_info.parameters, "motor_id", static_cast<int>(i + 1));
    joint.gear_ratio = getDoubleParam(joint_info.parameters, "gear_ratio", 1.0);
    joint.zero_offset = getDoubleParam(joint_info.parameters, "zero_offset", 0.0);
    joint.direction_sign = getDoubleParam(joint_info.parameters, "direction_sign", 1.0);
    joint.min_position = getDoubleParam(joint_info.parameters, "min_position", -3.14);
    joint.max_position = getDoubleParam(joint_info.parameters, "max_position", 3.14);
    joint.max_velocity = getDoubleParam(joint_info.parameters, "max_velocity", 0.2);
    joint.max_position_error = getDoubleParam(joint_info.parameters, "max_position_error", 0.2);
    joint.state_position = getDoubleParam(joint_info.parameters, "initial_position", 0.0);
    joint.state_velocity = 0.0;
    joint.command_position = joint.state_position;
    joint.previous_command_position = joint.command_position;

    if (joint.gear_ratio <= 0.0) {
      RCLCPP_ERROR(LOGGER, "Joint %s gear_ratio must be positive.", joint.name.c_str());
      return false;
    }
    if (std::abs(joint.direction_sign) != 1.0) {
      RCLCPP_ERROR(LOGGER, "Joint %s direction_sign must be 1 or -1.", joint.name.c_str());
      return false;
    }
    if (joint.min_position >= joint.max_position) {
      RCLCPP_ERROR(LOGGER, "Joint %s has invalid position limits.", joint.name.c_str());
      return false;
    }
    if (joint.max_velocity <= 0.0 || joint.max_position_error <= 0.0) {
      RCLCPP_ERROR(
        LOGGER, "Joint %s max_velocity and max_position_error must be positive.",
        joint.name.c_str());
      return false;
    }
  }
  return true;
}

bool TrunkHardwareSystem::validateInterfaces() const
{
  for (const auto& joint : info_.joints) {
    if (!containsInterface(joint.command_interfaces, hardware_interface::HW_IF_POSITION)) {
      RCLCPP_ERROR(LOGGER, "Joint %s must expose position command interface.", joint.name.c_str());
      return false;
    }
    if (!containsInterface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !containsInterface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        LOGGER, "Joint %s must expose position and velocity state interfaces.",
        joint.name.c_str());
      return false;
    }
  }
  return true;
}

bool TrunkHardwareSystem::hasTimedOut(const rclcpp::Time& time) const
{
  if (last_write_time_.nanoseconds() == 0) {
    return false;
  }
  const double age_ms = (time - last_write_time_).seconds() * 1000.0;
  return age_ms > static_cast<double>(command_timeout_ms_);
}

bool TrunkHardwareSystem::isCommandSafe(const JointConfig& joint, double period_sec) const
{
  if (!std::isfinite(joint.command_position)) {
    RCLCPP_ERROR(LOGGER, "Joint %s received non-finite command.", joint.name.c_str());
    return false;
  }
  if (joint.command_position < joint.min_position || joint.command_position > joint.max_position) {
    RCLCPP_ERROR(
      LOGGER, "Joint %s command %.6f outside soft limits [%.6f, %.6f].",
      joint.name.c_str(), joint.command_position, joint.min_position, joint.max_position);
    return false;
  }
  const double requested_velocity =
    std::abs(joint.command_position - joint.previous_command_position) / period_sec;
  if (requested_velocity > joint.max_velocity) {
    RCLCPP_ERROR(
      LOGGER, "Joint %s requested velocity %.6f exceeds limit %.6f rad/s.",
      joint.name.c_str(), requested_velocity, joint.max_velocity);
    return false;
  }
  const double tracking_error = std::abs(joint.command_position - joint.state_position);
  if (tracking_error > joint.max_position_error) {
    RCLCPP_ERROR(
      LOGGER, "Joint %s position error %.6f exceeds limit %.6f rad.",
      joint.name.c_str(), tracking_error, joint.max_position_error);
    return false;
  }
  return true;
}

hardware_interface::return_type TrunkHardwareSystem::readHardware(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  (void)time;
  const double period_sec = std::max(1e-6, period.seconds());

  if (dry_run_) {
    if (!warned_dry_run_) {
      RCLCPP_WARN(
        LOGGER,
        "TrunkHardwareSystem is running in dry_run mode. Replace readHardware/writeHardware "
        "with the real motor bus driver before controlling a robot.");
      warned_dry_run_ = true;
    }
    for (auto& joint : joints_) {
      const double previous_position = joint.state_position;
      joint.state_position = joint.command_position;
      joint.state_velocity = (joint.state_position - previous_position) / period_sec;
    }
    return hardware_interface::return_type::OK;
  }

  // TODO(wxl): Read motor encoder position/velocity from serial/CAN/EtherCAT/IP transport here.
  RCLCPP_ERROR_ONCE(
    LOGGER,
    "Real trunk hardware read path is not connected. Implement readHardware() for transport '%s'.",
    transport_.c_str());
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type TrunkHardwareSystem::writeHardware(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  (void)time;
  (void)period;
  if (dry_run_) {
    return hardware_interface::return_type::OK;
  }

  // TODO(wxl): Convert joint command to motor units using direction/offset/gear ratio and send it.
  RCLCPP_ERROR_ONCE(
    LOGGER,
    "Real trunk hardware write path is not connected. Implement writeHardware() for transport '%s'.",
    transport_.c_str());
  return hardware_interface::return_type::ERROR;
}

std::string TrunkHardwareSystem::getStringParam(
  const std::unordered_map<std::string, std::string>& params,
  const std::string& name,
  const std::string& default_value)
{
  const auto it = params.find(name);
  return it == params.end() ? default_value : it->second;
}

double TrunkHardwareSystem::getDoubleParam(
  const std::unordered_map<std::string, std::string>& params,
  const std::string& name,
  double default_value)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return default_value;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception&) {
    RCLCPP_WARN(LOGGER, "Invalid double parameter %s='%s'; using %.6f.",
      name.c_str(), it->second.c_str(), default_value);
    return default_value;
  }
}

int TrunkHardwareSystem::getIntParam(
  const std::unordered_map<std::string, std::string>& params,
  const std::string& name,
  int default_value)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return default_value;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception&) {
    RCLCPP_WARN(LOGGER, "Invalid int parameter %s='%s'; using %d.",
      name.c_str(), it->second.c_str(), default_value);
    return default_value;
  }
}

bool TrunkHardwareSystem::getBoolParam(
  const std::unordered_map<std::string, std::string>& params,
  const std::string& name,
  bool default_value)
{
  const auto it = params.find(name);
  if (it == params.end()) {
    return default_value;
  }
  return it->second == "true" || it->second == "1" || it->second == "True";
}

}  // namespace trunk_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  trunk_hardware_interface::TrunkHardwareSystem,
  hardware_interface::SystemInterface)
