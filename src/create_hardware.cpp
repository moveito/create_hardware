#include "create_hardware/create_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <exception>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("create_hardware");

namespace create_hardware
{
CallbackReturn CreateHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(LOGGER, "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);
  joint_ratio_.resize(info_.joints.size(), 1);
  joint_offset_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
  }

  if (info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end()
      && info_.hardware_parameters.at("use_dummy") == "true")
  {
    use_dummy_ = true;
    RCLCPP_INFO(LOGGER, "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  auto usb_port = info_.hardware_parameters.at("usb_port");
  auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const char * log = nullptr;

  RCLCPP_INFO(LOGGER, "usb_port: %s", usb_port.c_str());
  RCLCPP_INFO(LOGGER, "baud_rate: %d", baud_rate);

  model_ = create::RobotModel::CREATE_2;
  baud_rate = model_.getBaud();
  robot_ = std::make_unique<create::Create>(model_, false);
  robot_->setModeReportWorkaround(false);
  if (!robot_->connect(usb_port, baud_rate))
  {
    RCLCPP_FATAL(LOGGER, "%s", log);
    return CallbackReturn::ERROR;
  }

  robot_->setMode(create::MODE_FULL);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CreateHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(LOGGER, "export_state_interfaces");

  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CreateHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(LOGGER, "export_command_interfaces");

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn CreateHardware::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(LOGGER, "start");

  for (uint i = 0; i < joints_.size(); i++)
  {
    if (use_dummy_ && std::isnan(joints_[i].state.position))
    {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
    }
  }

  read(rclcpp::Time{}, rclcpp::Duration(0, 0));
  reset_command();
  write(rclcpp::Time{}, rclcpp::Duration(0, 0));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CreateHardware::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_DEBUG(LOGGER, "stop");
  return CallbackReturn::SUCCESS;
}

return_type CreateHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (use_dummy_)
  {
    return return_type::OK;
  }

  float wheelRadius = model_.getWheelDiameter() / 2.0;

  joints_[0].state.position = robot_->getLeftWheelDistance() / wheelRadius;
  joints_[0].state.velocity = robot_->getRequestedLeftWheelVel() / wheelRadius;
  joints_[1].state.position = robot_->getRightWheelDistance() / wheelRadius;
  joints_[1].state.velocity = robot_->getRequestedRightWheelVel() / wheelRadius;

  //RCLCPP_INFO(LOGGER, "left pos: %lf, left vel: %lf, right pos: %lf, right vel: %lf", joints_[0].state.position, joints_[0].state.velocity, joints_[1].state.position, joints_[1].state.velocity);
  
  return return_type::OK;
}

return_type CreateHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  //RCLCPP_INFO(LOGGER, "left: %lf, right: %lf", joints_[0].command.velocity, joints_[1].command.velocity);
  
  // command.velocity is rad/s
  // libcreate function need velocity in m/s
  float wheel_radius = model_.getWheelDiameter() / 2.0;
  float left_vel  = joints_[0].command.velocity * wheel_radius;
  float right_vel = joints_[1].command.velocity * wheel_radius;
  
  // send the velocity to create
  if (!robot_->driveWheels(left_vel, right_vel))
    RCLCPP_WARN(LOGGER, "drive wheel error");
  
  return return_type::OK;
}

return_type CreateHardware::reset_command()
{
  for (uint i = 0; i < joints_.size(); i++)
  {
    joints_[i].command.position = joints_[i].state.position;
    joints_[i].command.velocity = 0.0;
    joints_[i].command.effort = 0.0;
  }

  return return_type::OK;
}

}  // namespace create_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(create_hardware::CreateHardware, hardware_interface::SystemInterface)
