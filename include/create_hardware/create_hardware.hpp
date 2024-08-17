#ifndef CREATE_HARDWARE__CREATE_HARDWARE_HPP_
#define CREATE_HARDWARE__CREATE_HARDWARE_HPP_

#include "create/create.h"
#include "create_hardware/visiblity_control.h"

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>

#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace create_hardware
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  JointValue state{};
  JointValue command{};
};

class CreateHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CreateHardware)

  CREATE_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CREATE_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CREATE_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CREATE_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CREATE_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CREATE_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  CREATE_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type reset_command();

  std::unique_ptr<create::Create> robot_;
  create::RobotModel model_ = create::RobotModel::CREATE_2;

  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  std::vector<float> joint_ratio_;
  std::vector<float> joint_offset_;

  bool use_dummy_{false};
};
}  // namespace create_hardware
#endif // CREATE_HARDWARE__CREATE_HARDWARE_HPP_
