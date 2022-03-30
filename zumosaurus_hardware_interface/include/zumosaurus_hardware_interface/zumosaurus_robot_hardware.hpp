// Copyright (c) 2022, poine
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZUMOSAURUS_HARDWARE_INTERFACE__ZUMOSAURUS_ROBOT_HARDWARE_HPP_
#define ZUMOSAURUS_HARDWARE_INTERFACE__ZUMOSAURUS_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "zumosaurus_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace zumosaurus_hardware_interface
{
class ZumosaurusRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZumosaurusRobotHardware)

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  ZUMOSAURUS_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace zumosaurus_hardware_interface

#endif  // ZUMOSAURUS_HARDWARE_INTERFACE__ZUMOSAURUS_ROBOT_HARDWARE_HPP_
