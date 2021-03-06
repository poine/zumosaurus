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

#ifndef ZUMOSAURUS_CONTROLLERS__SKEL_CUSTOM_CONTROLLER_HPP_
#define ZUMOSAURUS_CONTROLLERS__SKEL_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "zumosaurus_controllers/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace zumosaurus_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SkelCustomController : public controller_interface::ControllerInterface
{
public:
  ZUMOSAURUS_CONTROLLERS_PUBLIC
  SkelCustomController();

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ZUMOSAURUS_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::string interface_name_;

  // TODO(anyone): replace the state and command message types
  // Command subscribers and Controller State publisher
  using ControllerCommandMsg = control_msgs::msg::JointJog;

  rclcpp::Subscription<ControllerCommandMsg>::SharedPtr command_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerCommandMsg>> input_command_;

  using ControllerStateMsg = control_msgs::msg::JointControllerState;
  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;
};

}  // namespace zumosaurus_controllers

#endif  // ZUMOSAURUS_CONTROLLERS__SKEL_CUSTOM_CONTROLLER_HPP_
