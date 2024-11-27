// Copyright 2024 Intelligent Robotics Lab
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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__DIALOGCONFIRMATION_HPP_
#define HRI__DIALOGCONFIRMATION_HPP_

#include <algorithm>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"

#include "std_msgs/msg/int8.hpp"

namespace hri
{

class DialogConfirmation
  : public hri::BtActionNode<
    whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit DialogConfirmation(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("language"), // es/en
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr speech_start_publisher_;

  std::string lang_;
  
  const int START_LISTENING_{0};
};

}  // namespace dialog

#endif  // HRI__DIALOGCONFIRMATION_HPP_
