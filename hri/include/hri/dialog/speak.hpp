// Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

#ifndef DIALOG__Speak_HPP_
#define DIALOG__Speak_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <iostream>
#include <sstream>

#include "audio_common_msgs/action/tts.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace dialog
{

class Speak : public hri::BtActionNode<
    audio_common_msgs::action::TTS, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Speak(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("speech_text"),
        BT::InputPort<std::string>("params")
      });
  }

private:
  BT::NodeStatus on_idle();
  std::string swap_placeholders(std::string text, std::vector<std::string> elems);


  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr speech_text_publisher_; // To publish the text to be spoken
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr speech_start_publisher_; // To indicate the start of the speech

  std::string placeholder_{"[]"};
  
  const int START_SPEECH_{1};
};

}  // namespace dialog

#endif  // HRI__Speak_HPP_
