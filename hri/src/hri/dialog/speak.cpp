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
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hri/dialog/speak.hpp"

namespace dialog
{
using namespace std::chrono_literals;
using namespace std::placeholders;

Speak::Speak(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: hri::BtActionNode<audio_common_msgs::action::TTS, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);

  if (!node_->has_parameter("placeholder")) {
    node_->declare_parameter("placeholder", placeholder_);
  }
  node_->get_parameter("placeholder", placeholder_);

  RCLCPP_DEBUG(node_->get_logger(), "Placeholder: %s", placeholder_.c_str());

  speech_text_publisher_ = node_->create_publisher<std_msgs::msg::String>("speech_text", 10);
  speech_start_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_phase", 10);

  speech_text_publisher_->on_activate();
  speech_start_publisher_->on_activate();

}

std::string
Speak::swap_placeholders(std::string text, std::vector<std::string> elems)
{
  for (const auto& elem : elems) {
    size_t pos = text.find(placeholder_);
    if (pos != std::string::npos) {
      text.replace(pos, placeholder_.length(), elem);
      RCLCPP_DEBUG(node_->get_logger(), "Modified string: %s", text.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Placeholder %s not found in prompt", placeholder_.c_str());
      break;
    }
  }

  return text;
}


void
Speak::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "SPEAK");
  rclcpp::spin_some(node_->get_node_base_interface());
  
  
  goal_ = audio_common_msgs::action::TTS::Goal();
  std::string text;
  getInput("speech_text", text);

  std::vector<std::string> params;
  std::string sparams;
  getInput("params", sparams);

  std::stringstream ss(sparams);
  std::string item;
  std::vector<std::string> result;
  while (std::getline(ss, item, ';')) {
    RCLCPP_INFO(node_->get_logger(), "Param: %s", item.c_str());
    params.push_back(item);
  }
  RCLCPP_INFO(node_->get_logger(), "Text: %s", text.c_str());

  text = swap_placeholders(text, params);

  goal_.text = text;
  RCLCPP_INFO(node_->get_logger(), "Sending goal. Text: %s", goal_.text.c_str());

  auto speech_text_msg_ = std_msgs::msg::String();
  auto speech_start_msg_ = std_msgs::msg::Int8();

  speech_text_msg_.data = goal_.text;
  speech_start_msg_.data = START_SPEECH_;

  speech_text_publisher_->publish(speech_text_msg_);
  speech_start_publisher_->publish(speech_start_msg_);

}

BT::NodeStatus
Speak::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Speak  >(name, "say", config);
    };

  factory.registerBuilder<dialog::Speak>("Speak", builder);
}
