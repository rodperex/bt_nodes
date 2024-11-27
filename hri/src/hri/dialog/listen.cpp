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
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hri/dialog/listen.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Listen::Listen(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: hri::BtActionNode<whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  listen_start_publisher_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_phase", 10);
}

void
Listen::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Listen ticked");
  goal_ = whisper_msgs::action::STT::Goal();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = START_LISTENING_;

  listen_start_publisher_->publish(msg_dialog_action);
}

BT::NodeStatus Listen::on_success()
{

  RCLCPP_INFO(node_->get_logger(), "I heard: %s", result_.result->transcription.text.c_str());

  if (result_.result->transcription.text.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("listened_text", result_.result->transcription.text);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Listen>(name, "whisper/listen", config);
    };

  factory.registerBuilder<dialog::Listen>("Listen", builder);
}
