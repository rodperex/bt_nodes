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

#include "attention/deactivate_attention.hpp"

namespace attention
{


DeactivateAttention::DeactivateAttention(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string what;
  
  config().blackboard->get("node", node_);

  getInput("what", what);
  if (what == "base") {
    node_->remove_activation("attention_system_base");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown what: %s. Deactivating generic", what.c_str());
    node_->remove_activation("attention_system_base");
  }

  attention_pub_ = node_->create_publisher<std_msgs::msg::String>("attention_system", 10);
  attention_pub_->on_activate();
}
BT::NodeStatus DeactivateAttention::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ACTIVATE_ATTENTION");
  
  std_msgs::msg::String msg;
  msg.data = "";
  attention_pub_->publish(msg);
  
  rclcpp::spin_some(node_->get_node_base_interface());
  
  return BT::NodeStatus::SUCCESS;
}

void DeactivateAttention::halt()
{
  attention_pub_->on_deactivate();
}

}  // namespace attention

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<attention::DeactivateAttention>("DeactivateAttention");
}

