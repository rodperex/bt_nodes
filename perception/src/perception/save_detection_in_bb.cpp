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

#include "perception/save_detection_in_bb.hpp"

namespace perception
{


SaveDetectionInBB::SaveDetectionInBB(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  
  getInput("key", key_);
  getInput("detection", detection_);
  
}
BT::NodeStatus SaveDetectionInBB::tick()
{
  config().blackboard->set("detection", std::make_shared<perception_system_interfaces::msg::Detection>(detection_));
  RCLCPP_INFO(rclcpp::get_logger("bt_node"), "Detection saved in blackboard");
  return BT::NodeStatus::SUCCESS;
}

void SaveDetectionInBB::halt()
{
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::SaveDetectionInBB>("SaveDetectionInBB");
}
