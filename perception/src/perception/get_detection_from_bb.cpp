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

#include "perception/get_from_blackboard.hpp"

namespace perception
{


GetDetectionFromBB::GetDetectionFromBB(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string what;
  

  getInput("key", key_);
  
}
BT::NodeStatus GetDetectionFromBB::tick()
{
  
  perception_system_interfaces::msg::Detection detection;

  config().blackboard->get(key_, detection);
  rclcpp::spin_some(node_->get_node_base_interface());
  
  setOutput("detection", std::make_shared<perception_system_interfaces::msg::Detection>(detection));
  return BT::NodeStatus::SUCCESS;
}

void GetDetectionFromBB::halt()
{
  perception_pub_->on_deactivate();
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::GetDetectionFromBB>("GetDetectionFromBB");
}
