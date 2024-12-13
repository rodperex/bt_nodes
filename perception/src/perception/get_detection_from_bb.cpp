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

#include "perception/get_detection_from_bb.hpp"

namespace perception
{


GetDetectionFromBB::GetDetectionFromBB(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  getInput("key", key_);
}
BT::NodeStatus GetDetectionFromBB::tick()
{
  
  std::shared_ptr<perception_system_interfaces::msg::Detection> detection_ptr;

  config().blackboard->get(key_, detection_ptr);
  
  setOutput("detection", detection_ptr);
  return BT::NodeStatus::SUCCESS;
}

void GetDetectionFromBB::halt()
{
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::GetDetectionFromBB>("GetDetectionFromBB");
}
