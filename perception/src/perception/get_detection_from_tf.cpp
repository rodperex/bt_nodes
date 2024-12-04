// Copyright 2024 Rodrigo Pérez-Rodríguez
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


#include "perception/get_detection_from_tf.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

GetDetectionFromTF::GetDetectionFromTF(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  getInput("frame", frame_);
}

BT::NodeStatus
GetDetectionFromTF::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "GET_DETECTION_FROM_TF");

  auto base2entity_msg = tf_buffer_.lookupTransform("base_link", frame_, tf2::TimePointZero);
  
  pl::getInstance(node_)->update(30);
  auto detections = pl::getInstance(node_)->get_detection_at(base2entity_msg);

  if (detections.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No detection found at (%.2f, %.2f, %.2f) from base_link",
      base2entity_msg.transform.translation.x,
      base2entity_msg.transform.translation.y,
      base2entity_msg.transform.translation.z);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("detection", std::make_shared<perception_system_interfaces::msg::Detection>(detections[0]));

  return BT::NodeStatus::SUCCESS;  
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::GetDetectionFromTF>("GetDetectionFromTF");
}

