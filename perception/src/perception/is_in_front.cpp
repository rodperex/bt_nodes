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

#include "perception/is_in_front.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

IsInFront::IsInFront(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  std::string model;
  
  config().blackboard->get("node", node_);

  getInput("confidence", confidence_);
  getInput("model", model);
  getInput("entity_to_identify", entity_);

  detection_at_input_ = true;
  if (!getInput("detection", detection_)) {
    detection_at_input_ = false;
  }

  if (model == "people") {
    node_->add_activation("perception_system/perception_people_detection");
  } else if (model == "object") {
    node_->add_activation("perception_system/perception_object_detection");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown model: %s. Activating generic", model.c_str());
    node_->add_activation("perception_system/perception_object_detection");
  }
}
BT::NodeStatus IsInFront::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "IsInFront ticked");

  std::vector<perception_system_interfaces::msg::Detection> detections;
  
  if (!detection_at_input_) {
    config().blackboard->get(entity_, detection_);
  }

  rclcpp::spin_some(node_->get_node_base_interface());
  
  detections = pl::getInstance(node_)->get_by_features(detection_, confidence_);

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections found");
    setOutput("direction", -1); // If no detections, just turn right bu default
    return BT::NodeStatus::FAILURE;
  }

  detection_ = detections[0];

  // double dx = detection.center3d.position.x;
  // double dy = detection.center3d.position.y;
  // double yaw = atan2(dx, -dy);
  double yaw = atan2(detection_.center3d.position.y, detection_.center3d.position.x);
  yaw = yaw * 180.0 / M_PI;

  if (std::abs((yaw)) > 5.0) { // If the angle is greater than 5 degrees, the detection is not in front
    if (yaw > 0) {
      setOutput("direction", 1); // Left
    } else {
      setOutput("direction", -1); // Right
    }
    return BT::NodeStatus::FAILURE;
  }

  // The detection is in front
  // Publish the detection
  pl::getInstance(node_)->publishTF_EKF(detection_, entity_, true);
  setOutput("direction", 0);
  return BT::NodeStatus::SUCCESS;
  

}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsInFront>("IsInFront");
}
