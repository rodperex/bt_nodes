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


#include "perception/identify.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

Identify::Identify(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  entity_(""),
  confidence_(0.2)
{
  config().blackboard->get("node", node_);

  getInput("entity_to_identify", entity_);
  RCLCPP_INFO(node_->get_logger(), "Identifying %s", entity_.c_str());

  if (!getInput("confidence", confidence_)) {
    RCLCPP_WARN(node_->get_logger(), "No confidence provided. Using default value %f", confidence_);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Confidence threshold: %.2f", confidence_);
  }
  
}

BT::NodeStatus
Identify::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "IDENTIFY");
  std::vector<perception_system_interfaces::msg::Detection> detections;

  if (!getInput("detection", detection_)) {
    RCLCPP_ERROR(node_->get_logger(), "No detection at input");
    return BT::NodeStatus::FAILURE;
  }

  pl::getInstance(node_)->update(30);
  detections = pl::getInstance(node_)->get_by_features(*detection_, confidence_);

  if (detections.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Perception system did not identify %s", entity_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::sort(
    detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
      return a.score > b.score;
  });

  // Publish the detection
  RCLCPP_INFO(node_->get_logger(), "Perception system detected %s with confidence %f. Publishing TF", entity_.c_str(), detections[0].score);
  pl::getInstance(node_)->publishTF_EKF(detections[0], entity_, true);

  return BT::NodeStatus::SUCCESS;  
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::Identify>("Identify");
}

