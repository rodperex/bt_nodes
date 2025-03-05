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
  rclcpp::spin_some(node_->get_node_base_interface());
  std::vector<perception_system_interfaces::msg::Detection> detections;

  if (!getInput("detection", detection_)) {
    RCLCPP_ERROR(node_->get_logger(), "No detection at input");
    return BT::NodeStatus::FAILURE;
  }

  pl::getInstance(node_)->update(30);
  pl::getInstance(node_)->set_interest("person", true);
  // detections = pl::getInstance(node_)->get_by_features(*detection_, confidence_);
  detections = pl::getInstance(node_)->get_by_type(detection_->type); //Until getting by features works

  // save_detection_tf_to_bb(detection_, "map", entity_);

  if (detections.empty()) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "Perception system did not identify %s", entity_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // std::sort(
  //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
  //     return a.score > b.score;
  // });

  // Publish the detection
  RCLCPP_DEBUG(node_->get_logger(), "[IDENTIFY]: Detected %s with confidence %.2f", entity_.c_str(), detections[0].score);
  pl::getInstance(node_)->publishTF_EKF(detections[0], entity_, true);
  detections.clear();
  return BT::NodeStatus::SUCCESS;  
}

void
Identify::save_detection_tf_to_bb(std::shared_ptr<perception_system_interfaces::msg::Detection> detection, std::string source_frame, std::string child_frame)
{
  std::shared_ptr<geometry_msgs::msg::TransformStamped> map2person_msg_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();  
  
  map2person_msg_ptr->header.stamp = detection->header.stamp;
  map2person_msg_ptr->header.frame_id = source_frame;
  map2person_msg_ptr->child_frame_id = child_frame;
  map2person_msg_ptr->transform.translation.x = detection->center3d.position.x;
  map2person_msg_ptr->transform.translation.y = detection->center3d.position.y;
  map2person_msg_ptr->transform.translation.z = detection->center3d.position.z;
  map2person_msg_ptr->transform.rotation.x = 0.0;
  map2person_msg_ptr->transform.rotation.y = 0.0;
  map2person_msg_ptr->transform.rotation.z = 0.0;
  map2person_msg_ptr->transform.rotation.w = 1.0;
  
  config().blackboard->set("last_map2person", map2person_msg_ptr);
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::Identify>("Identify");
}

