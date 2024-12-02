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
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
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
    RCLCPP_INFO(node_->get_logger(), "Getting detection from blackboard to check if it is in front");
    config().blackboard->get<std::shared_ptr<perception_system_interfaces::msg::Detection>>(entity_, detection_);
  }

  rclcpp::spin_some(node_->get_node_base_interface());
  
  detections = pl::getInstance(node_)->get_by_features(*detection_, confidence_);

  // Update the blackboard with the detection
  RCLCPP_INFO(node_->get_logger(), "Updating blackboard with detection of %s", entity_.c_str());
  config().blackboard->set<std::shared_ptr<perception_system_interfaces::msg::Detection>>(entity_, std::make_shared<perception_system_interfaces::msg::Detection>(detections[0]));

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections found");
    setOutput("direction", -1.0); // If no detections, just turn right bu default
    return BT::NodeStatus::FAILURE;
  }

  detection_ = std::make_shared<perception_system_interfaces::msg::Detection>(detections[0]);

  geometry_msgs::msg::TransformStamped ps2detection_msg, bf2ps_msg;
  tf2::Stamped<tf2::Transform> bf2ps, ps2detection;
  ps2detection_msg.transform.translation.x = detection_->center3d.position.x; // Perception system to detection
  ps2detection_msg.transform.translation.y = detection_->center3d.position.y;
  ps2detection_msg.transform.translation.z = detection_->center3d.position.z;
  ps2detection_msg.header.frame_id = detection_->header.frame_id;

  bf2ps_msg = tf_buffer_.lookupTransform("base_footprint", detection_->header.frame_id, tf2::TimePointZero);
  
  tf2::fromMsg(bf2ps_msg, bf2ps);
  tf2::fromMsg(ps2detection_msg, ps2detection);
  tf2::Transform bf2detection = bf2ps * ps2detection;

  // double yaw = tf2::getYaw(bf2detection.getRotation());
  double yaw = std::atan2(bf2detection.getOrigin().y(), bf2detection.getOrigin().x());

  yaw = yaw * 180.0 / M_PI;
  RCLCPP_DEBUG(node_->get_logger(), "Detection at (%.2f, %.2f, %.2f) from %s",
    detection_->center3d.position.x,
    detection_->center3d.position.y,
    detection_->center3d.position.z,
    detection_->header.frame_id.c_str());
  RCLCPP_DEBUG(node_->get_logger(), "Detection at (%.2f, %.2f, %.2f) from base_footprint. YAW: %.2f degrees",
    bf2detection.getOrigin().x(),
    bf2detection.getOrigin().y(),
    bf2detection.getOrigin().z(),
    yaw);

  if (std::abs((yaw)) > 5.0) { // If the angle is greater than 5 degrees, the detection is not in front
    if (yaw > 0) {
      RCLCPP_INFO(node_->get_logger(), "Detection is at %.2f degrees to the left", yaw);
      setOutput("direction", 1.0); // Left
    } else {
      RCLCPP_INFO(node_->get_logger(), "Detection is at %.2f degrees to the right", yaw);
      setOutput("direction", -1.0); // Right
    }
    return BT::NodeStatus::FAILURE;
  }

  // The detection is in front. Publish the detection
  RCLCPP_INFO(node_->get_logger(), "Detection is in front");
  pl::getInstance(node_)->publishTF_EKF(*detection_, entity_, true);
  setOutput("direction", 0.0);
  return BT::NodeStatus::SUCCESS;
  

}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsInFront>("IsInFront");
}
