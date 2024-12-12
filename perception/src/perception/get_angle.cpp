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

#include "perception/get_angle.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

GetAngle::GetAngle(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  std::string what;
  
  config().blackboard->get("node", node_);
  getInput("base_frame", base_frame_);
  getInput("tf_frame", tf_frame_);
  getInput("target", target_);
  getInput("conficende", confidence_);
  getInput("what", what);

  if (what == "person") {
    node_->add_activation("perception_system/perception_people_detection");
  } else if (what == "object") {
    node_->add_activation("perception_system/perception_object_detection");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[IsDetected] Unknown what: %s. Activating generic", what.c_str());
    node_->add_activation("perception_system/perception_object_detection");
  }
}
BT::NodeStatus GetAngle::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "GetAngle ticked");
  rclcpp::spin_some(node_->get_node_base_interface());

  std::vector<perception_system_interfaces::msg::Detection> detections;
  perception_system_interfaces::msg::Detection detection;
  
  config().blackboard->get(target_, detection);

  
  detections = pl::getInstance(node_)->get_by_features(detection, confidence_);


  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections found");
    return BT::NodeStatus::FAILURE;
  }

  // double dx = detection.center3d.position.x;
  // double dy = detection.center3d.position.y;
  // double yaw = atan2(dx, -dy);
  double yaw = atan2(detection.center3d.position.y, detection.center3d.position.x);

  setOutput("yaw", yaw);
  if (yaw > 0) {
    setOutput("direction", 1);
  } else {
    setOutput("direction", -1);
  }
  
  return BT::NodeStatus::SUCCESS;
}

void GetAngle::halt()
{
  RCLCPP_DEBUG(node_->get_logger(), "GetAngle halted");
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::GetAngle>("GetAngle");
}
