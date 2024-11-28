// Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

#include "perception/is_pointing.hpp"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsPointing::IsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_()
{
  config().blackboard->get("node", node_);

  getInput("camera_frame", camera_frame_);
  getInput("low_pointing_limit", low_pointing_limit_);
  getInput("high_pointing_limit", high_pointing_limit_);
  getInput("output_frame", output_frame_);

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus IsPointing::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "IsPointing ticked while IDLE");
  }

  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(true);
  rclcpp::spin_some(node_->get_node_base_interface());

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;

  // std::sort(
  //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
  //     return perception_system::diffIDs(this->person_id_, a.color_person) <
  //     perception_system::diffIDs(this->person_id_, b.color_person);
  //   });

  best_detection = detections[0];

  RCLCPP_INFO(
    node_->get_logger(), "Best detection: %s, color: %ld, pointing: %d",
    best_detection.unique_id.c_str(), best_detection.color_person,
    best_detection.pointing_direction);

  int direction = -1;
  for (int i = low_pointing_limit_; i <= high_pointing_limit_; i++) {
    RCLCPP_INFO(
      node_->get_logger(), "Pointing direction %d",
      best_detection.pointing_direction);
    if (best_detection.pointing_direction == i) {
      direction = i;
      RCLCPP_INFO(node_->get_logger(), "I like the direction the person is pointing at: %d", direction);
      setOutput("pointing_direction", direction);
      if (pl::getInstance(node_)->publishTF_EKF(best_detection, output_frame_, true) == -1) {
        return BT::NodeStatus::FAILURE;
      }
      break;
    }
  }

  if (direction == -1) { // No match
    RCLCPP_ERROR(node_->get_logger(), "Invalid pointing direction");
    return BT::NodeStatus::FAILURE;
  }

  // If someones is pointing, we return SUCCESS and populate output ports
  setOutput("detection", best_detection);
  setOutput("output_frame", output_frame_);
  setOutput("pointing_direction", direction);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsPointing>("IsPointing");
}
