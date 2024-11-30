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
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("low_pointing_limit", low_pointing_limit_);
  getInput("high_pointing_limit", high_pointing_limit_);
  getInput("threshold", threshold_);

  RCLCPP_INFO(node_->get_logger(), "Activating perception_people_detection");
  node_->add_activation("perception_system/perception_people_detection");
}

BT::NodeStatus IsPointing::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsPointing ticked while IDLE");
  }

  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(35);

  auto detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Processing %ld detections...", detections.size());

  // std::sort(
  //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
  //     return perception_system::diffIDs(this->person_id_, a.color_person) <
  //     perception_system::diffIDs(this->person_id_, b.color_person);
  //   });
  
  // Remove detections with low confidence
  for (auto it = detections.begin(); it != detections.end(); ) {
    auto const & detection = *it;
    if (detection.score <= threshold_) {
      RCLCPP_INFO(
        node_->get_logger(), "Removing detection %s due to low confidence (%f)",
          detection.class_name.c_str(), detection.score);
      it = detections.erase(it); // If low confidence, remove and go to next
    } else {
      ++it; // If kept, just go to the next
    }
  }

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections above confidence threshold");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection = detections[0];

  RCLCPP_INFO(node_->get_logger(), "Best detection pointing direction: %d",
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
      setOutput("detection", best_detection);
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

  // If someones is pointing to a valid direction, return SUCCESS and populate remaining output ports
  setOutput("output_frame", output_frame_);
  
  RCLCPP_INFO(node_->get_logger(), "Someone (%s) is pointing to a valid direction: %d", output_frame_.c_str(), direction);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsPointing>("IsPointing");
}
