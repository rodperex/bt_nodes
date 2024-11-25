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

#include "perception/IsDetected.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDetected::IsDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  max_depth_(std::numeric_limits<double>::max()),
  max_entities_(1)
{
  std::string model;
  config().blackboard->get("node", node_);
  config().blackboard->get("model", model);

  if (model == "person") {
    node_->add_activation("perception_system/perception_people_detection");
  } else if (model == "object") {
    node_->add_activation("perception_system/perception_object_detection");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown model: %s. Activating generic", model.c_str());
    node_->add_activation("perception_system/perception_object_detection");
  }
  
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  
  getInput("interest", interest_);
  getInput("cam_frame", cam_frame_);
  getInput("confidence", threshold_);
  getInput("max_entities", max_entities_);
  getInput("order", order_);
  getInput("max_depth", max_depth_);
  // getInput("person_id", person_id_);

  RCLCPP_INFO(node_->get_logger(), "Interest: %s", interest_.c_str());

  frames_.clear();
}

BT::NodeStatus IsDetected::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked while IDLE");
  }

  RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked");

  pl::getInstance(node_)->set_interest(interest_, true);
  pl::getInstance(node_)->update(35);

  auto detections = pl::getInstance(node_)->get_by_type(interest_);

  if (detections.empty()) {
    // RCLCPP_WARN(node_->get_logger(), "No detections");
    RCLCPP_DEBUG(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Processing %ld detections...", detections.size());

  if (order_ == "color") {
    // sorted by the distance to the color person we should sort it by distance and also by left to right or right to left
    // RCLCPP_DEBUG(node_->get_logger(), "Sorting detections by color");
    // std::sort(
    //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
    //     return perception_system::diffIDs(this->person_id_, a.color_person) <
    //     perception_system::diffIDs(this->person_id_, b.color_person);
    //   });
  } else if (order_ == "depth") {
    RCLCPP_DEBUG(node_->get_logger(), "Sorting detections by depth");
    std::sort(
      detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
        return a.center3d.position.z < b.center3d.position.z;
      });
  }
  // auto pub = node_->create_publisher<sensor_msgs::msg::Image>(
  //   "/object_detected", 10);

  // pub->publish(detections[0].image);


  RCLCPP_DEBUG(node_->get_logger(), "Max Depth: %f", max_depth_);
  RCLCPP_DEBUG(node_->get_logger(), "Threshold: %f", threshold_);
  auto entity_counter = 0;
  for (auto it = detections.begin(); it != detections.end() && entity_counter < max_entities_; ) {
    auto const & detection = *it;
    if (detection.score <= threshold_ || detection.center3d.position.z > max_depth_) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Removing detection %s", detection.class_name.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "Score: %f", detection.score);
      RCLCPP_DEBUG(node_->get_logger(), "Depth: %f", detection.center3d.position.z);
      it = detections.erase(it);

    } else {
      std::string tf_name;
      tf_name = detection.class_name + "_" + std::to_string(entity_counter + 1);
      frames_.push_back(tf_name);
      if (pl::getInstance(node_)->publishTF(detection, tf_name) == -1) {
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "Publishing TF %s", tf_name.c_str());
      ++it;
      ++entity_counter;
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "Detections sorted and filtered");
  if (frames_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections after filtering");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(node_->get_logger(), "A %s has been found with the provided conditions", detections[0].class_name.c_str());
  // setOutput("best_detection", detections[0].class_name);
  setOutput("best_detection", detections[0]);
  setOutput("n_dectections", static_cast<int>(detections.size()) + 1);
  // setOutput("frames", frames_);
  setOutput("frame", frames_[0]); // TF frame of the best detected entity

  // frames_.clear();

  RCLCPP_DEBUG(node_->get_logger(), "Detections published");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsDetected>("IsDetected");
}
