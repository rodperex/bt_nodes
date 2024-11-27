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
  confidence_(0.5)
{
  config().blackboard->get("node", node_);

  getInput("entity_to_identify", entity_);
  RCLCPP_INFO(node_->get_logger(), "Identifying %s", entity_.c_str());

  if (!getInput("confidence", confidence_)) {
    RCLCPP_WARN(node_->get_logger(), "No confidence provided. Using default value %f", confidence_);
  }
  
}

BT::NodeStatus
Identify::tick()
{
  std::vector<perception_system_interfaces::msg::Detection> detections;
  // perception_system_interfaces::msg::Detection entity_detection;

  // if (status() == BT::NodeStatus::IDLE) {
    detection_at_input_ = true;
    if (!getInput("detection", detection_)) {
      RCLCPP_DEBUG(node_->get_logger(), "No detection at input");
      detection_at_input_ = false;
    }
  // }

  try
  {    

    if (detection_at_input_) { // Configure perception system to track the specified person
      RCLCPP_INFO(node_->get_logger(), "Storing detection of %s", entity_.c_str());
      // detections = pl::getInstance(node_)->get_by_type("person");
      // std::sort(
      //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
      //   return a.center3d.position.z < b.center3d.position.z;
      // });
      // getInput("detection", entity_detection);
      // detections = pl::getInstance(node_)->set_features_of_interest(detection_);
      config().blackboard->set(entity_, detection_);
    }
    
    RCLCPP_INFO(node_->get_logger(), "Getting detection of %s", entity_.c_str());
    
    if (!detection_at_input_) {
      config().blackboard->get(entity_, detection_);
      RCLCPP_INFO(node_->get_logger(), "Detection of %s retrieved", entity_.c_str());
    }

    pl::getInstance(node_)->update(30);
    detections = pl::getInstance(node_)->get_by_features(detection_, confidence_);

    std::sort(
      detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
        return a.score > b.score;
    });

    if (detections.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "%s NOT detected", entity_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "%s detected with confidence %f. Publishing TF", entity_.c_str(), detections[0].score);

    // Publish the detection
    pl::getInstance(node_)->publishTF_EKF(detections[0], entity_, true);
    
    return BT::NodeStatus::SUCCESS;
  }
  catch(const std::exception& ex)
  {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::Identify>("Identify");
}

