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

#include "perception/is_in_view.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

IsInView::IsInView(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  getInput("detection", detection_);
}
BT::NodeStatus IsInView::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "IS_IN_VIEW");
  rclcpp::spin_some(node_->get_node_base_interface());

  // auto base2entity_msg = tf_buffer_.lookupTransform("base_link", frame_, tf2::TimePointZero);
  
  pl::getInstance(node_)->update(30);
  auto detections = pl::getInstance(node_)->get_by_id(detection_->unique_id);
  RCLCPP_DEBUG(node_->get_logger(), "Detection obtained by ID");
  // auto detections = pl::getInstance(node_)->get_detection_at(base2entity_msg);
  // RCLCPP_DEBUG(node_->get_logger(), "Detection obtained from TF");

  if (detections.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Detection not in view");
    return BT::NodeStatus::FAILURE;
  } else if (detections[0].score < 0.5) {
    RCLCPP_INFO(node_->get_logger(), "Detection in view but score is too low");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS; 

}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsInView>("IsInView");
}
