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
  config().blackboard->get("node", node_);

  getInput("entity_to_identify", entity_);
}
BT::NodeStatus IsInFront::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "IS_IN_FRONT");

  rclcpp::spin_some(node_->get_node_base_interface());
  
  geometry_msgs::msg::TransformStamped base2entity_msg;
  tf2::Stamped<tf2::Transform> base2entity;
  
  base2entity_msg = tf_buffer_.lookupTransform("base_link", entity_, tf2::TimePointZero);
  tf2::fromMsg(base2entity_msg, base2entity);
  
  // double yaw = tf2::getYaw(base2entity.getRotation());
  double yaw = std::atan2(base2entity.getOrigin().y(), base2entity.getOrigin().x());

  yaw = yaw * 180.0 / M_PI;

  RCLCPP_DEBUG(node_->get_logger(), "Detection at (%.2f, %.2f, %.2f) from base_footprint. YAW: %.2f degrees",
    base2entity.getOrigin().x(),
    base2entity.getOrigin().y(),
    base2entity.getOrigin().z(),
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
  RCLCPP_INFO(node_->get_logger(), "Detection is in front!");
  setOutput("direction", 0.0);
  return BT::NodeStatus::SUCCESS;
  

}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsInFront>("IsInFront");
}
