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

#include "motion/base/face.hpp"

namespace base
{

Face::Face(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<nav2_msgs::action::NavigateToPose, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  getInput("base_frame", base_frame_);
  getInput("tf_frame", tf_frame_);
}

void Face::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Face ticked");
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::TransformStamped robot_to_goal, map_to_robot;

  RCLCPP_INFO(node_->get_logger(), "Transforming %s to %s", base_frame_.c_str(), tf_frame_.c_str());
  try {
    robot_to_goal = tf_buffer_.lookupTransform(base_frame_, tf_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "Could not transform %s to %s: %s", base_frame_.c_str(), tf_frame_.c_str(), ex.what());
    setStatus(BT::NodeStatus::RUNNING);
  }

  // RCLCPP_INFO(node_->get_logger(), "Transforming map to %s", base_frame_.c_str());
  // try {
  //   map_to_robot = tf_buffer_.lookupTransform("map", base_frame_, tf2::TimePointZero);
  // } catch (const tf2::TransformException & ex) {
  //   RCLCPP_WARN(
  //     node_->get_logger(), "Could not transform map to %s: %s", base_frame_.c_str(), ex.what());
  //   setStatus(BT::NodeStatus::RUNNING);
  // }

  goal.header.stamp = node_->now();
  goal.header.frame_id = base_frame_;

  // Set position to the robot's current position
  // goal.pose.position.x = map_to_robot.transform.translation.x;
  // goal.pose.position.y = map_to_robot.transform.translation.y;
  // goal.pose.position.z = map_to_robot.transform.translation.z;
  goal.pose.position.x = 0;
  goal.pose.position.y = 0;
  goal.pose.position.z = 0;

  // Calculate the yaw angle to face the target frame
  double dx = robot_to_goal.transform.translation.x;
  double dy = robot_to_goal.transform.translation.y;
  double yaw = std::atan2(dy, dx); // Angle to face the target frame

  // Convert yaw to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw); // Roll and pitch are zero; yaw is computed

  goal.pose.orientation.x = q.x();
  goal.pose.orientation.y = q.y();
  goal.pose.orientation.z = q.z();
  goal.pose.orientation.w = q.w();

  RCLCPP_INFO(node_->get_logger(), "Robot will rotate to face %s (yaw=%.2f radians)", tf_frame_.c_str(), yaw);

  goal_.pose = goal;
}


}  // namespace base

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<base::Face>(name, "navigate_to_pose", config);
  };

  factory.registerBuilder<base::Face>("Face", builder);
}
