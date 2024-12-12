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

#include "motion/navigation/navigate_to.hpp"
#include <chrono>

namespace navigation
{

NavigateTo::NavigateTo(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<nav2_msgs::action::NavigateToPose, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
  // tf_buffer_(),
  // tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void NavigateTo::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "NavigateTo ticked");
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::TransformStamped map_to_goal;
  std::string tf_frame, xml_path;

  getInput("tf_frame", tf_frame);
  getInput("will_finish", will_finish_);
  
  double distance_tolerance = 0;
  getInput("distance_tolerance", distance_tolerance);

  if (tf_frame.length() > 0) { // There is a TF to go, ignore coordinates
    RCLCPP_INFO(node_->get_logger(), "Transforming map to %s", tf_frame.c_str());
    try {
      map_to_goal = tf_buffer_->lookupTransform("map", tf_frame, tf2::TimePointZero);
      // rclcpp::Time current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      // rclcpp::Time time_threshold = current_time - rclcpp::Duration::from_seconds(5.0);
      // geometry_msgs::msg::TransformStamped map_to_goal =
      //   tf_buffer_->lookupTransform("map", tf_frame, tf2::TimePoint(std::chrono::seconds(time_threshold.nanoseconds() / 1000000000)));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        node_->get_logger(), "Could not transform map to %s: %s", tf_frame.c_str(), ex.what());
      setStatus(BT::NodeStatus::FAILURE);
      return;
    }
    
      goal.pose.position.x = map_to_goal.transform.translation.x;
      goal.pose.position.y = map_to_goal.transform.translation.y;
      goal.pose.position.z = 0.0;
      goal.pose.orientation.x = map_to_goal.transform.rotation.x;
      goal.pose.orientation.y = map_to_goal.transform.rotation.y;
      goal.pose.orientation.z = map_to_goal.transform.rotation.z;
      goal.pose.orientation.w = map_to_goal.transform.rotation.w;

      goal.header.frame_id = tf_frame;
  } else { // No TF, use coordinates

    getInput("x", goal.pose.position.x);
    getInput("y", goal.pose.position.y);
    RCLCPP_INFO(node_->get_logger(), "Setting goal to x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
    
    goal.pose.orientation.w = 1.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
  }

  goal.header.frame_id = "map";

  RCLCPP_INFO(
    node_->get_logger(), "Sending coordinates. Frame: %s, x: %f, y: %f", goal.header.frame_id.c_str(),goal.pose.position.x, goal.pose.position.y);
  
  if (distance_tolerance != 0) {
    RCLCPP_INFO(node_->get_logger(), "Setting distance tolerance to %f", distance_tolerance);
    xml_path = generate_xml_file(nav_to_pose_truncated_xml, distance_tolerance);
    goal_.behavior_tree = xml_path;
  }

  goal_.pose = goal;
}

BT::NodeStatus NavigateTo::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
  if (will_finish_) {
    return BT::NodeStatus::SUCCESS;
  }
  goal_updated_ = true;
  on_tick();
  on_new_goal_received();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::on_aborted()
{
  if (will_finish_) {
    return BT::NodeStatus::FAILURE;
  }
  on_tick();
  on_new_goal_received();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateTo::on_cancelled()
{
  if (will_finish_) {
    return BT::NodeStatus::SUCCESS;
  }
  on_tick();
  on_new_goal_received();
  RCLCPP_INFO(node_->get_logger(), "Navigation cancelled");
  return BT::NodeStatus::RUNNING;
}

}  // namespace navigation

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<navigation::NavigateTo>(name, "navigate_to_pose", config);
  };

  factory.registerBuilder<navigation::NavigateTo>("NavigateTo", builder);
}
