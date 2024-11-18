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

#include "motion/navigation/NavigateThrough.hpp"


namespace navigation
{

NavigateThrough::NavigateThrough(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
    nav2_msgs::action::NavigateThroughPoses, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
}

void
NavigateThrough::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "NavigateThrough ticked");
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::TransformStamped map_to_goal;
  std::vector<geometry_msgs::msg::PoseStamped> poses_to_goal;
  int steps;
  std::string tf_frame, xml_path, trajectory_style;

  getInput("tf_frame", tf_frame);
  getInput("n_poses", steps);
  getInput("style", trajectory_style);

  
  if (tf_frame.length() > 0) { // There is a TF to go, ignore coordinates
    RCLCPP_INFO(node_->get_logger(), "Transforming %s to %s", "map", tf_frame.c_str());
    try {
      map_to_goal = tf_buffer_.lookupTransform("map", tf_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        node_->get_logger(), "Could not transform %s to %s: %s", "map", tf_frame.c_str(), ex.what());
      setStatus(BT::NodeStatus::RUNNING);
    }
    
    goal.pose.position.x = map_to_goal.transform.translation.x;
    goal.pose.position.y = map_to_goal.transform.translation.y;
    goal.pose.orientation.x = map_to_goal.transform.rotation.x;
    goal.pose.orientation.y = map_to_goal.transform.rotation.y;
    goal.pose.orientation.z = map_to_goal.transform.rotation.z;
    goal.pose.orientation.w = map_to_goal.transform.rotation.w;

  } else {
    getInput("x", goal.pose.position.x);
    getInput("y", goal.pose.position.y);
    
    RCLCPP_INFO(node_->get_logger(), "Setting goal to x: %f, y: %f", goal.pose.position.x, goal.pose.position.y);
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
  }

  goal.header.frame_id = "map";

  poses_to_goal = generate_poses_to_goal(goal, map_to_goal, steps, trajectory_style);

  RCLCPP_INFO(
    node_->get_logger(), "Sending goal: x: %f, y: %f, qx: %f, qy: %f, qz: %f qw: %f. Frame: %s",
    goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.x, goal.pose.orientation.y,
    goal.pose.orientation.z, goal.pose.orientation.w, goal.header.frame_id.c_str());

  goal_.poses = poses_to_goal;
}

BT::NodeStatus
NavigateThrough::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus
NavigateThrough::on_aborted()
{
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus
NavigateThrough::on_cancelled()
{
  return BT::NodeStatus::SUCCESS;
}

std::vector<geometry_msgs::msg::PoseStamped>
NavigateThrough::generate_poses_to_goal(    
    const geometry_msgs::msg::PoseStamped & goal,
    geometry_msgs::msg::TransformStamped map_to_goal,
    int n_poses,
    const std::string & style)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;

  for (int i = 0; i < n_poses; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = goal.header.frame_id;
    // pose.header.stamp = node_->now();

    // Fraction of the path
    double fraction = static_cast<double>(i) / (n_poses - 1);

    if (style == "straight") {
      // Straight line between the initial and final poses
      pose.pose.position.x = fraction * map_to_goal.transform.translation.x;
      pose.pose.position.y = fraction * map_to_goal.transform.translation.y;

    } else if (style == "circular-l" || style == "circular-r") {
      // Radius of the arc
      double radius = std::sqrt(
          map_to_goal.transform.translation.x * map_to_goal.transform.translation.x +
          map_to_goal.transform.translation.y * map_to_goal.transform.translation.y) / 2.0;
      double angle = fraction * M_PI;  // Rotation of 180 degrees

      // Calculation of the center of the arc and application of the angle according to the direction
      double sign = (style == "circular-l") ? 1.0 : -1.0;
      pose.pose.position.x = radius * std::cos(angle) * sign;
      pose.pose.position.y = radius * std::sin(angle);
    }

    // Orientation remains stable
    pose.pose.orientation = goal.pose.orientation;

    poses.push_back(pose);
  }  
  return poses;
}


}  // namespace navigation

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<navigation::NavigateThrough>(name, "/navigate_through_poses", config);
  };

  factory.registerBuilder<navigation::NavigateThrough>("NavigateThrough", builder);
}
