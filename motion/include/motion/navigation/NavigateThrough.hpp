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

#ifndef NAVIGATION__NAVIGATE_THROUGH_HPP_
#define NAVIGATION__NAVIGATE_THROUGH_HPP_

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace navigation
{

class NavigateThrough : public motion::BtActionNode<
    nav2_msgs::action::NavigateThroughPoses, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit NavigateThrough(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("tf_frame"),
        BT::InputPort<std::string>("style"), // straight, circular-l, circular-r
        BT::InputPort<int>("n_poses")
      });
  }

private:
  std::vector<geometry_msgs::msg::PoseStamped> generate_poses_to_goal(    
    const geometry_msgs::msg::PoseStamped & goal,
    geometry_msgs::msg::TransformStamped map_to_goal,
    int n_poses, const std::string & style);
  
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  double distance_tolerance_;
  std::string tf_frame_, xml_path_;
  geometry_msgs::msg::PoseStamped pose_;
  
};

}  // namespace navigation

#endif  // NAVIGATION__NAVIGATE_THROUGH_HPP_
