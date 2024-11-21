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

#ifndef BASE_FACE__HPP_
#define BASE_FACE__HPP_

#include <string>
#include <fstream>
#include <iostream>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion/navigation/utils.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace base
{

class Face : public motion::BtActionNode<
    nav2_msgs::action::NavigateToPose, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Face(
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
        BT::InputPort<std::string>("base_frame"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_, tf_frame_;  
};

}  // namespace navigation

#endif  // BASE_FACE__HPP_
