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

#ifndef PERCEPTION_IS_IN_FRONT_HPP_
#define PERCEPTION_IS_IN_FRONT_HPP_

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection.hpp"

namespace perception
{

class IsInFront : public BT::ConditionNode
{
public:
  explicit IsInFront(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("confidence"),
        BT::InputPort<std::string>("entity_to_identify"),
        BT::InputPort<perception_system_interfaces::msg::Detection>("target"),
        BT::InputPort<std::string>("what"),
        BT::OutputPort<int>("direction")
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string target_;
  double confidence_;

  std::string entity_;

};

}  // namespace perception

#endif  // PERCEPTION_IS_IN_FRONT_HPP_
