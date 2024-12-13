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

#ifndef GET_FROM_BB__HPP
#define GET_FROM_BB__HPP

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "perception_system_interfaces/msg/detection.hpp"


namespace perception
{

class GetDetectionFromBB : public BT::ActionNodeBase
{
public:
  explicit GetDetectionFromBB(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("key"),
        BT::OutputPort<std::shared_ptr<perception_system_interfaces::msg::Detection>>("detection")
      });
  }

private:

  std::string key_;

};

}  // namespace perception

#endif  // GET_FROM_BB__HPP
