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


#ifndef IDENTIFY_HPP_
#define IDENTIFY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection.hpp"

namespace perception
{

using namespace std::chrono_literals;
using std::placeholders::_1;

class Identify : public BT::ActionNodeBase
{
public:
  Identify(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("entity_to_identify"), // Name of the entity to identify. Will be used as key in the blackboard
                                                          // A TF frame with the name of the entity will be published
        BT::InputPort<double>("confidence"), // Confidence threshold
        BT::InputPort<perception_system_interfaces::msg::Detection>("detection"), // Detection of the entity to identify.
                                                                                  // If present, it is stored in the blackboard for later use
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string entity_;
  bool detection_at_input_;
  float confidence_;

  perception_system_interfaces::msg::Detection detection_;
  
};

} // namespace perception

#endif  // IDENTIFY_HPP_