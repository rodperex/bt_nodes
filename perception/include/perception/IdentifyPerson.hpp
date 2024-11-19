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


#ifndef IDENTIFY_PERSON_HPP_
#define IDENTIFY_PERSON_HPP_

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

class IdentifyPerson : public BT::ActionNodeBase
{
public:
  IdentifyPerson(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  void halt() override
  {}

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("person_to_identify"),
        BT::InputPort<bool>("get_features") // If true, features are saved. If false, features were already saved
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  std::string person_;
  bool get_features_;

  // tf2::BufferCore tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  
};

} // namespace perception

#endif  // IDENTIFY_PERSON_HPP_
