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

#ifndef PERCEPTION__ISDETECTED_HPP_
#define PERCEPTION__ISDETECTED_HPP_

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"
#include "perception_system_interfaces/msg/detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

class IsDetected : public BT::ConditionNode
{
public:
  explicit IsDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<int>("max_entities"), // Max number of entities to consider
        BT::InputPort<std::string>("model"), // YOLO model (object or people)
        BT::InputPort<std::string>("interest"), // What to look for
        BT::InputPort<float>("confidence"), // Confidence threshold
        BT::InputPort<std::string>("order"), // How to sort the detections
        BT::InputPort<double>("max_depth"), // Max depth to consider
        BT::OutputPort<std::shared_ptr<perception_system_interfaces::msg::Detection>>("best_detection"), // Best detected entity (first in the list)
        BT::OutputPort<int>("n_detections"), // Number of detections found
        BT::OutputPort<std::string>("frame") // Frame of the best detected entity (suffix: _1)
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string interest_, order_, cam_frame_;
  double threshold_, max_depth_;
  int max_entities_;
  std::int64_t person_id_;
  std::vector<std::string> frames_;
};

}  // namespace perception

#endif  // PERCEPTION__ISDETECTED_HPP_
