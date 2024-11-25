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

#include "perception/IsPointing.hpp"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsPointing::IsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_()
{
  config().blackboard->get("node", node_);

  getInput("camera_frame", camera_frame_);
  getInput("low_pointing_limit", low_pointing_limit_);
  getInput("high_pointing_limit", high_pointing_limit_);
  getInput("output_frame", output_frame_);

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

// // Distance between two transformations
// double tfs_distance(
//   const geometry_msgs::msg::TransformStamped & tf1,
//   const geometry_msgs::msg::TransformStamped & tf2)
// {
//   return sqrt(
//     pow(tf1.transform.translation.x - tf2.transform.translation.x, 2) +
//     pow(tf1.transform.translation.y - tf2.transform.translation.y, 2) +
//     pow(tf1.transform.translation.z - tf2.transform.translation.z, 2));
// }

// // Mean of two transformations
// geometry_msgs::msg::TransformStamped tfs_mean(
//   const geometry_msgs::msg::TransformStamped & tf1,
//   const geometry_msgs::msg::TransformStamped & tf2)
// {
//   geometry_msgs::msg::TransformStamped mean = tf1;

//   mean.transform.translation.x = (tf1.transform.translation.x + tf2.transform.translation.x) / 2;
//   mean.transform.translation.y = (tf1.transform.translation.y + tf2.transform.translation.y) / 2;
//   mean.transform.translation.z = (tf1.transform.translation.z + tf2.transform.translation.z) / 2;

//   return mean;
// }

int IsPointing::publish_detection_tf(
  const perception_system_interfaces::msg::Detection & detection)
{
  int pointing_direction = -1;  
  geometry_msgs::msg::TransformStamped map2camera_msg;

  RCLCPP_INFO(
    node_->get_logger(), "Detectection in frame_id %s",
    detection.header.frame_id.c_str());

  try {
    map2camera_msg = tf_buffer_->lookupTransform("map", camera_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s", "map",
      camera_frame_.c_str(), ex.what());
    return -1;
  }

  // 0 is right, 1 is down-right, 2 is down, 3 is down-left, 4 is left, 5 is up-left, 6 is up, 7 is up-right
  RCLCPP_INFO(
    node_->get_logger(), "Low pointing limit %d. High point limit %d", low_pointing_limit_,
    high_pointing_limit_);
  for (int i = low_pointing_limit_; i <= high_pointing_limit_; i++) {
    RCLCPP_INFO(
      node_->get_logger(), "Pointing direction %d",
      detection.pointing_direction);
    if (detection.pointing_direction == i) {
      pointing_direction = i;

      tf2::Transform camera2detection;
      camera2detection.setOrigin(
        tf2::Vector3(
          detection.center3d.position.x, detection.center3d.position.y,
          detection.center3d.position.z));
      camera2detection.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      tf2::Transform map2camera;
      tf2::fromMsg(map2camera_msg.transform, map2camera);

      tf2::Transform map2detection = map2camera * camera2detection;

      geometry_msgs::msg::TransformStamped map2detection_msg;
      map2detection_msg.header.frame_id = "map";
      map2detection_msg.child_frame_id = output_frame_;

      map2detection_msg.transform = tf2::toMsg(map2detection);
      map2detection_msg.transform.translation.z = 0.0;

      tf_static_broadcaster_->sendTransform(map2detection_msg);
      break;
    }
  }
  return pointing_direction;
}

BT::NodeStatus IsPointing::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "IsPointing ticked while IDLE");
  }

  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(true);
  rclcpp::spin_some(node_->get_node_base_interface());

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;

  // std::sort(
  //   detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
  //     return perception_system::diffIDs(this->person_id_, a.color_person) <
  //     perception_system::diffIDs(this->person_id_, b.color_person);
  //   });

  best_detection = detections[0];

  RCLCPP_INFO(
    node_->get_logger(), "Best detection: %s, color: %ld, pointing: %d",
    best_detection.unique_id.c_str(), best_detection.color_person,
    best_detection.pointing_direction);

  // int direction = publish_detection_tf(best_detection);
  int direction;
  pl::getInstance(node_)->publishTF(best_detection, output_frame_);
  for (int i = low_pointing_limit_; i <= high_pointing_limit_; i++) {
    RCLCPP_INFO(
      node_->get_logger(), "Pointing direction %d",
      best_detection.pointing_direction);
    if (best_detection.pointing_direction == i) {
      direction = i;
      setOutput("pointing_direction", direction);
      break;
    }
  }

  if (direction == -1) {
    RCLCPP_ERROR(node_->get_logger(), "Error, invalid pointing direction");
    return BT::NodeStatus::FAILURE;
  }

  // If someones is pointing, we return SUCCESS and populate output ports
  setOutput("detection", best_detection);
  setOutput("output_frame", output_frame_);
  setOutput("pointing_direction", direction);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsPointing>("IsPointing");
}
