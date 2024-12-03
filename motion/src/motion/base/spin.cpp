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

#include "motion/base/spin.hpp"

namespace base
{

Spin::Spin(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  vel_pub_->on_activate();
}

BT::NodeStatus Spin::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "SPIN");

  getInput("angle", angle_);
  getInput("speed", speed_);
  getInput("forever", forever_);
  // getInput("direction", direction_);

  if (forever_) {
    RCLCPP_DEBUG(node_->get_logger(), "Spinning forever");
  } else {
    // RCLCPP_INFO(node_->get_logger(), "Spinning %.2f degrees at %.2f rad/s (dir: %d)", angle_, speed_, direction_);
    RCLCPP_DEBUG(node_->get_logger(), "Spinning %.2f degrees at %.2f rad/s", angle_, speed_);
  }

  if (status() == BT::NodeStatus::IDLE) {
    rotated_angle_ = 0;
    last_time_ = std::chrono::high_resolution_clock::now();
    std::time_t current_time = std::chrono::system_clock::to_time_t(last_time_);
    std::tm local_time = *std::localtime(&current_time);
    RCLCPP_DEBUG(node_->get_logger(), "Starting to spin at: %d:%d:%d", local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::Twist cmd_vel;
  // cmd_vel.angular.z = speed_ * direction_;
  cmd_vel.angular.z = speed_ * std::copysign(1, angle_);

  if (forever_) {
    RCLCPP_DEBUG(node_->get_logger(), "Spinning forever");
    if (angle_ != 0) {
      vel_pub_->publish(cmd_vel);
    }
    return BT::NodeStatus::RUNNING;
  } else if ((std::abs(rotated_angle_) < std::abs(angle_))) {
    // RCLCPP_DEBUG(node_->get_logger(), "Target: %.2f degrees (dir: %d). Spinned %.2f degrees: ", angle_, direction_, rotated_angle_);
    RCLCPP_INFO(node_->get_logger(), "Target: %.2f degrees. Spinned %.2f degrees: ", angle_, rotated_angle_);
    vel_pub_->publish(cmd_vel);
    auto curr_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time_);
    rotated_angle_ += (speed_ * elapsed.count() / 1000.0) * 180 / M_PI;
    last_time_ = curr_time;
    return BT::NodeStatus::RUNNING;
  }

  // std::time_t current_time = std::chrono::system_clock::to_time_t(last_time_);
  // std::tm local_time = *std::localtime(&current_time);
  // RCLCPP_INFO(node_->get_logger(), "Finished spinning %.2f degrees at %d:%d:%d", angle_, local_time.tm_hour, local_time.tm_min, local_time.tm_sec);
  RCLCPP_INFO(node_->get_logger(), "Finished spinning %.2f degrees", angle_);
  return BT::NodeStatus::SUCCESS;
}

void Spin::halt()
{
  RCLCPP_DEBUG(node_->get_logger(), "Spin halted");
}

}  // namespace base

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<base::Spin>("Spin");
}
