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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__QUERY_HPP_
#define HRI__QUERY_HPP_

#include <algorithm>
#include <string>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "llama_msgs/action/generate_response.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"

namespace dialog
{

class Query
: public hri::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Query(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("text"),
        BT::InputPort<std::string>("intention"),
        BT::OutputPort<std::string>("intention_value")
      });
  }

private:
  std::string load_text_file(std::string path);
  std::string swap_placeholders(std::string text, std::vector<std::string> elems);

  std::string prompt_file_path_;
  std::string grammar_file_path_;
  std::string placeholder_{"[]"};

  const int START_SPEECH_{1};
};

}  // namespace dialog

#endif  // HRI__QUERY_HPP_
