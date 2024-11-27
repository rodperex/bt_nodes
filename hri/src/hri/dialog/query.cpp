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

#include "hri/dialog/query.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

Query::Query(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: hri::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{

  node_->declare_parameter("prompt_file", prompt_file_path_);
  node_->declare_parameter("grammar_file", grammar_file_path_);
  node_->get_parameter("prompt_file", prompt_file_path_);
  node_->get_parameter("grammar_file", grammar_file_path_);

  if (!node_->has_parameter("placeholder")) {
    node_->declare_parameter("placeholder", placeholder_);
  }
  node_->get_parameter("placeholder", placeholder_);

  try {
    std::string pkg_dir = ament_index_cpp::get_package_share_directory("hri");
    prompt_file_path_ = pkg_dir + "/config/" + prompt_file_path_;
    grammar_file_path_ = pkg_dir + "/config/" + grammar_file_path_;
  } catch (const std::exception &e) { 
    RCLCPP_ERROR(node_->get_logger(), "Error getting package share directory: %s", e.what());
    return;
  }

}

std::string
Query::load_text_file(std::string path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  return buffer.str();
  
}

std::string
Query::swap_placeholders(std::string text, std::vector<std::string> elems)
{
  for (const auto& elem : elems) {
    size_t pos = text.find(placeholder_);
    if (pos != std::string::npos) {
      text.replace(pos, placeholder_.length(), elem);
      RCLCPP_DEBUG(node_->get_logger(), "Modified string: %s", text.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Placeholder %s not found in prompt", placeholder_.c_str());
      break;
    }
  }

  return text;
}

void Query::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Query ticked");

  std::string text, intention;
  getInput("text", text);
  getInput("intention", intention);

  std::string prompt = load_text_file(prompt_file_path_);
  if (prompt.length() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "Error loading the prompt");
    return;
  }  

  std::vector<std::string> params = {text, intention};
  
  prompt = swap_placeholders(prompt, params);
  if (prompt.length() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "Error loading the prompt");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Prompt: %s", prompt.c_str());

  goal_.prompt = prompt;
  goal_.reset = true;
  goal_.sampling_config.temp = 0.0;
  goal_.sampling_config.grammar = load_text_file(grammar_file_path_);

  if (goal_.sampling_config.grammar.length() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "Error loading the grammar");
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Grammar: %s", goal_.sampling_config.grammar.c_str());

}

BT::NodeStatus Query::on_success()
{

  RCLCPP_INFO(node_->get_logger(), "Response: %s", result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }

  json response = json::parse(result_.result->response.text);
  std::string value = response["intention"];

  if (value.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("intention_value", value);
  RCLCPP_INFO(node_->get_logger(), "Intention value: %s", value.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Query>(name, "/llama/generate_response", config);
    };

  factory.registerBuilder<dialog::Query>("Query", builder);
}
