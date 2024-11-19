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


#include "perception/IdentifyPerson.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

IdentifyPerson::IdentifyPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  person_("")
  // get_features_(false),
  // tf_buffer_(),
  // tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  getInput("person_to_identify", person_);
  getInput("get_features", get_features_);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus
IdentifyPerson::tick()
{
  try
  {    
    // // Get the transform from map to the person
    // auto map2person_msg = tf_buffer_.lookupTransform("map", person_, tf2::TimePointZero);
    // tf2::Stamped<tf2::Transform> map2person;
    // tf2::fromMsg(map2person_msg, map2person);

    // // Publish transform
    // static_broadcaster_->sendTransform(tf2::toMsg(map2person));

    if (get_features_) // Configure perception system to track the specified person
    {
      // pl::getInstance(node_)->set_features_of_interest(person_);
    }
    // Get the detection of the person with the previously configured features
    perception_system_interfaces::msg::Detection detection;
    // detection = pl::getInstance(node_)->get_by_features(person_);
    // Publish the detection
    pl::getInstance(node_)->publicTF(detection, person_);
  
    return BT::NodeStatus::SUCCESS;
  }
  catch(const std::exception& ex)
  {
    RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  
  return BT::NodeStatus::RUNNING;
}


}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IdentifyPerson>("IdentifyPerson");
}

