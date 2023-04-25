// Copyright 2019 Intelligent Robotics Lab
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

#include "behavior_tree_nodes/RequestedDoorClosed.hpp"

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace plansys2_gpsr_ros2d2
{

RequestedDoorClosed::RequestedDoorClosed(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void RequestedDoorClosed::halt() {std::cout << "RequestedDoorClosed halt" << std::endl;}

BT::NodeStatus RequestedDoorClosed::tick()
{
  std::string door;
  getInput<std::string>("door", door);
  std::cout << "Closed requested door: " << door << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2_gpsr_ros2d2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_gpsr_ros2d2::RequestedDoorClosed>("RequestedDoorClosed");
}
