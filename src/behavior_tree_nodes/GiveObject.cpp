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

#include <string>
#include <iostream>

#include "behavior_tree_nodes/GiveObject.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace plansys2_gpsr_ros2d2
{

GiveObject::GiveObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
GiveObject::halt()
{
  std::cout << "GiveObject halt" << std::endl;
}

BT::NodeStatus
GiveObject::tick()
{
  std::string object;
  getInput<std::string>("object", object);
  std::cout << "GiveObject tick " << counter_ << std::endl;

  if (counter_++ < 5) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_gpsr_ros2d2

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_gpsr_ros2d2::GiveObject>("GiveObject");
}