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

#ifndef BEHAVIOR_TREE_NODES__MOVE_HPP_
#define BEHAVIOR_TREE_NODES__MOVE_HPP_

#include <map>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plansys2_bt_actions/BTActionNode.hpp"

namespace plansys2_gpsr_ros2d2
{

class Move : public plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  explicit Move(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("goal")};}

private:
  std::map<std::string, geometry_msgs::msg::Pose2D> waypoints_;
};

}  // namespace plansys2_gpsr_ros2d2

#endif  // BEHAVIOR_TREE_NODES__MOVE_HPP_
