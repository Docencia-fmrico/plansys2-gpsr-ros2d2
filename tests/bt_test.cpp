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

#include <list>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

TEST(bt_action, close_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_close_door_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_close_door_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <CloseDoor    name="close_door" door="${arg1}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, pick_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_pick_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_pick_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Pick    name="pick" object="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, drop_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_drop_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_drop_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Drop    name="dro" object="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, open_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_open_door_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_open_door_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <OpenDoor    name="open_door" door="${arg1}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, request_open_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_requested_door_opened_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_requested_door_opened_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
           <RequestedDoorOpened    name="requested_door_opened" door="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, request_close_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_requested_door_closed_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_requested_door_closed_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
           <RequestedDoorClosed    name="requested_door_closed" door="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, give_object_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_give_object_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_give_object_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <GiveObject    name="give_object" object="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

TEST(bt_action, arrange_object_btn)
{
  auto node = rclcpp::Node::make_shared("ros2d2_arrange_object_bt_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("ros2d2_arrange_object_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <ArrangeObject    name="arrange_object" object="${arg0}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  BT::NodeStatus status;
  for (int i = 0; i < 5; i++) {
    while (!finish && rclcpp::ok()) {
      status = tree.rootNode()->executeTick();
      finish = status != BT::NodeStatus::RUNNING;
      rate.sleep();
    }
    ASSERT_TRUE(status == BT::NodeStatus::SUCCESS);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
