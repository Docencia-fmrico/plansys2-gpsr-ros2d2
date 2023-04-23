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
// WITHOUT WARRANTIES OR CONdITIONS OF ANY KINd, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <plansys2_pddl_parser/Utils.h>

#include <memory>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Assemble : public rclcpp::Node
{
public:
  Assemble()
  : rclcpp::Node("controller_node")
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_knowledge();

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return false;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (first)");
    }

    return true;
  }

  void init_knowledge()
  {
    // Init the robot
    problem_expert_->addInstance(plansys2::Instance{"r2d2", "robot"});

    // Init the people
    problem_expert_->addInstance(plansys2::Instance{"granny", "person"});

    // Init the rooms
    problem_expert_->addInstance(plansys2::Instance{"kitchen", "room"});
    problem_expert_->addInstance(plansys2::Instance{"living_room", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bathroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"bedroom", "room"});  

    // Init the doors
    problem_expert_->addInstance(plansys2::Instance{"d1", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d2", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d3", "door"});

    // Init the pre-doors
    problem_expert_->addInstance(plansys2::Instance{"d1_k", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d1_lr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d2_br", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d2_lr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d3_br", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d3_bath", "predoor"});

    // Init the objects
    problem_expert_->addInstance(plansys2::Instance{"towel", "object"});
    problem_expert_->addInstance(plansys2::Instance{"dish", "object"});
    problem_expert_->addInstance(plansys2::Instance{"glass", "object"});

    // General knowledge
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at granny kitchen)"));

    // Objects status
    problem_expert_->addPredicate(plansys2::Predicate("(object_at towel living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at dish bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at glass bedroom)"));

    // Connections 
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen d1_k)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d1_k kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected living_room d1_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d1_lr living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d1 d1_k d1_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d1 d1_lr d1_k)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d1_k d1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d1_lr d1)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom d2_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d2_br bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected living_room d2_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d2_lr living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d2 d2_br d2_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d2 d2_lr d2_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d2_br d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d2_lr d2)"));

    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom d3_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d3_br bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom d3_bath)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d3_bath bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d3 d3_br d3_bath)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d3 d3_bath d3_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d3_br d3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d3_bath d3)"));

    // doors status
    problem_expert_->addPredicate(plansys2::Predicate("(door_open d1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d3)"));

    // Human orders
    problem_expert_->addPredicate(plansys2::Predicate("(human_request_closedoor d1)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(human_request_opendoor d2)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(human_request_object granny towel)"));

    // Order control predicates (indicates which type of order is inactive)
    problem_expert_->addPredicate(plansys2::Predicate("(no_object_request r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(no_open_door_request r2d2)"));
    // problem_expert_->addPredicate(plansys2::Predicate("(no_close_door_request r2d2)"));

    // Goal: robot at bathroom
    problem_expert_->setGoal(plansys2::Goal("(and (no_close_door_request r2d2))"));
  }

  void step()
  {
    if (!executor_client_->execute_and_check_plan()) {  // Plan finished
      auto result = executor_client_->getResult();

      if (result.value().success) {
        RCLCPP_INFO(get_logger(), "Plan succesfully finished");
      } else {
        RCLCPP_ERROR(get_logger(), "Plan finished with error");
      }
    }
    else {
      auto feedback = executor_client_->getFeedBack();

      for (const auto & action_feedback : feedback.action_execution_status) {
        std::cout << "[" << action_feedback.action << " " <<
          action_feedback.completion * 100.0 << "%]";
      }
      std::cout << std::endl;
    }
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Assemble>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
