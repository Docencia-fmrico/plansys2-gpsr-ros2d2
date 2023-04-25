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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Order types
enum { CLOSE = 0, OPEN = 1, OBJECT = 2, EMPTY = 3 };

// Execution status
enum { SUCCESS = 0, RUNNING = 1, FAILURE = 2, NEWPLAN = 3, IDLE = 4 };

// GUI positions
enum
{
  A_BATHROOM = 0,
  A_KITCHEN = 1,
  A_BEDROOM1 = 2,
  A_BEDROOM2 = 3,
  OPEN_REQUEST = 4,
  CLOSE_REQUEST = 5,
  OBJECT_REQUEST = 6,
  START = 7,
};

// Order structs
typedef struct
{
  std::string object;
  std::string room;
} arrange_order;

typedef struct
{
  std::string door;
  std::string person;
  std::string object;
  int type;
} human_request;

// A mouse callback function to get the coordinates of the mouse click
void mouse_callback(int event, int x, int y, int flags, void * param)
{
  if (event == cv::EVENT_LBUTTONDOWN) {
    cv::Point * point = (cv::Point *)param;
    point->x = x;
    point->y = y;

    std::cout << "Mouse clicked at: " << x << ", " << y << std::endl;
  }
}

class Assemble : public rclcpp::Node
{
public:
  Assemble()
  : rclcpp::Node("controller_node") {}

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    init_basic_knowledge();

    index_open_ = 0;
    index_close_ = 0;
    index_object_ = 0;
    yoda_request_.type = EMPTY;

    // Setp the objects and the doors
    objects_ = {"towel", "cutlery", "milk", "medicine", "clothes", "photo"};
    doors_ = {"d1", "d2", "d3", "d4", "d5"};
    active_zones_ = {false, false, false, false, false, false, false};

    auto gui_path_ =
      ament_index_cpp::get_package_share_directory("plansys2_gpsr_ros2d2") + "/img/gui.png";

    gui_image_ = cv::imread(gui_path_, cv::IMREAD_COLOR);

    cv::namedWindow("R2D2 control panel", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("R2D2 control panel", mouse_callback, &mouse_pos_);

    status_ = IDLE;

    return true;
  }

  void generate_plan(std::vector<arrange_order> arrange_orders, human_request request)
  {
    // Setup the new goal
    setup_goal(arrange_orders, request);

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal "
                << parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      return;
    }

    if (!executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_ERROR(get_logger(), "Error starting a new plan (replan)");
    }
  }

  void setup_goal(std::vector<arrange_order> arrange_orders, human_request request)
  {
    std::string goal_expression = "(and ";

    // Order initial status
    problem_expert_->addPredicate(plansys2::Predicate("(no_close_door_request r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(no_open_door_request r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(no_object_request r2d2)"));

    // Add all object goals
    for (auto order : arrange_orders) {
      goal_expression += "(arranged_object " + order.object + " " + order.room + ") ";
    }

    // Add the human request goal
    if (request.type == OPEN) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(human_request_opendoor " + request.door + ")"));
      problem_expert_->removePredicate(plansys2::Predicate("(no_open_door_request r2d2)"));
      goal_expression += "(no_open_door_request r2d2) ";
    } else if (request.type == CLOSE) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(human_request_closedoor " + request.door + ")"));
      problem_expert_->removePredicate(plansys2::Predicate("(no_close_door_request r2d2)"));
      goal_expression += "(no_close_door_request r2d2) ";
    } else if (request.type == OBJECT) {
      problem_expert_->addPredicate(
        plansys2::Predicate("(human_request_object granny " + request.object + ")"));
      problem_expert_->removePredicate(plansys2::Predicate("(no_object_request r2d2)"));
      goal_expression += "(no_object_request r2d2) ";
    }

    goal_expression += ")";
    problem_expert_->setGoal(plansys2::Goal(goal_expression));
  }

  void init_basic_knowledge()
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
    problem_expert_->addInstance(plansys2::Instance{"corridor", "room"});

    // Init the doors
    problem_expert_->addInstance(plansys2::Instance{"d1", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d2", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d3", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d4", "door"});
    problem_expert_->addInstance(plansys2::Instance{"d5", "door"});

    // Init the pre-doors (only places where the doors may be opened)
    problem_expert_->addInstance(plansys2::Instance{"d1_k", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d1_lr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d2_cr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d2_lr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d3_br", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d3_cr", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d4_bath", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d4_br", "predoor"});
    problem_expert_->addInstance(plansys2::Instance{"d5_house", "predoor"});

    // Init the objects
    problem_expert_->addInstance(plansys2::Instance{"towel", "object"});
    problem_expert_->addInstance(plansys2::Instance{"cutlery", "object"});
    problem_expert_->addInstance(plansys2::Instance{"milk", "object"});
    problem_expert_->addInstance(plansys2::Instance{"medicine", "object"});
    problem_expert_->addInstance(plansys2::Instance{"clothes", "object"});
    problem_expert_->addInstance(plansys2::Instance{"photo", "object"});

    // General knowledge
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at r2d2 living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_available r2d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(person_at granny bedroom)"));

    // Objects status
    problem_expert_->addPredicate(plansys2::Predicate("(object_at towel living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at cutlery corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at milk kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at medicine bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at clothes bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(object_at photo living_room)"));

    // Connections
    // kitchen <-> living_room
    problem_expert_->addPredicate(plansys2::Predicate("(connected kitchen d1_k)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d1_k kitchen)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected living_room d1_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d1_lr living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d1 d1_k d1_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d1 d1_lr d1_k)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d1_k d1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d1_lr d1)"));

    // living_room <-> corridor
    problem_expert_->addPredicate(plansys2::Predicate("(connected living_room d2_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d2_lr living_room)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor d2_cr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d2_cr corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d2 d2_lr d2_cr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d2 d2_cr d2_lr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d2_lr d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d2_cr d2)"));

    // corridor <-> bedroom
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor d3_cr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d3_cr corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom d3_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d3_br bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d3 d3_cr d3_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d3 d3_br d3_cr)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d3_cr d3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d3_br d3)"));

    // bedroom <-> bathroom
    problem_expert_->addPredicate(plansys2::Predicate("(connected bedroom d4_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d4_br bedroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected bathroom d4_bath)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d4_bath bathroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d4 d4_br d4_bath)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_connection d4 d4_bath d4_br)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d4_br d4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d4_bath d4)"));

    // corridor <-> house entrance door
    problem_expert_->addPredicate(plansys2::Predicate("(connected corridor d5_house)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected d5_house corridor)"));
    problem_expert_->addPredicate(plansys2::Predicate("(next_to_door d5_house d5)"));

    // doors status
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d1)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d2)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d3)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_closed d4)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_open d5)"));
  }

  int plan_step()
  {
    int status = RUNNING;

    if (executor_client_->execute_and_check_plan()) {  // Plan running
      auto result = executor_client_->getResult();
      auto feedback = executor_client_->getFeedBack();
      for (const auto & action_feedback : feedback.action_execution_status) {
        std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0
                  << "%]";
      }
      std::cout << std::endl;
    } else {
      auto result = executor_client_->getResult();

      if (result.value().success) {
        status = SUCCESS;
      } else {
        status = FAILURE;
      }
    }
    return status;
  }

  void execution_control()
  {
    // Generate a goal if the system is IDLE
    if (status_ == NEWPLAN) {
      // Print a list of the arrange actions
      for (int i = 0; i < arrange_orders_.size(); i++) {
        std::string text = arrange_orders_[i].object + " -> " + arrange_orders_[i].room;
        std::cout << text << std::endl;
      }

      // Generate a plan
      generate_plan(arrange_orders_, yoda_request_);
      status_ = RUNNING;

    } else if (status_ == RUNNING) {
      status_ = plan_step();
    } else if (status_ == SUCCESS || status_ == FAILURE) {
      status_ = IDLE;
    }
  }

  bool in_bounds(int x, int y, int x1, int x2, int y1, int y2)
  {
    return x > x1 && x < x2 && y > y1 && y < y2;
  }

  int clicked_zone_checker()
  {
    int zone = -1;

    if (in_bounds(mouse_pos_.x, mouse_pos_.y, 139, 220, 180, 256)) {
      zone = START;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 347, 701, 233, 274)) {
      zone = A_BATHROOM;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 347, 701, 320, 365)) {
      zone = A_KITCHEN;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 347, 701, 410, 455)) {
      zone = A_BEDROOM1;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 347, 701, 500, 545)) {
      zone = A_BEDROOM2;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 984, 1020, 333, 350)) {
      zone = OPEN_REQUEST;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 985, 1020, 370, 390)) {
      zone = CLOSE_REQUEST;
    } else if (in_bounds(mouse_pos_.x, mouse_pos_.y, 960, 1042, 406, 426)) {
      zone = OBJECT_REQUEST;
    }

    mouse_pos_.x = -1;
    mouse_pos_.y = -1;

    return zone;
  }

  void update_arrange_actions(int zone)
  {
    // Only update the arrange actions if the zone is valid
    if (zone != -1 && zone < 5) {
      active_zones_[zone] = !active_zones_[zone];
    }

    // Check for the active zones and draw a rectangle in the text
    for (int i = 0; i < active_zones_.size(); i++) {
      if (i == A_BATHROOM) {
        if (active_zones_[i]) {
          // Draw a rectangle in the bathroom text
          cv::rectangle(
            edited_gui_, cv::Point(347, 233), cv::Point(701, 274), cv::Scalar(0, 255, 0), 2);

          // Add the arrange order to the list
          arrange_orders_.push_back({"towel", "bathroom"});
        }
      } else if (i == A_KITCHEN) {
        if (active_zones_[i]) {
          // Draw a rectangle in the kitchen text
          cv::rectangle(
            edited_gui_, cv::Point(347, 320), cv::Point(701, 365), cv::Scalar(0, 255, 0), 2);

          // Add the arrange order to the list
          arrange_orders_.push_back({"cutlery", "kitchen"});
        }
      } else if (i == A_BEDROOM1) {
        if (active_zones_[i]) {
          // Draw a rectangle in the bedroom1 text
          cv::rectangle(
            edited_gui_, cv::Point(347, 410), cv::Point(701, 455), cv::Scalar(0, 255, 0), 2);

          // Add the arrange order to the list
          arrange_orders_.push_back({"clothes", "bedroom"});
        }
      } else if (i == A_BEDROOM2) {
        if (active_zones_[i]) {
          // Draw a rectangle in the bedroom2 text
          cv::rectangle(
            edited_gui_, cv::Point(347, 500), cv::Point(701, 545), cv::Scalar(0, 255, 0), 2);

          // Add the arrange order to the list
          arrange_orders_.push_back({"photo", "bedroom"});
        }
      }
    }
  }

  void update_yoda_request(int zone)
  {
    if (zone == OPEN_REQUEST) {
      active_zones_[zone] = true;
      active_zones_[CLOSE_REQUEST] = false;
      active_zones_[OBJECT_REQUEST] = false;

      // Update the clicked times
      if (index_open_ < 4) {
        index_open_++;
      } else {
        index_open_ = 0;
      }
    } else if (zone == CLOSE_REQUEST) {
      active_zones_[zone] = true;
      active_zones_[OPEN_REQUEST] = false;
      active_zones_[OBJECT_REQUEST] = false;

      // Update the clicked times
      if (index_close_ < 4) {
        index_close_++;
      } else {
        index_close_ = 0;
      }
    } else if (zone == OBJECT_REQUEST) {
      active_zones_[zone] = true;
      active_zones_[OPEN_REQUEST] = false;
      active_zones_[CLOSE_REQUEST] = false;

      // Update the clicked times
      if (index_object_ < 5) {
        index_object_++;
      } else {
        index_object_ = 0;
      }
    }

    for (int i = 4; i < active_zones_.size(); i++) {
      if (i == OPEN_REQUEST) {
        // Generate a text in the image representing the door
        cv::putText(
          edited_gui_, doors_[index_open_], cv::Point(992, 345), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(0, 0, 255), 2);

        if (active_zones_[i]) {
          yoda_request_.type = OPEN;
          yoda_request_.door = doors_[index_open_];
        }
      } else if (i == CLOSE_REQUEST) {
        // Generate a text in the image representing the door
        cv::putText(
          edited_gui_, doors_[index_close_], cv::Point(994, 385), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(255, 0, 0), 2);

        if (active_zones_[i]) {
          yoda_request_.type = CLOSE;
          yoda_request_.door = doors_[index_close_];
        }
      } else if (i == OBJECT_REQUEST) {
        // Generate a text in the image representing the door
        cv::putText(
          edited_gui_, objects_[index_object_], cv::Point(970, 420), cv::FONT_HERSHEY_SIMPLEX, 0.5,
          cv::Scalar(0, 255, 0), 2);

        if (active_zones_[i]) {
          yoda_request_.type = OBJECT;
          yoda_request_.object = objects_[index_object_];
        }
      }
    }
  }

  void gui_updater()
  {
    edited_gui_ = gui_image_.clone();

    // Check the mouse position
    int zone = clicked_zone_checker();

    // Update the list of arrange actions
    update_arrange_actions(zone);

    // Generate the request
    update_yoda_request(zone);

    // Check for start condition
    if (zone == START && status_ == IDLE) {
      std::cout << "Start condition" << std::endl;
      status_ = NEWPLAN;
    }

    cv::imshow("R2D2 control panel", edited_gui_);
    cv::waitKey(1);
  }

private:
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  int status_;
  cv::Mat gui_image_;
  cv::Mat edited_gui_;
  cv::Point mouse_pos_;
  std::vector<std::string> objects_;
  std::vector<std::string> doors_;
  std::vector<arrange_order> arrange_orders_;
  human_request yoda_request_;
  std::vector<bool> active_zones_;
  int index_open_;
  int index_close_;
  int index_object_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Assemble>();

  // Init the world knowledge
  node->init();

  rclcpp::Rate rate(5);

  while (rclcpp::ok()) {
    // Spin the node
    rclcpp::spin_some(node->get_node_base_interface());

    // Update the GUI
    node->gui_updater();

    // Spin the plan control
    node->execution_control();

    // Sleep as needed
    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
