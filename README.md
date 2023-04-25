[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-8d59dc4de5201274e310e4c54b9627a8934c3b88527886e3b421487c677d23eb.svg)](https://classroom.github.com/a/j9y_86cr)

[![main](https://github.com/Docencia-fmrico/patrolling-ros2d2/actions/workflows/main.yaml/badge.svg?branch=main)](https://github.com/Docencia-fmrico/patrolling-ros2d2/actions/workflows/main.yaml)

# Plansys2 GPSR ROS2D2

Ejercicio 4 de Planificación y Sistemas Cognitivos 2023

## Content table
1. [Introduction](#Introduction)
2. [About](#About)
3. [Usage](#Usage)

## Introduction

The main objective of this project is to equip a TurtleBot3 Burger with planification capabilities, enabling it to carry out various domestic activities. This is achieved by using the PlanSys2 stack and using Behavior Trees to represent each of the robot's actions.

The specific problem at hand involves assisting a person with various tasks within their home, such as placing objects in designated locations, opening/closing doors, and retrieving objects for the person.

Given the nature of the problem, tasks can be classified into two distinct priority levels:

Human tasks - these tasks are specifically requested by the person and take priority over all other tasks. Examples include retrieving objects for the person or opening/closing doors as requested.

Other tasks - these tasks are secondary in priority and include activities such as moving objects, arranging objects in specific locations, moving the robot to specific locations, and opening/closing doors when not specifically requested by the person.

In order to successfully execute the tasks, the robot will need to generate valid plans by selecting appropriate actions from the available Behavior Trees. This will require usage of PlanSys2.

## About

### Planing

The code implements a node for controlling the execution of a planning problem using the Plansys2 framework. The node initializes a set of domain and problem experts, planner and executor clients, and defines a set of initial state and goal predicates.

The planning problem involves navigating a robot through a set of rooms and doors to deliver an object to a person in a specific room, while satisfying a set of constraints and goals. The node uses the Plansys2 framework to generate a plan for achieving the goal state and then executes the plan by calling the executor client. The node runs continuously and monitors the status of the plan execution until completion, at which point it outputs a success message or an error message if the execution failed.

The domain includes these durative actions:

**move_through_door**: moves a robot from one location to another through a door.

**move**: moves a robot from one location to another.

**open_door**: opens a door.

**requested_door_opened**: opens a door that a human has requested to be opened.

**close_door**: closes a door.

**requested_door_closed**: closes a door that a human has requested to be closed.

**pick**: picks up an object.

**drop**: drops an object.

**give_object**: gives an object to a person.

**arrange_object**: arranges an object in a room.


Each node has a counter programmed into its tick function that causes the node to continue returning the status RUNNING until it has been ticked 5 times. This simulates the duration of the action.

**Move_through_door**

The MoveThroughDoor class inherits from plansys2::BtActionNode, which is a base class for Behavior Tree nodes that represent actions that can be executed by the robot. The constructor of the MoveThroughDoor class initializes some variables related to the waypoints that the robot needs to follow, as well as the door that needs to be opened to reach the final goal. These waypoints refer to locations near each door that are specifically designated for opening the door. These locations are unique to each door and cannot be found anywhere else.

The on_tick() method returns RUNNING as long as the action is being executed. When the on_success() method is called, it returns SUCCESS, indicating that the action has been completed successfully.

**Move**

The class is responsible for moving the robot to a specific location using the ROS Navigation2 stack. The Move class inherits from the BtActionNode class and is registered as a behavior tree node. The constructor initializes the Move class and retrieves the parameters for the waypoints. The on_tick function handles moving the robot to the specified goal by setting the goal pose and returning a running status. The on_success function returns a success status when the robot reaches the goal pose. 

### World

After careful consideration of the limitations posed by Gazebo and the code from Pal Robotics, we made the decision to switch to Webots and adopt the TurtleBot3 Burger for our project. This allowed us to leverage our existing knowledge and experience with the simulator, which we used to create a custom world tailored to our needs. Our aim was to maximize the capabilities of the platform and optimize our workflow, while also ensuring that we could achieve our project goals in a timely and efficient manner. Through this approach, we were able to overcome the challenges we faced and deliver a high-quality solution that met the project requirements:

![alt text](https://github.com/Docencia-fmrico/plansys2-gpsr-ros2d2/blob/main/img/House_img.png)

In addition to creating our own world, we mapped it out to enable seamless navigation.

![alt text](https://github.com/Docencia-fmrico/plansys2-gpsr-ros2d2/blob/main/img/big_house_tags.png)

To perform the mapping process, we followed the conventional steps. However, instead of launching Gazebo, we utilized an alternate package to initiate the simulation:
https://github.com/OscarMrZ/tb3_webots_minimal

Following this steps:

```
export TURTLEBOT3_MODEL=burger

ros2 launch tb3_webots_minimal robot_launch.py mapper:=true rviz:=true

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

export TURTLEBOT3_MODEL=burger

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

And afterwards saving the map.

Before starting the mapping process, it was necessary to solve a known bug in the Cartographer package. The bug occurs when two laser messages are received at the same time, which causes issues with the update frequency of certain parameters. In order to fix this issue, the update frequency of 'odometry_sampling_ratio', 'imu_sampling_ratio', and 'rangefinder_sampling_ratio' parameters must be set to 0.5. 

  [https://groups.google.com/g/google-cartographer/c/b-NiyoRky2c?pli=1]
  
To implement the fix, the user should open the configuration file 'turtlebot3_lds_2d.lua' located at '/opt/ros/humble/share/turtlebot3_cartographer/config/' using a text editor with sudo privileges (e.g., 'sudo vim /opt/ros/humble/share/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua') and change the mentioned parameters to 0.5.

### Tests

To be able to analyze if our code is correct and runs as desired, we have created a series.

#### Run test

In order to check if a node does not fail, we create its behaviur tree, blackboard and run it a total of 5 times. If the NodeStatus is success at the end of the five runs, the test is passed.

The following nodes have been subjectd to this test: close_door, open_door, pick, drop, request_opened, request_closed, give_object and arrange_object.

```
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
```


## Usage

### Installation and launching

First clone the next git repository: 

https://github.com/PlanSys2/ros2_planning_system.git

Afterwards you should clone that allows to launch the webots simulator: 

https://github.com/OscarMrZ/tb3_webots_minimal.git

In order to oppen the simulation launch: 

```
ros2 launch tb3_webots_minimal robot_launch.py
```

Next launch from this repository:

```
ros2 launch plansys2_gpsr_ros2d2 robot_bringup_launch.py

ros2 launch plansys2_gpsr_ros2d2 plansys_bringup_launch.py
```

And finnaly run it:

```
ros2 run plansys2_gpsr_ros2d2 controller_node.cpp
```

## Working example

https://youtu.be/TuEe4allwSE

## Instructions
En grupos de 4, haced una aplicación en ROS 2 usando PlanSys2 que use el dominio de la [Práctica 3](https://github.com/Docencia-fmrico/planning-exercise/blob/main/README.md). El robot debe poder realizar en un simulador, navegando con Nav2, goals similares a los siguientes:

(ordena_casa robot1)
(abrir_puerta puerta_principal)
(dar robot1 vaso_leche abuelita)
(dar robot1 medicina abuelita)

Puntuación (sobre 10):   
* +5 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* +2 CI con Test de los nodos BT
* -3 Warnings o que no pase los tests.
