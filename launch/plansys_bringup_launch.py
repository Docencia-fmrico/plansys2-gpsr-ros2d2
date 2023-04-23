# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    project_dir = get_package_share_directory('plansys2_gpsr_ros2d2')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': project_dir + '/pddl/granny_house.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions

    move_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'move',
            'bt_xml_file': project_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    move_through_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_through_door',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'move_through_door',
            'bt_xml_file': project_dir + '/behavior_trees_xml/move_through_door.xml'
          }
        ])
    
    open_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='open_door',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'open_door',
            'bt_xml_file': project_dir + '/behavior_trees_xml/open_door.xml'
          }
        ])
    
    close_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='close_door',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'close_door',
            'bt_xml_file': project_dir + '/behavior_trees_xml/close_door.xml'
          }
        ])

    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'pick',
            'bt_xml_file': project_dir + '/behavior_trees_xml/pick.xml'
          }
        ])
    
    drop_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='drop',
        namespace=namespace,
        output='screen',
        parameters=[
          project_dir + '/config/plansys_config.yaml',
          {
            'action_name': 'drop',
            'bt_xml_file': project_dir + '/behavior_trees_xml/drop.xml'
          }
        ])

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # Declare the BT plansys2 actions
    ld.add_action(move_cmd)
    ld.add_action(move_through_door_cmd)
    ld.add_action(open_door_cmd)
    ld.add_action(close_door_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(drop_cmd)

    return ld
