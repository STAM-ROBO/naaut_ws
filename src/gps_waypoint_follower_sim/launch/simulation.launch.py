# Copyright (c) 2018 Intel Corporation
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
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    gps_wpf_dir = get_package_share_directory("gps_waypoint_follower_sim")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "naaut.world")

    urdf = os.path.join(gps_wpf_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    models_dir = os.path.join(gps_wpf_dir, "models") + os.pathsep + f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    set_gazebo_model_path_cmd = None

    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")
    set_gazebo_model_path_cmd = SetEnvironmentVariable("GAZEBO_MODEL_PATH", models_dir)

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess( cmd=['gzclient'], cwd=[launch_dir], output='both')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)
    ld.add_action(start_gazebo_server_cmd)
    #ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
