# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("gps_waypoint_follower_sim")
    rl_params_file = os.path.join(gps_wpf_dir, "config", "ekf_navsat_params.yaml")
    
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            
            #launch_ros.actions.Node(
            #    package="robot_localization",
            #    executable="ekf_node",
            #    name="ekf_filter_node_local",
            #    output="screen",
            #    parameters=[rl_params_file, {"use_sim_time": True}],
            #    remappings=[("odometry/filtered", "odometry/local")],
            #),
            
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_tf_pub",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
            ),
            
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_global",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                remappings=[
                    #expected_topic,   available topic
                    #inputs
                    ("imu/data", "imu"),
                    ("odometry/filtered", "odometry/global"),
                    ("gps/fix", "gps/fix"),
                    #outputs
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    
                ],
            ),
        ]
    )
