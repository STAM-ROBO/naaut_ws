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
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    launch_dir = os.path.join(get_package_share_directory("gps_waypoint_follower_sim"), 'launch')
    
    params_dir = os.path.join(get_package_share_directory("naaut-base"), "params")
    
    nav2_params = os.path.join(params_dir, "nav2_no_map_params.yaml")
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation.launch.py')
        )
    )
    
    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'mapviz.launch.py'))
    )
    
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )
    
    interactive_wf_node = Node(
        package="gps_waypoint_follower_sim",
        executable="interactive_waypoint_follower",
        name='interactive_wf'
    )
    
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf_navsat.launch.py'))
    )
    
    gnss_rtk_receiver = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver_node',
        output='screen',
        parameters=[{
            'orientation_topic':'/imu',
            'imu_link':'imu_link',
            'fix_topic':'/gps/fix',
            'gnss_link':'gps_link',
                }])
    
    motor_interface_node=Node(
        package='dc_motor_driver',
        executable='dc_motor_driver',
        name='dc_motor_driver',
        output='screen',
        parameters=[{
                    'serial_port' : '/dev/serial/by-path/platform-3610000.usb-usb-0:2.4:1.0',
                    'baudrate' : 115200
                    }])
    
    map_odom_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_tf_pub",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    
    #Load URDF into memory
    urdf_file = os.path.join(get_package_share_directory("naaut-base"), 'urdf', "robot.urdf")   
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
        
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'rate': 20, }])
    
    naaut_simulator = Node(
        package="naaut_sim",
        executable="run_sim",
        name="naaut_sim",
    )

    return LaunchDescription([ 
        #interactive_wf_node,
        motor_interface_node,
        robot_state_publisher_node,
        map_odom_tf_pub,
        #robot_localization_cmd,
        navigation2_cmd,
        #gnss_rtk_receiver,
        naaut_simulator
    ])
