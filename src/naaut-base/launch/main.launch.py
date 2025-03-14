import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    main_pkg_name="naaut-base"
    
    robot_params_dir=os.path.join(get_package_share_directory(main_pkg_name),'params')
    rviz_config_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')
    
    #Load URDF into memory
    urdf_file = os.path.join(get_package_share_directory(main_pkg_name), 'urdf', "robot.urdf")   
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    launch_nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
            launch_arguments={'params_file': os.path.join(robot_params_dir, "nav2_params.yaml")}.items(),
            )
        
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'rate': 20, }])
    
    radio_teleop_receiver = Node(
         package='teleop_receiver',
         executable='receiver',
         name='radio_teleop_receiver',
         output='screen',
         parameters=[{
                    'serial_port' : '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_709a94ae5324ed11926c94e8f9a97352-if00-port0',
                    'baudrate' : 115200,
                    'velocity_topic' : '/cmd_vel',
                    }])

    motor_control_node=Node(
        package='minolo',
        executable='motor_controller',
        name='minolo_motor_controller',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(robot_params_dir, 'motor_controller_params.yaml')])
    
    motor_interface_node=Node(
        package='minolo',
        executable='motor_interface',
        name='minolo_motor_interface',
        output='screen',
        parameters=[
            os.path.join(robot_params_dir, 'motor_interface_params.yaml')])
    
    localization_node=Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_params_dir, 'state_estimation_params.yaml')],       
    )
    
    return LaunchDescription([
        motor_control_node,
        motor_interface_node,
        robot_state_publisher_node,
        radio_teleop_receiver,
        #launch_nav2,
   ])