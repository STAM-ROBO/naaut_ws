NAAUT Project

Installation


1) Install ROS2 humble desktop + ROS2 dev tools
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2) Clone this repo. This repo is a workspace.

3) Install prerequistes 
Please, add your prerequistes if your Packages needs them.
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-tf-transformations
pip install um982-driver
pip install pyserial

4) Build the workspace
Use the --symlink-install option to be able to modify and run python code without rebuilding the workspace
cd ws_naaut 
colcon build --symlik-install

*******************************

Packages description

naaut-base: 
    - main launch files
    - vessel-specific config files
    - 3D models

um982_driver:
    - ROS2 driver for Unicorecomm UM982 dual antenna RTK receiver
