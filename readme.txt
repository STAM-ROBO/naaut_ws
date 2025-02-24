NAAUT Project

Installation


1) Install ROS2 humble desktop + ROS2 dev tools
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2) Clone this repo. This repo is a workspace.

3) Install prerequistes 
Please, add your prerequistes if your Packages needs them.
sudo apt install -Y ros-humble-navigation2
sudo apt install -Y ros-humble-nav2-bringup
sudo apt install -Y ros-humble-robot-localization
sudo apt install -Y ros-humble-tf2-tools
sudo apt install -Y ros-humble-tf-transformations
pip install pyserial pyproj

4) Build the workspace
Use the --symlink-install option to be able to modify and run python code without rebuilding the workspace
cd naaut_ws
colcon build --symlik-install

*******************************

Packages description

naaut-base:
    - main launch files
    - vessel-specific config files
    - 3D models

um982_driver:
    - ROS2 driver for Unicorecomm UM982 dual antenna RTK receiver
