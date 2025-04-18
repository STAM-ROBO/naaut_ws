NAAUT Project

Installation


1) Install ROS2 humble desktop + ROS2 dev tools
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

2) Clone this repo. This repo is a workspace.

3) Install prerequistes 
Please, add your prerequistes if your Packages needs them.
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-robot-localization
sudo apt install -y ros-humble-tf2-tools
sudo apt install -y ros-humble-tf-transformations
pip install pyserial pyproj
pip install fastapi
pip install uvicorn 
pip install pyyaml

4) Build the workspace
Use the --symlink-install option to be able to modify and run python code without rebuilding the workspace
cd naaut_ws
colcon build --symlik-install

If needed, build and install the CH34x USB-to-serial adapter
cd \naaut_ws\src\CH341SER_LINUX
sudo make install

also, add a udev rule to assign RW permissions to all tty devices to all users
sudoedit /etc/udev/rules.d/50-ttyusb.rules
add this line and save:
KERNEL=="ttyUSB[0-9]*",NAME="tts/USB%n",SYMLINK+="%k",GROUP="uucp",MODE="0666"
unplug/plug device

*******************************
create a python pkg:
cd \naaut_ws\src
ros2 pkg create --build-type ament_python --node-name my_node my_package

Packages description

naaut-base:
    - main launch files
    - vessel-specific config files
    - 3D models

um982_driver:
    - ROS2 driver for Unicorecomm UM982 dual antenna RTK receiver
