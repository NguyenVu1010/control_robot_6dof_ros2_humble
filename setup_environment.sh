#!/bin/bash
echo "Installing System Dependencies..."
sudo apt update
sudo apt install -y build-essential cmake git libserial-dev socat gnome-terminal lsof

echo "Installing ROS 2 Humble Packages..."
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers \
     ros-humble-hardware-interface ros-humble-xacro \
     ros-humble-joint-state-broadcaster ros-humble-visualization-msgs

echo "Installing Python Dependencies..."
pip3 install PyQt6 pyserial

echo "Adding user to dialout group for Serial access..."
sudo usermod -a -G dialout $USER

echo "DONE! Please log out and log back in for Group changes to take effect."