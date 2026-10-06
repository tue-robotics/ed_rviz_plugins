# ED RVIZ Plugins

[![CI](https://github.com/tue-robotics/ed_rviz_plugins/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/ed_rviz_plugins/actions/workflows/main.yml)

Visualizes the `ed_gui_server` output in RVIZ.

## Installation

Depends on:

- <https://github.com/tue-robotics/ed.git> (ROS 2 branch)
- <https://github.com/tue-robotics/ed_gui_server/tree/ros2> (ROS 2 branch; provides `ed_gui_server_interfaces`)

Check out the ROS 2 packages in your workspace:

    cd <your_ros2_workspace>/src
    git clone -b ros2 https://github.com/tue-robotics/ed_gui_server.git
    git clone -b ros2 https://github.com/tue-robotics/ed_rviz_plugins.git

Build with colcon:

    cd <your_ros2_workspace>
    colcon build --packages-up-to ed_rviz_plugins

## Tutorial

All ED tutorials can be found in the ed_tutorials package: <https://github.com/tue-robotics/ed_tutorials>
