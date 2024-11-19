# waypoint_editor_ros

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

The ROS package of waypoint editing tool

<p align="center">
  <img src="images/waypoint_editor.png" height="400px"/>
</p>

## Features
### Edit the waypoint
- add
  - Add a waypoint to the end of the list
  - The id is not needed
  - The goal pose is needed ("2D Nav Goal" in rviz)
- modify
  - Modify the waypoint
  - The id is needed
  - The goal pose is needed ("2D Nav Goal" in rviz)
- insert
  - Insert a waypoint
  - The id is needed
  - The goal pose is needed ("2D Nav Goal" in rviz)
- delete
  - Delete the waypoint
  - The id is needed
  - The goal pose is not needed
### Undo/Redo
- undo
  - Click on "←" button
  - Undo the editing the waypoint
- redo
  - Click on "→" button
  - Redo the editing the waypoint
### ID
- The ID of the waypoint is automatically assigned
- If you delete a waypoint, the ID will be reassigned


## Environment
- Ubuntu 20.04
- ROS Noetic

## Dependencies
- [waypoint_manager_ros](https://github.com/ToshikiNakamura0412/waypoint_manager_ros.git)

## Install and Build
```
# clone repository
cd ~/catkin_ws/src
git clone https://github.com/abc181525/waypoint_editor_ros.git

# build
cd ~/catkin_ws
catkin_make
```

## How to use
```
roslaunch waypoint_editor_ros waypoint_editor.launch
roslaunch waypoint_editor_ros waypoint_manager.launch

sudo apt-get install ros-noetic-topic-tools
rosrun topic_tools relay /waypoint_manager/global_goal /move_base_simple/goal

rostopic pub /finish_flag std_msgs/Bool "data: true"
```

## Nodes
### waypoint_editor
#### Service Topics
- ~\<name>/edit_waypoint
  - Edit the waypoint
- ~\<name>/redo_waypoint
  - Redo editing the waypoint
- ~\<name>/undo_waypoint
  - Undo editing the waypoint

#### Parameters
- ~\<name>/<b>waypoint_file</b> (str, default: `waypoints.yaml`):<br>
  The file path of waypoints

### edit_client
#### Subscribed Topics
- /move_base_simple/goal (`geometry_msgs/PoseStamped`)
 - The goal pose

