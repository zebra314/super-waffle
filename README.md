# super-waffle

The final project of DD2410, implement a mission planner for TIAGo to execute three different missions

## Installation

For Docker users

```bash
make
```

For native Ubuntu users

```bash
rosdep update
rosdep install --from-paths tiago_ws --ignore-src --rosdistro=$ROS_DISTRO -y
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Usage

```bash
roslaunch robotics_project gazebo_project.launch
roslaunch robotics_project launch_project.launch
```

## Files

- [bt_students.py](./tiago_ws/src/robotics_project/scripts/behaviour_trees/bt_students.py)

- [sm_students.py](./tiago_ws/src/robotics_project/scripts/state_machines/sm_students.py)

- [launch_project.launch](./tiago_ws/src/robotics_project/launch/launch_project.launch)