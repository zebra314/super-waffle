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
cd ./tiago_ws
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

For the PCs in the lab rooms, which have already been set up

```bash
# Add this line at the end of your .bashrc file and source it:
export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/
source ~/.bashrc

cd ./tiago_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## Usage

```bash
roslaunch robotics_project gazebo_project.launch
roslaunch robotics_project launch_project.launch
```

## Files

- For grade E, [sm_students.py](./tiago_ws/src/robotics_project/scripts/state_machines/sm_students.py)

- For grade C and A, [bt_students.py](./tiago_ws/src/robotics_project/scripts/behaviour_trees/bt_students.py)

- [launch_project.launch](./tiago_ws/src/robotics_project/launch/launch_project.launch)