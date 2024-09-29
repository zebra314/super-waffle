#!/bin/zsh
source /opt/ros/noetic/setup.zsh
cd /root/tiago_ws && catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /root/tiago_ws/devel/setup.zsh
exec "$@" 