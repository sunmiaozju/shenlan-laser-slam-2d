#!/bin/bash

ip=$(ip addr show eth0 | grep -o 'inet [0-9]\+\.[0-9]\+\.[0-9]\+\.[0-9]\+' | grep -o [0-9].*)

# export ROS_HOSTNAME=192.168.199.158    #极路由ip

export ROS_MASTER_URI="http://$ip:11311" #网卡ip

export ROS_HOSTNAME=$ip
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic install
