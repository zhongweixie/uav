#!/bin/bash


gnome-terminal --window --title="仿真和位置pid" -e 'bash -c "roslaunch control_demo turtle.launch; exec bash"' \
--tab --title="/cmd_vel乌龟速度" -e 'bash -c "sleep 5; rostopic echo /turtle1/cmd_vel; exec bash"' \
--tab --title="/pose乌龟位置" -e 'bash -c "sleep 5; rostopic echo /turtle1/pose; exec bash"' 
gnome-terminal --window --title="/odom乌龟里程计" -e 'bash -c "sleep 15; rostopic echo /turtle1/odom; exec bash"' \
--tab --title="节点计算里程计" -e 'bash -c "sleep 10; rosrun control_demo cal_odom; exec bash"' 
