#!/bin/bash

tmux \
  new-session  "roslaunch iarc_sim_3d sim.launch single:=false map_tf:=true" \; \
  split-window "roslaunch iarc_fuses hud.launch" \; \
  split-window "ROS_NAMESPACE=/drone_1 rosrun teleop_twist_keyboard teleop_twist_keyboard.py" \; \
  split-window -h "ROS_NAMESPACE=/drone_2 rosrun teleop_twist_keyboard teleop_twist_keyboard.py" \; \
  split-window "rostopic pub /drone_1/ardrone/takeoff std_msgs/Empty "{}" --latch" \; \
  split-window "rostopic pub /drone_2/ardrone/takeoff std_msgs/Empty "{}" --latch" \;
