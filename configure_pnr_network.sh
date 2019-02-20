#!/usr/bin/env bash

# Configure the servier-side of PNR ROS.

# not sure if we'll be using wlan0 or enxb827ebbc6516 yet.
export ROS_IP=$(ifconfig wlan0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')
export ROS_MASTER_URI=http://$ROS_IP:11311

