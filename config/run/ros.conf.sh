#!/usr/bin/env bash

if [ -z "${ROS_VERSION:-}" ]; then
    die "ROS_VERSION is not set."
fi

if [ "${ROS_VERSION}" = "1" ]; then
    source $CONFIG_HOME/ros1.conf.sh
elif [ "${ROS_VERSION}" = "2" ]; then
    source $CONFIG_HOME/ros2.conf.sh
else
    die "Unsupported ROS version: $ROS_VERSION"
fi
