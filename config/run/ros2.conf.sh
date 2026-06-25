#!/usr/bin/env bash

STEPIT_ARGS="${STEPIT_ARGS:-} -P ros2"
STEPIT_PLUGIN_ARGS="${STEPIT_PLUGIN_ARGS:-} --ros-args --remap __ns:=${NS:-/stepit} ${STEPIT_ROS2_ARGS:-}"
