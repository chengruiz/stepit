#!/usr/bin/env bash

STEPIT_ARGS="${STEPIT_ARGS:-} -r aliengo -c joystick_usb"

export STEPIT_BLACKLIST_PLUGINS="${STEPIT_BLACKLIST_PLUGINS:-}:robot_unitree_go1:robot_unitree_b1"
