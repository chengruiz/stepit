STEPIT_ARGS="-r b2 -c joystick -f joystick@unitree2"
STEPIT_PLUGIN_ARGS=

# Other environment variables
export LD_LIBRARY_PATH=$STEPIT_WS/src/stepit/extern/robot_sdk/unitree_sdk2/thirdparty/lib/$(uname -m):$LD_LIBRARY_PATH
