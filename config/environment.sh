# This file is an example of the environment variables that can be set for StepIt.

export STEPIT_DEFAULT_CONTROLINPUT=""  # string, the default control input
export STEPIT_DEFAULT_POLICY=""        # string, the default policy
export STEPIT_DEFAULT_PUBLISHER=""     # string, the default data publisher
export STEPIT_DEFAULT_ROBOTAPI=""      # string, the default robot API
export STEPIT_DEFAULT_SPIN=""          # string, the default spin

export STEPIT_PLUGIN_DIRS="build/lib"  # string, the directory of the plugins
export STEPIT_EXTRA_PLUGIN_DIRS=""     # string, the extra directories of the plugins beyond the default ones
export STEPIT_COMM_CPUID=0             # int, the CPU ID of the communication thread to bind to
export STEPIT_AGENT_CPUID=1            # int, the CPU ID of the agent thread to bind to
export STEPIT_VERBOSITY=2              # int [0, 3], the verbosity level

export STEPIT_PUBLISH_STATUS=1         # bool, whether to publish the agent status
export STEPIT_PUBLISH_LOW_LEVEL=1      # bool, whether to publish the robot low-level state
export STEPIT_PUBLISH_ARRAY=1          # bool, whether to publish other array data

export STEPIT_DEFAULT_JOYSTICK=""      # string, the default joystick (plugin: joystick_base)
export STEPIT_DEFAULT_NNRTAPI=""       # string, the default neural network runtime (plugin: nnrt_base)

export STEPIT_JOYSTICK_ID=-1                # int, the joystick ID (/dev/js*) to read (plugin: joystick_native)
export STEPIT_HOST_IP="192.168.1.120"       # string, the robot host ip (plugin: robot_deeprobotics_*)
export STEPIT_NETIF="eth0"                  # string, the network interface for communication (plugin: robot_unitree2)
export STEPIT_UNITREE2_DOMAIN_ID=0          # int, DDS domain id for unitree_sdk2 (plugin: robot_unitree2)
export STEPIT_JOY_NAME=""                   # string, the key mapping type of the joystick (plugin: stepit_ros, stepit_ros2)
export STEPIT_ROS_NODE_NAME="stepit_ros"    # string, the ROS1 node name (plugin: stepit_ros)
export STEPIT_ROS2_NODE_NAME="stepit_ros2"  # string, the ROS2 node name (plugin: stepit_ros2)

export STEPIT_ROS2_QOS_RELIABILITY="best_effort"  # string, the ROS2 QoS reliability policy (plugin: stepit_ros2)
export STEPIT_ROS2_QOS_DURABILITY="volatile"      # string, the ROS2 QoS durability policy (plugin: stepit_ros2)
export STEPIT_ROS2_QOS_HISTORY="keep_last"        # string, the ROS2 QoS history policy (plugin: stepit_ros2)
