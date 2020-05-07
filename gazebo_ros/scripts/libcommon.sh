#!/bin/sh

# the function relocates all the ROS remappings in the command at the end of the
# string this allows some punky uses of rosrun, for more information see:
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/387
relocate_remappings()
{
  command_line=${1}

  for w in $command_line; do
    if $(echo "$w" | grep -q ':='); then
      ros_remaps="$ros_remaps $w"
    else
      gazebo_args="$gazebo_args $w"
    fi
  done

  echo "$gazebo_args$ros_remaps" | cut -c 1-
}

# In order to avoid a runtime dependency on gazebo_dev in Debian packages
# a fallback to the default location of /usr/share/gazebo is used.
# For more information see:
# https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323
find_gazebo()
{
    if pkg-config --exists gazebo
    then
        setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo
    elif [ -d "/usr/share/gazebo" ]
    then
        echo "using default path /usr/share/gazebo"
        setup_path=/usr/share/gazebo
    else
        echo "gazebo not found, neither via pkg-config nor in /usr/share/gazebo"
        exit -1
    fi
}
