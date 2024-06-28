if(NOT DEFINED IGNORE_GAZEBO_MSGS_WARNING)
  if(DEFINED ENV{ROS_DISTRO})
    set(ROS_DISTRO $ENV{ROS_DISTRO})
    if(NOT (ROS_DISTRO STREQUAL "iron" OR
            ROS_DISTRO STREQUAL "humble"))
      message(WARNING "This gazebo_msgs package hosts messages designed initially for Gazebo Classic "
        "which is not available on ROS 2 Jazzy which is unavailable on ROS 2 Jazzy. "
        "The new Gazebo uses the ros_gz bridge message vailable at:"
        "https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#bridge-communication-between-ros-and-gazebo\n"
        "To avoid this warning and use this Gazebo Classic messages anyway you can use flag -DIGNORE_GAZEBO_MSGS_WARNING")
    endif()
  endif()
endif()
