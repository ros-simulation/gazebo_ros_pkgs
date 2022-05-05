# Depend on system install of Gazebo
set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
find_package(gazebo REQUIRED)
unset(FPHSA_NAME_MISMATCHED)

message(STATUS "Gazebo version: ${GAZEBO_VERSION}")

# Workaround for https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1372
# Real fix should go into Gazebo11 https://github.com/osrf/gazebo/issues/3008
# On Ubuntu Jammy Gazebo11 is used directly from Ubuntu, before removing this
# please be sure that the package is updated there.
pkg_check_modules(TBB tbb)
set(TBB_PKG_CONFIG "tbb")
if(NOT TBB_FOUND)
  message(STATUS "TBB not found, attempting to detect manually")
  set(TBB_PKG_CONFIG "")

  # Workaround for CMake bug https://gitlab.kitware.com/cmake/cmake/issues/17135
  unset(TBB_FOUND CACHE)

  find_package(TBB CONFIG)
  if(TBB_FOUND)
    set(TBB_LIBRARIES TBB::tbb)
  else()
    find_library(tbb_library tbb ENV LD_LIBRARY_PATH)
    if(tbb_library)
      set(TBB_FOUND true)
      set(TBB_LIBRARIES ${tbb_library})
    else()
      message(FATAL_ERROR "Missing: TBB - Threading Building Blocks")
    endif()
  endif()
endif()

# The following lines will tell catkin to add the Gazebo directories and libraries to the
# respective catkin_* cmake variables.
set(gazebo_dev_INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS})
set(gazebo_dev_LIBRARY_DIRS ${GAZEBO_LIBRARY_DIRS} ${TBB_LIBRARY_DIRS})
set(gazebo_dev_LIBRARIES ${GAZEBO_LIBRARIES} ${TBB_LIBRARIES})

# Append gazebo CXX_FLAGS to CMAKE_CXX_FLAGS (c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
