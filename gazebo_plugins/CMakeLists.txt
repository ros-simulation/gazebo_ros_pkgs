cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  nodelet 
  angles 
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  urdf 
  tf 
  dynamic_reconfigure 
  driver_base 
  rosgraph_msgs 
  trajectory_msgs 
  pcl_ros 
  image_transport 
  rosconsole
)

# include(FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   pkg_check_modules(GAZEBO gazebo)
# else()
#   message(FATAL_ERROR "FindPkgConfig")
# endif()

# include (FindPkgConfig)
# if (PKG_CONFIG_FOUND)
#   # pkg_check_modules(GAZEBO gazebo)
#   # set(ENV{PKG_CONFIG_PATH} "${PROJECT_SOURCE_DIR}/gazebo/lib/pkgconfig")
#   pkg_check_modules(GAZEBO_SERVER REQUIRED gazebo)
#   pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
#   pkg_check_modules(GAZEBO_TRANSPORT REQUIRED gazebo_transport)
#   pkg_check_modules(XML libxml-2.0)
# else()
#   message(FATAL_ERROR "pkg-config is required; please install it")
# endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO_SYSTEM REQUIRED gazebo)
  # pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
  # pkg_check_modules(GAZEBO_TRANSPORT REQUIRED gazebo_transport)
  pkg_check_modules(XML libxml-2.0)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

pkg_check_modules(OGRE OGRE)

execute_process(COMMAND
pkg-config --variable=plugindir OGRE
OUTPUT_VARIABLE OGRE_PLUGIN_PATH
OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS OGRE_PLUGIN_PATH=${OGRE_PLUGIN_PATH})


catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CameraSynchronizer.cfg cfg/GazeboRosCamera.cfg cfg/GazeboRosOpenniKinect.cfg cfg/Hokuyo.cfg)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_SYSTEM_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS})
#link_directories(${OGRE_LIBRARY_DIRS} ${GAZEBO_SYSTEM_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#link_directories(/usr/local/lib/gazebo-1.5/plugins)

set(cxx_flags)
foreach (item ${GAZEBO_SYSTEM_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()
message(status "\n\n${cxx_flags}\n\n")

set(ld_flags)
foreach (item ${GAZEBO_SYSTEM_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()
message(status "\n\n${ld_flags}\n\n")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  vision_reconfigure 
  gazebo_ros_camera_utils 
  gazebo_ros_camera 
  gazebo_ros_multicamera 
  gazebo_ros_depth_camera 
  gazebo_ros_openni_kinect 
  gazebo_ros_gpu_laser 
  gazebo_ros_laser 
  gazebo_ros_block_laser 
  gazebo_ros_p3d 
  gazebo_ros_imu 
  gazebo_ros_f3d 
  gazebo_ros_bumper 
  gazebo_ros_template 
  gazebo_ros_projector 
  gazebo_ros_force 
  gazebo_ros_joint_trajectory 
  gazebo_ros_joint_pose_trajectory 
  
  CATKIN_DEPENDS 
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy 
  nodelet 
  angles 
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  urdf 
  tf 
  dynamic_reconfigure 
  driver_base 
  rosgraph_msgs 
  trajectory_msgs 
  pcl_ros 
  image_transport 
  rosconsole
#  DEPENDS gazebo
  )


add_executable(hokuyo_node src/hokuyo_node.cpp)
add_dependencies(hokuyo_node ${PROJECT_NAME}_gencfg)
target_link_libraries(hokuyo_node
  ${catkin_LIBRARIES}
)


add_library(vision_reconfigure src/vision_reconfigure.cpp)
add_dependencies(vision_reconfigure gazebo_plugins_gencfg)
target_link_libraries(vision_reconfigure ${catkin_LIBRARIES})

add_executable(camera_synchronizer src/camera_synchronizer.cpp)
target_link_libraries(camera_synchronizer vision_reconfigure ${catkin_LIBRARIES})

add_definitions(-fPIC)

find_package(Boost REQUIRED COMPONENTS thread)

add_library(gazebo_ros_camera_utils src/gazebo_ros_camera_utils.cpp)
set_target_properties(gazebo_ros_camera_utils PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera_utils PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera_utils ${GAZEBO_SYSTEM_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(MultiCameraPlugin src/MultiCameraPlugin.cpp)
set_target_properties(MultiCameraPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(MultiCameraPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(MultiCameraPlugin ${GAZEBO_SYSTEM_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_camera src/gazebo_ros_camera.cpp)
set_target_properties(gazebo_ros_camera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils ${GAZEBO_SYSTEM_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_multicamera src/gazebo_ros_multicamera.cpp)
set_target_properties(gazebo_ros_multicamera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_multicamera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_multicamera gazebo_ros_camera_utils ${GAZEBO_SYSTEM_LIBRARIES} MultiCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_depth_camera src/gazebo_ros_depth_camera.cpp)
set_target_properties(gazebo_ros_depth_camera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_depth_camera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_depth_camera gazebo_ros_camera_utils ${GAZEBO_SYSTEM_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_openni_kinect src/gazebo_ros_openni_kinect.cpp)
set_target_properties(gazebo_ros_openni_kinect PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_openni_kinect PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_openni_kinect gazebo_ros_camera_utils ${GAZEBO_SYSTEM_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_gpu_laser src/gazebo_ros_gpu_laser.cpp)
set_target_properties(gazebo_ros_gpu_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_gpu_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_gpu_laser ${catkin_LIBRARIES} ${GAZEBO_SYSTEM_LIBRARIES} GpuRayPlugin)

add_library(gazebo_ros_laser src/gazebo_ros_laser.cpp)
set_target_properties(gazebo_ros_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_laser ${GAZEBO_SYSTEM_LIBRARIES} RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_block_laser src/gazebo_ros_block_laser.cpp)
set_target_properties(gazebo_ros_block_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_block_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_block_laser ${GAZEBO_SYSTEM_LIBRARIES} RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_p3d src/gazebo_ros_p3d.cpp)
set_target_properties(gazebo_ros_p3d PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_p3d PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_p3d ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_imu src/gazebo_ros_imu.cpp)
set_target_properties(gazebo_ros_imu PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_imu PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_imu ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_f3d src/gazebo_ros_f3d.cpp)
set_target_properties(gazebo_ros_f3d PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_f3d PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_f3d ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_bumper src/gazebo_ros_bumper.cpp)
add_dependencies(gazebo_ros_bumper gazebo_msgs_gencpp)
set_target_properties(gazebo_ros_bumper PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_bumper PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_bumper ${GAZEBO_SYSTEM_LIBRARIES} ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})


add_library(gazebo_ros_template src/gazebo_ros_template.cpp)
set_target_properties(gazebo_ros_template PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_template PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_template ${Boost_LIBRARIES})

add_library(gazebo_ros_projector src/gazebo_ros_projector.cpp)
set_target_properties(gazebo_ros_projector PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_projector PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_projector ${Boost_LIBRARIES})

add_library(gazebo_ros_force src/gazebo_ros_force.cpp)
set_target_properties(gazebo_ros_force PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_force PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_force ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_trajectory src/gazebo_ros_joint_trajectory.cpp)
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_ros_joint_trajectory gazebo_msgs_gencpp)
target_link_libraries(gazebo_ros_joint_trajectory ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_pose_trajectory src/gazebo_ros_joint_pose_trajectory.cpp)
set_target_properties(gazebo_ros_joint_pose_trajectory PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_pose_trajectory PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_ros_joint_pose_trajectory gazebo_msgs_gencpp)
target_link_libraries(gazebo_ros_joint_pose_trajectory ${Boost_LIBRARIES})

add_executable(pub_joint_trajectory_test test/pub_joint_trajectory_test.cpp)
set_target_properties(pub_joint_trajectory_test PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(pub_joint_trajectory_test PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(pub_joint_trajectory_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_SYSTEM_LIBRARIES})

# exec_program("glxinfo" OUTPUT_VARIABLE glx_out RETURN_VALUE glx_val)
# if(${glx_val} EQUAL 0)
#   if(${glx_out} MATCHES "direct rendering: Yes")
#     message(STATUS "run test here")
#     rosbuild_add_rostest_labeled(gazebo test/p3d_test/test_3_single_pendulums.launch)
#     rosbuild_add_rostest_labeled(gazebo test/p3d_test/test_single_pendulum.launch)
#     rosbuild_add_rostest_labeled(gazebo test/p3d_test/test_3_double_pendulums.launch)
#     rosbuild_add_rostest_labeled(gazebo test/p3d_test/test_double_pendulum.launch)
#     rosbuild_add_rostest_labeled(gazebo test/bumper_test/test_bumper.launch)
#   endif(${glx_out} MATCHES "direct rendering: Yes")
# else(${glx_val} EQUAL 0)
#   message(STATUS "no test here")
# endif(${glx_val} EQUAL 0)

install(TARGETS 
  hokuyo_node 
  vision_reconfigure 
  camera_synchronizer
  gazebo_ros_camera_utils
  gazebo_ros_camera
  gazebo_ros_multicamera
  MultiCameraPlugin
  gazebo_ros_depth_camera
  gazebo_ros_openni_kinect
  gazebo_ros_openni_kinect
  gazebo_ros_laser
  gazebo_ros_block_laser
  gazebo_ros_p3d
  gazebo_ros_imu
  gazebo_ros_f3d
  gazebo_ros_bumper
  gazebo_ros_template
  gazebo_ros_projector
  gazebo_ros_force
  gazebo_ros_joint_trajectory
  gazebo_ros_joint_pose_trajectory
  pub_joint_trajectory_test  
  gazebo_ros_gpu_laser
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(PROGRAMS scripts/set_wrench.py scripts/set_pose.py scripts/gazebo_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )
