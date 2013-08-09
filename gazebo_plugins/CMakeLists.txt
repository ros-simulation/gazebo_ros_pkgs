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
  pcl_conversions
  image_transport 
  rosconsole
  cv_bridge
  polled_camera
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

# Depend on system install of Gazebo and SDFormat
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CameraSynchronizer.cfg cfg/GazeboRosCamera.cfg cfg/GazeboRosOpenniKinect.cfg cfg/Hokuyo.cfg)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()


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
  gazebo_ros_prosilica 
  gazebo_ros_force 
  gazebo_ros_joint_trajectory 
  gazebo_ros_joint_pose_trajectory
  gazebo_ros_diff_drive
  gazebo_ros_skid_steer_drive
  gazebo_ros_video
  gazebo_ros_planar_move
  
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
  pcl_conversions
  image_transport 
  rosconsole
  DEPENDS 
    gazebo 
    SDF
  )

## Executables
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

add_executable(pub_joint_trajectory_test test/pub_joint_trajectory_test.cpp)
set_target_properties(pub_joint_trajectory_test PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(pub_joint_trajectory_test PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(pub_joint_trajectory_test ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pub_joint_trajectory_test ${catkin_EXPORTED_TARGETS}) # don't build until gazebo_msgs is done

add_definitions(-fPIC) # what is this for?

## Plugins
add_library(gazebo_ros_camera_utils src/gazebo_ros_camera_utils.cpp)
set_target_properties(gazebo_ros_camera_utils PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera_utils PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(MultiCameraPlugin src/MultiCameraPlugin.cpp)
set_target_properties(MultiCameraPlugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(MultiCameraPlugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(MultiCameraPlugin ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_camera src/gazebo_ros_camera.cpp)
set_target_properties(gazebo_ros_camera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_camera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_camera gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_multicamera src/gazebo_ros_multicamera.cpp)
set_target_properties(gazebo_ros_multicamera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_multicamera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_multicamera gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} MultiCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_depth_camera src/gazebo_ros_depth_camera.cpp)
set_target_properties(gazebo_ros_depth_camera PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_depth_camera PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_depth_camera gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_openni_kinect src/gazebo_ros_openni_kinect.cpp)
set_target_properties(gazebo_ros_openni_kinect PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_openni_kinect PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_openni_kinect gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} DepthCameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_gpu_laser src/gazebo_ros_gpu_laser.cpp)
set_target_properties(gazebo_ros_gpu_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_gpu_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_gpu_laser ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} GpuRayPlugin)

add_library(gazebo_ros_laser src/gazebo_ros_laser.cpp)
set_target_properties(gazebo_ros_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_laser ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_block_laser src/gazebo_ros_block_laser.cpp)
set_target_properties(gazebo_ros_block_laser PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_block_laser PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_block_laser ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} RayPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_p3d src/gazebo_ros_p3d.cpp)
set_target_properties(gazebo_ros_p3d PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_p3d PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_p3d ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_imu src/gazebo_ros_imu.cpp)
set_target_properties(gazebo_ros_imu PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_imu PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_imu ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_f3d src/gazebo_ros_f3d.cpp)
set_target_properties(gazebo_ros_f3d PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_f3d PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_f3d ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_bumper src/gazebo_ros_bumper.cpp)
add_dependencies(gazebo_ros_bumper gazebo_msgs_gencpp)
set_target_properties(gazebo_ros_bumper PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_bumper PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_bumper ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${Boost_LIBRARIES} ContactPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_projector src/gazebo_ros_projector.cpp)
set_target_properties(gazebo_ros_projector PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_projector PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_projector ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_library(gazebo_ros_prosilica src/gazebo_ros_prosilica.cpp)
set_target_properties(gazebo_ros_prosilica PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_prosilica PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_prosilica gazebo_ros_camera_utils ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} CameraPlugin ${catkin_LIBRARIES})

add_library(gazebo_ros_force src/gazebo_ros_force.cpp)
set_target_properties(gazebo_ros_force PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_force PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_force ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_trajectory src/gazebo_ros_joint_trajectory.cpp)
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_trajectory PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_ros_joint_trajectory gazebo_msgs_gencpp)
target_link_libraries(gazebo_ros_joint_trajectory ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_joint_pose_trajectory src/gazebo_ros_joint_pose_trajectory.cpp)
set_target_properties(gazebo_ros_joint_pose_trajectory PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_joint_pose_trajectory PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_ros_joint_pose_trajectory gazebo_msgs_gencpp)
target_link_libraries(gazebo_ros_joint_pose_trajectory ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_diff_drive src/gazebo_ros_diff_drive.cpp)
set_target_properties(gazebo_ros_diff_drive PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_diff_drive PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_diff_drive ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_skid_steer_drive src/gazebo_ros_skid_steer_drive.cpp)
set_target_properties(gazebo_ros_skid_steer_drive PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_skid_steer_drive PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_skid_steer_drive ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(gazebo_ros_video src/gazebo_ros_video.cpp)
set_target_properties(gazebo_ros_video PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_video PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_video ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_LIBRARIES})

add_library(gazebo_ros_planar_move src/gazebo_ros_planar_move.cpp)
set_target_properties(gazebo_ros_planar_move PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_planar_move PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_planar_move ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##
## Add your new plugin here
##

## Template
add_library(gazebo_ros_template src/gazebo_ros_template.cpp)
set_target_properties(gazebo_ros_template PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_ros_template PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(gazebo_ros_template ${GAZEBO_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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
  gazebo_ros_prosilica
  gazebo_ros_force
  gazebo_ros_joint_trajectory
  gazebo_ros_joint_pose_trajectory
  gazebo_ros_diff_drive
  gazebo_ros_skid_steer_drive
  gazebo_ros_video
  gazebo_ros_planar_move
  pub_joint_trajectory_test  
  gazebo_ros_gpu_laser
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS scripts/set_wrench.py scripts/set_pose.py scripts/gazebo_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
