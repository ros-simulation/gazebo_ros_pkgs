# Sensors supported in Gazebo ROS packages

## Initial note

As detailed in the CONTRIBUITON.md guide, Gazebo ROS packages are a wrapper that 
connects upstream Gazebo simulations with the ROS framework. Although there is
some code in this repo that implements simulations, this should be an exception.
Please checkout the CONTRIBUTION.md guide for more details about which code
should be submitted to this repository.

   - description: 
   - status:
   - gazebo plugin:
   - example:


### List of supported sensors

 * camera_synchronizer
 * vision_reconfigure
   - description: 
   - status: needs-work, dead?
   - gazebo plugin: CameraSynchronizerConfig
   - example:

 * gazebo_ros_block_laser
   - description: implements ray based sensors (lasers). Publishes
     sensors_msgs::PointCloud.
   - status: maintained
   - gazebo plugin: RayPlugin
   - example: gazebo_plugins/test/test_worlds/gazebo_ros_block_laser.world
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world 

 * gazebo_ros_bumper
   - description:  implements a contact sensor. Publishes
     gazebo_msgs::ContactsState.
   - status: maintained, needs-cleanup
   - gazebo plugin:
   - example: gazebo_plugins/test/bumper_test/gazebo_ros_bumper.world: 
   - test: gazebo_plugins/test/test_worlds/bumper_test.world:

 * gazebo_ros_camera
   gazebo_ros_camera_utils
   - description: implements a camera. Publishes: sensor_msgs::Image, sensor_msgs::CameraInfo
   - status: maintained, dynamic-reconfigure
   - gazebo plugin: CameraPlugin, GazeboRosCameraUtils
   - example: gazebo_plugins/test/test_worlds/gazebo_ros_block_laser.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_depth_camera.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_camera.world:
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world:
              gazebo_plugins/test/multi_robot_scenario/xacro/camera/camera.xacro:

 * gazebo_ros_depth_camera
   - description: implements depth camera based sensors. Publishes: sensor_msgs::Image, 
     sensor_msgs::CameraInfo, sensors_msgs::PointCloud2
   - status: maintained, not-just-a-wrapper, needs-cleanup
   - gazebo plugin: DepthCameraPlugin
   - example: gazebo_plugins/test/test_worlds/gazebo_ros_depth_camera.world
              gazebo_plugins/test/test_worlds/gazebo_ros_trimesh_collision.world

 * gazebo_ros_diff_drive
   - description: implements a diff drive base. Publishes: sensor_msgs::JointState,
     nav_msgs::Odometry
   - status: maintained, not-just-a-wrapper, rosparams, needs-cleanup
   - gazebo plugin: ModelPlugin (generic)
   - example:  gazebo_plugins/test/multi_robot_scenario/xacro/p3dx/pioneer3dx_plugins.xacro
               gazebo_plugins/test/multi_robot_scenario/launch/pioneer3dx.gazebo.launch

 * gazebo_ros_elevator
   - description: implements an elevator
   - status: maintained
   - gazebo plugin: ElevatorPlugin
   - example: gazebo_plugins/test/test_worlds/elevator.world

 * gazebo_ros_f3d
   - description: controller that fake (emtpy publisher) a 6 dof force sensor
     Publishes geometry_msgs::WrenchStamped
   - status: stub
   - gazebo plugin: ModelPlugin (generic)
   - example: --

 * gazebo_ros_force
   - description: collects data from a ROS topic and applies wrench to a body accordingly.
   - status: maintained, doxygen
   - gazebo plugin: ModelPlugin (generic)
   - example: --

 * gazebo_ros_ft_sensor
   - description: implements Force/Torque sensor. Publishes: geometry_msgs/WrenchStamped messages
   - status: maintained, doxygen
   - gazebo plugin: ModelPlugin (generic)
   - example: --

 * gazebo_ros_gpu_laser
   - description:  implements GPU laser based sensors. Publishes: 
     sensor_msgs::LaserScan
   - status: maintained
   - gazebo plugin: GpuRayPlugin
   - example: gazebo_plugins/test/test_worlds/gazebo_ros_gpu_laser.world
              gazebo_plugins/test/multi_robot_scenario/xacro/laser/hokuyo_gpu.xacro

 * gazebo_ros_hand_of_god
   - description: Drives floating object around based on the location of a TF frame
   - status: maintained, doxygen
   - gazebo plugin: ModelPlugin (generic)
   - example: --

 * gazebo_ros_imu
   - description: implements an IMU sensor. Publishes: sensor_msgs::Imu 
   - status: maintained, not-just-a-wrapper
   - gazebo plugin: ModelPlugin (generic)
   - example: --

 * gazebo_ros_joint_pose_trajectory
 * gazebo_ros_joint_state_publisher
 * gazebo_ros_joint_trajectory
 * gazebo_ros_laser
 * gazebo_ros_multicamera
 * gazebo_ros_openni_kinect
 * gazebo_ros_p3d
 * gazebo_ros_planar_move
 * gazebo_ros_projector
 * gazebo_ros_prosilica
 * gazebo_ros_range
 * gazebo_ros_skid_steer_drive
 * gazebo_ros_template
 * gazebo_ros_tricycle_drive
 * gazebo_ros_utils
 * gazebo_ros_video
 * hokuyo_node
 * MultiCameraPlugin
 * 
