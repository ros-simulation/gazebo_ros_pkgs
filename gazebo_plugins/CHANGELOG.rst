^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.7 (2016-06-10)
------------------

2.5.6 (2016-04-28)
------------------
* fix gazebo7 deprecation warnings on kinetic
* Contributors: Steven Peters

2.5.5 (2016-04-27)
------------------
* merge indigo, jade to kinetic-devel
* Accept /world for the frameName parameter in gazebo_ros_p3d
* Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * Upgrade to gazebo 7 and remove deprecated driver_base dependency
  * disable gazebo_ros_control until dependencies are met
  * Remove stray backslash
* Update maintainer for Kinetic release
* use HasElement in if condition
* Contributors: Hugo Boyer, Jackie Kay, Jose Luis Rivero, Steven Peters, William Woodall, Yuki Furuta

2.5.3 (2016-04-11)
------------------

2.5.2 (2016-02-25)
------------------
* Fix row_step of openni_kinect plugin
* remove duplicated code during merge
* merging from indigo-devel
* Merge pull request `#368 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/368>`_ from l0g1x/jade-devel
  Covariance for published twist in skid steer plugin
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* gazebo_ros_utils.h: include gazebo_config.h
  Make sure to include gazebo_config.h,
  which defines the GAZEBO_MAJOR_VERSION macro
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
  cherry-picked from indigo-devel
  Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Fix compiler error with SetHFOV
  In gazebo7, the rendering::Camera::SetHFOV function
  is overloaded with a potential for ambiguity,
  as reported in the following issue:
  https://bitbucket.org/osrf/gazebo/issues/1830
  This fixes the build by explicitly defining the
  Angle type.
* Add missing boost header
  Some boost headers were remove from gazebo7 header files
  and gazebo_ros_joint_state_publisher.cpp was using it
  implicitly.
* Fix gazebo7 build errors
  The SensorPtr types have changed from boost:: pointers
  to std:: pointers,
  which requires boost::dynamic_pointer_cast to change to
  std::dynamic_pointer_cast.
  A helper macro is added that adds a `using` statement
  corresponding to the correct type of dynamic_pointer_cast.
  This macro should be narrowly scoped to protect
  other code.
* Fix gazebo6 deprecation warnings
  Several RaySensor functions are deprecated in gazebo6
  and are removed in gazebo7.
  The return type is changed to use ignition math
  and the function name is changed.
  This adds ifdef's to handle the changes.
* Publish organized point cloud from openni_kinect plugin
* Added covariance matrix for published twist message in the skid steer plugin, as packages such as robot_localization require an associated non-zero covariance matrix
* Added a missing initialization inside Differential Drive
* 2.4.9
* Generate changelog
* Merge pull request `#335 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/335>`_ from pal-robotics-forks/add_range_sensor_plugin
  Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Import changes from jade-branch
* Add range world and launch file
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Add ifdefs to fix build with gazebo2
  It was broken by `#315 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/315>`_.
  Fixes `#321 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/321>`_.
* Use Joint::SetParam for joint velocity motors
  Before gazebo5, Joint::SetVelocity and SetMaxForce
  were used to set joint velocity motors.
  The API has changed in gazebo5, to use Joint::SetParam
  instead.
  The functionality is still available through the SetParam API.
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, John Hsu, Jose Luis Rivero, Kentaro Wada, Krystian, Mirko Ferrati, Steven Peters, hsu

2.5.1 (2015-08-16)
------------------
* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added gazebo version check
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev (`#323 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/323>`_)
  Declare the dependency.
  It can be fixed later if we don't want it.
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

* Port of Pal Robotics range sensor plugin to Jade
* Added a comment about the need of libgazebo5-dev in runtime
* Added gazebo version check
* Added missing files
* Added elevator plugin
* Use c++11
* run_depend on libgazebo5-dev
* Contributors: Jose Luis Rivero, Nate Koenig, Steven Peters

2.5.0 (2015-04-30)
------------------
* run_depend on libgazebo5-dev instead of gazebo5
* Changed the rosdep key for gazebo to gazebo5, for Jade Gazebo5 will be used.
* Contributors: Steven Peters, William Woodall

2.4.9 (2015-08-16)
------------------
* Adds range plugin for infrared and ultrasound sensors from PAL Robotics
* Import changes from jade-branch
* Add range world and launch file
* Add ifdefs to fix build with gazebo2
* Use Joint::SetParam for joint velocity motors
* Set GAZEBO_CXX_FLAGS to fix c++11 compilation errors
* Contributors: Bence Magyar, Jose Luis Rivero, Steven Peters

2.4.8 (2015-03-17)
------------------
* fixed mistake at calculation of joint velocity
* [gazebo_ros_diff_drive] force call SetMaxForce since this Joint::Reset in gazebo/physics/Joint.cc reset MaxForce to zero and ModelPlugin::Reset is called after Joint::Reset
* add PointCloudCutoffMax
* Contributors: Kei Okada, Michael Ferguson, Sabrina Heerklotz

2.4.7 (2014-12-15)
------------------
* fix missing ogre flags: removed from gazebo default (5.x.x candidate) cmake config
* Fixing handling of non-world frame velocities in setModelState.
* fix missing ogre flags (removed from gazebo cmake config)
* change header to use opencv2/opencv.hpp issue `#274 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/274>`_
* Update Gazebo/ROS tutorial URL
* Merge pull request `#237 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/237>`_ from ros-simulation/update_header_license
  Update header license for Indigo
* Contributors: John Hsu, Jose Luis Rivero, Robert Codd-Downey, Tom Moore, hsu

2.4.6 (2014-09-01)
------------------
* Update gazebo_ros_openni_kinect.cpp
* merging from hydro-devel into indigo-devel
* Merge pull request `#204 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/204>`_ from fsuarez6/hydro-devel
  gazebo_plugins: Adding ForceTorqueSensor Plugin
* Updated to Apache 2.0 license
* Merge pull request `#180 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/180>`_ from vrabaud/indigo-devel
  remove PCL dependency
* merging
* check deprecation of gazebo::Joint::SetAngle by SetPosition
* compatibility with gazebo 4.x
* Update changelogs for the upcoming release
* Fix build with gazebo4 and indigo
* Added Gaussian Noise generator
* publish organized pointcloud from openni plugin
* Changed measurement direction to "parent to child"
* gazebo_plugin: Added updateRate parameter to the gazebo_ros_imu plugin
* gazebo_plugins: Adding ForceTorqueSensor Plugin
* remove PCL dependency
* ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
  ros_camera_utils: Adding CameraInfoManager to satisfy full ROS camera API (relies on https://github.com/ros-perception/image_common/pull/20 )
* Contributors: John Hsu, Jonathan Bohren, Jose Luis Rivero, Nate Koenig, Ryohei Ueda, Vincent Rabaud, fsuarez6, gborque, John Binney

2.4.5 (2014-08-18)
------------------
* Replace SetAngle with SetPosition for gazebo 4 and up
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero, Steven Peters

2.4.4 (2014-07-18)
------------------
* Merge branch 'hydro-devel' into indigo-devel
* gazebo_ros_diff_drive gazebo_ros_tricycle_drive encoderSource option names updated
* gazebo_ros_diff_drive is now able to use the wheels rotation of the optometry or the gazebo ground truth based on the 'odometrySource' parameter
* simple linear controller for the tricycle_drive added
* second robot for testing in tricycle_drive_scenario.launch added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* BDS licenses header fixed and tricycle drive plugin added
* format patch of hsu applied
* Updated package.xml
* Fix repo names in package.xml's
* ros diff drive supports now an acceleration limit
* Pioneer model: Diff_drive torque reduced
* GPU Laser test example added
* fixed gpu_laser to work with workspaces
* hand_of_god: Adding hand-of-god plugin
  ros_force: Fixing error messages to refer to the right plugin
* Remove unneeded dependency on pcl_ros
* minor fixes on relative paths in xacro for pioneer robot
* gazebo test model pionneer 3dx updated with xacro path variables
* pioneer model update for the multi_robot_scenario
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fixed camera to work with workspaces
* fixed links related to changed name
* diff drive name changed to multi robot scenario
* working camera added
* Merge remote-tracking branch 'upstream/hydro-devel' into hydro-devel
* fix in pioneer xacro model for diff_drive
* Laser colour in rviz changed
* A test model for the ros_diff_drive ros_laser and joint_state_publisher added
* the ros_laser checkes now for the model name and adds it als prefix
* joint velocity fixed using radius instead of diameter
* ROS_INFO on laser plugin added to see if it starts
* fetched with upstream
* gazebo_ros_diff_drive was enhanced to publish the wheels tf or the wheels joint state depending on two additinal xml options <publishWheelTF> <publishWheelJointState>
* Gazebo ROS joint state publisher added
* Contributors: Dave Coleman, John Hsu, Jon Binney, Jonathan Bohren, Markus Bader, Steven Peters

2.4.3 (2014-05-12)
------------------
* gazebo_plugins: add run-time dependency on gazebo_ros
* Merge pull request `#176 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/176>`_ from ros-simulation/issue_175
  Fix `#175 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/175>`_: dynamic reconfigure dependency error
* Remove unneeded dependency on pcl_ros
* Fix `#175 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/175>`_: dynamic reconfigure dependency error
* Contributors: Steven Peters

2.4.2 (2014-03-27)
------------------
* merging from hydro-devel
* bump patch version for indigo-devel to 2.4.1
* merging from indigo-devel after 2.3.4 release
* "2.4.0"
* catkin_generate_changelog
* Contributors: John Hsu

2.4.1 (2013-11-13)
------------------

2.3.5 (2014-03-26)
------------------
* update test world for block laser
* this corrects the right orientation of the laser scan and improves on comparison between 2 double numbers
* Initialize ``depth_image_connect_count_`` in openni_kinect plugin
* multicamera bad namespace. Fixes `#161 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/161>`_
  There was a race condition between GazeboRosCameraUtils::LoadThread
  creating the ros::NodeHandle and GazeboRosCameraUtils::Load
  suffixing the camera name in the namespace
* Use function for accessing scene node in gazebo_ros_video
* readded the trailing whitespace for cleaner diff
* the parent sensor in gazebo seems not to be active
* Contributors: Dejan Pangercic, Ian Chen, John Hsu, Jordi Pages, Toni Oliver, Ugo Cupcic

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* Simplify ``gazebo_plugins/CMakeLists.txt``
  Replace ``cxx_flags`` and ``ld_flags`` variables with simpler cmake macros
  and eliminate unnecessary references to ``SDFormat_LIBRARIES``, since
  they are already part of ``GAZEBO_LIBRARIES``.
* Put some cmake lists on multiple lines to improve readability.
* Add dependencies on dynamic reconfigure files
  Occasionally the build can fail due to some targets having an
  undeclared dependency on automatically generated dynamic
  reconfigure files (GazeboRosCameraConfig.h for example). This
  commit declares several of those dependencies.

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* gazebo_plugins: use shared pointers for variables shared among cameras
  It is not allowed to construct a shared_ptr from a pointer to a member
  variable.
* gazebo_plugins: moved initialization of shared_ptr members of
  GazeboRosCameraUtils to `GazeboRosCameraUtils::Load()`
  This fixes segfaults in gazebo_ros_depth_camera and
  gazebo_ros_openni_kinect as the pointers have not been initialized
  there.
* Use `RenderingIFace.hh`

2.3.2 (2013-09-19)
------------------
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  `include <physics/physics.hh>` will not be valid
  `include <gazebo/physics/physics.hh>` must be done instead.
* Merge branch 'hydro-devel' of `gazebo_ros_pkgs <github.com:ros-simulation/gazebo_ros_pkgs>`_ into synchronize_with_drcsim_plugins
* change includes to use brackets in headers for export
* per pull request comments
* Changed resolution for searchParam.
* Don't forget to delete the node!
* Removed info message on robot namespace.
* Retreive the tf prefix from the robot node.
* synchronize with drcsim plugins

2.3.1 (2013-08-27)
------------------
* Remove direct dependency on pcl, rely on the transitive dependency from pcl_ros
* Cleaned up template, fixes for header files

2.3.0 (2013-08-12)
------------------
* enable image generation when pointcloud is requested, as the generated image is used by the pointcloud
* gazebo_plugins: replace deprecated boost function
  This is related to this `gazebo issue #581 <https://bitbucket.org/osrf/gazebo/issue/581/boost-shared_-_cast-are-deprecated-removed>`_
* gazebo_plugins: fix linkedit issues
  Note: other linkedit errors were fixed upstream
  in gazebo
* gazebo_ros_openni_kinect plugin: adds publishing of the camera info
  again (fixes `#95 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/95>`_)
* Merge pull request `#90 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/90>`_ from piyushk/add_model_controller
  added a simple model controller plugin that uses a twist message
* renamed plugin from model controller to planar move
* prevents dynamic_reconfigure from overwritting update rate param on start-up
* removed anonymizer from include guard
* fixed odometry publication for model controller plugin
* added a simple model controller plugin that uses a twist message to control models

2.2.1 (2013-07-29)
------------------
* Added prosilica plugin to install TARGETS

2.2.0 (2013-07-29)
------------------
* Switched to pcl_conversions instead of using compiler flags for Hydro/Groovy PCL support
* fixed node intialization conflict between gzserver and gzclient. better adherance to gazebo style guidelines
* Fixed template
* removed ros initialization from plugins
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information about the SDFormat package.
* ROS Video Plugin for Gazebo - allows displaying an image stream in an OGRE texture inside gazebo. Also provides a fix for `#85 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/85>`_.
* patch a fix for prosilica plugin (startup race condition where `rosnode_` might still be NULL).
* Added explanation of new dependency in gazebo_ros_pkgs
* switch Prosilica camera from type depth to regular camera (as depth data were not used).
* migrating prosilica plugin from pr2_gazebo_plugins
* Removed tbb because it was a temporary dependency for a Gazebo bug
* SDF.hh --> sdf.hh
* Added PCL to package.xml

2.1.5 (2013-07-18)
------------------
* Include <sdf/sdf.hh> instead of <sdf/SDF.hh>
  The sdformat package recently changed the name of an sdf header
  file from SDF.hh to SDFImpl.hh; this change will use the lower-case
  header file which should work with old and new versions of sdformat
  or gazebo.

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------
* temporarily add tbb as a work around for `#74 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/74>`_

2.1.2 (2013-07-12)
------------------
* Fixed compatibility with new PCL 1.7.0
* Tweak to make SDFConfig.cmake
* Re-enabled dynamic reconfigure for camera utils - had been removed for Atlas
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Removed SVN references
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Small deprecated warning
* Fixed errors and deprecation warnings from Gazebo 1.9 and the sdformat split
* Source code formatting.
* Merge pull request `#59 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/59>`_ from ros-simulation/CMake_Tweak
  Added dependency to prevent missing msg header, cleaned up CMakeLists
* export diff drive and skid steer for other catkin packages
* install diff_drive and skid_steer plugins
* Added dependency to prevent missing msg header, cleaned up CMakeLists
* Added ability to switch off publishing TF.

2.1.0 (2013-06-27)
------------------
* gazebo_plugins: always use gazebo/ path prefix in include directives
* gazebo_plugins: call Advertise() directly after initialization has
  completed in gazebo_ros_openni_kinect and gazebo_ros_depth_camera
  plugins, as the sensor will never be activated otherwise
* Merge pull request `#41 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/41>`_ from ZdenekM/hydro-devel
  Added skid steering plugin (modified diff drive plugin).
* Merge pull request `#35 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/35>`_ from meyerj/fix_include_directory_installation_target
  Header files of packages gazebo_ros and gazebo_plugins are installed to the wrong location
* Rotation fixed.
* Skid steering drive plugin.
* gazebo_plugins: added missing initialization of `GazeboRosDepthCamera::advertised_`
* gazebo_plugins: fixed depth and openni kinect camera plugin segfaults
* gazebo_plugins: terminate the service thread properly on destruction of a PubMutliQueue object without shuting down ros
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* fix for terminating the `service_thread_` in PubQueue.h
* added differential drive plugin to gazebo plugins

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Check camera util is initialized before publishing - fix from Atlas

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Created tests for various spawning methods
* Added debug info to shutdown
* Fixed gazebo includes to be in <gazebo/...> format
* Cleaned up file, addded debug info
* Merge branch 'groovy-devel' into plugin_updates
* Merged changes from Atlas ROS plugins, cleaned up headers
* Merged changes from Atlas ROS plugins, cleaned up headers
* fix curved laser issue
* Combining Atlas code with old gazebo_plugins
* Combining Atlas code with old gazebo_plugins
* Small fixes per ffurrer's code review
* Added the robot namespace to the tf prefix.
  The tf_prefix param is published under the robot namespace and not the
  robotnamespace/camera node which makes it non-local we have to use the
  robot namespace to get it otherwise it is empty.
* findreplace ConnectWorldUpdateStart ConnectWorldUpdateBegin
* Fixed deprecated function calls in gazebo_plugins
* Deprecated warnings fixes
* Removed the two plugin tests that are deprecated
* Removed abandoned plugin tests
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
