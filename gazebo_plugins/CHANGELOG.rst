^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* gazebo_plugins: added missing initialization of GazeboRosDepthCamera::advertised_
* gazebo_plugins: fixed depth and openni kinect camera plugin segfaults
* gazebo_plugins: terminate the service thread properly on destruction of a PubMutliQueue object without shuting down ros
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* fix for terminating the service_thread_ in PubQueue.h
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
