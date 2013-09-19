^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.2 (2013-09-19)
------------------

2.3.1 (2013-08-27)
------------------
* Cleaned up template, fixes for header files
* Renamed plugin to match file name, tweaked CMakeLists
* Created a header file for the ros_control gazebo plugin

2.3.0 (2013-08-12)
------------------
* Renamed ros_control_plugin, updated documentation

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------
* Standardized the way ROS nodes are initialized in gazebo plugins
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.
* Merge branch 'hydro-devel' into tranmission_parsing
* Doc and debug update
* Merged hydro-devel
* Hid debug info
* Merged from Hydro-devel
* Merge branch 'hydro-devel' into tranmission_parsing
* Moved trasmission parsing to ros_control

2.1.5 (2013-07-18)
------------------

2.1.4 (2013-07-14)
------------------
* Fixed for Jenkins broken dependency on SDF in ros_control

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Fixed errors and deprecation warnings from Gazebo 1.9 and the sdformat split
* making RobotHWSim::initSim pure virtual
* Cleaning up code
* Adding install targets

2.1.0 (2013-06-27)
------------------
* Made version match the rest of gazebo_ros_pkgs per bloom
* Added dependency on ros_controllers
* Clarifying language in readme
* Made default period Gazebo's period
* Made control period optional
* Tweaked README
* Added support for reading <tranmission> tags and other cleaning up
* Renamed RobotSim to RobotHWSim
* Renaming all gazebo_ros_control stuff to be in the same package
* Refactoring gazebo_ros_control packages into a single package, removing exampls (they will go elsewhere)
* updating readme for gazebo_ros_control
* Merging in gazebo_ros_control
* making gazebo_ros_control a metapackage
* Moving readme
* Merging readmes
* eating this
* Merging gazebo_ros_control and ros_control_gazebo

2.0.2 (2013-06-20)
------------------

2.0.1 (2013-06-19)
------------------

2.0.0 (2013-06-18)
------------------
