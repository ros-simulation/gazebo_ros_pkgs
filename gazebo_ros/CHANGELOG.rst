^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gazebo_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.6 (2014-09-01)
------------------
* Merge pull request `#232 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/232>`_ from ros-simulation/fix_get_physics_properties_non_ode
  Fix get physics properties non ode
* Merge pull request `#183 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/183>`_ from ros-simulation/issue_182
  Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
* check physics engine type before calling set_physics_properties and get_physics_properteis
* check physics engine type before calling set_physics_properties and get_physics_properteis
* Fixes for calling GetParam() with different physic engines.
* 2.3.6
* Update changelogs for the upcoming release
* Fixed boost any cast
* Removed a few warnings
* Update for hydro + gazebo 1.9
* Fix build with gazebo4 and indigo
* Fix STL iterator errors, misc. cppcheck (`#182 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/182>`_)
  There were some errors in STL iterators.
  Initialized values of member variables in constructor.
  Removed an unused variable (model_name).
* Contributors: Carlos Agüero, John Hsu, Jose Luis Rivero, Nate Koenig, Steven Peters, hsu, osrf

2.4.5 (2014-08-18)
------------------
* Port fix_build branch for indigo-devel
  See pull request `#221 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/221>`_
* Contributors: Jose Luis Rivero

2.4.4 (2014-07-18)
------------------
* Fix repo names in package.xml's
* fix issue `#198 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/198>`_
  Operator ``==`` is not recognized by sh scripts.
* Add verbose parameter
  Add verbose parameter for --verbose gazebo flag
* added osx support for gazebo start scripts
* Contributors: Arn-O, Jon Binney, Markus Achtelik, Vincenzo Comito

2.4.3 (2014-05-12)
------------------
* added osx support for gazebo start scripts
* Remove gazebo_ros dependency on gazebo_plugins
* Contributors: Markus Achtelik, Steven Peters

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
* gazebo_ros: [less-than-minor] fix newlines
* gazebo_ros: remove assignment to self
  If this is needed for any twisted reason, it should be made clear
  anyway. Assuming this line is harmless and removing it because it
  generates cppcheck warnings.
* Contributors: Paul Mathieu

2.3.4 (2013-11-13)
------------------
* rerelease because sdformat became libsdformat, but we also based change on 2.3.4 in hydro-devel.
* remove debug statement
* fix sdf spawn with initial pose
* fix sdf spawn with initial pose
* Merge branch 'hydro-devel' into ``spawn_model_pose_fix``
* fix indentation
* Merge pull request `#142 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/142>`_ from hsu/hydro-devel
  fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion
* Merge pull request `#140 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/140>`_ from ``v4hn/spawn_model_sleep``
  replace time.sleep by rospy.Rate.sleep
* fix spawn initial pose.  When model has a non-zero initial pose and user specified initial model spawn pose, add the two.
* fix issue `#38 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/38>`_, gui segfault on model deletion by removing an obsolete call to set selected object state to "normal".
* replace time.sleep by rospy.Rate.sleep
  time was not even imported, so I don't know
  why this could ever have worked...
* Add time import
  When using the -wait option the script fails because is missing the time import
* Use pre-increment for iterators
* Fix iterator erase() problems

2.4.0 (2013-10-14)
------------------

2.3.3 (2013-10-10)
------------------
* Cleaned up unnecessary debug output that was recently added
* Fixed issue where ``catkin_find`` returns more than one library if it is installed from both source and debian

2.3.2 (2013-09-19)
------------------
* Make gazebo includes use full path
  In the next release of gazebo, it will be required to use the
  full path for include files. For example,
  `include <physics/physics.hh>` will not be valid
  `include <gazebo/physics/physics.hh>` must be done instead.
* update gazebo includes
* Fixed a minor typo in spawn_model error message when `-model` not specified

2.3.1 (2013-08-27)
------------------
* Cleaned up template, fixes for header files

2.3.0 (2013-08-12)
------------------
* gazebo_ros: fixed missing dependency on TinyXML
* gazebo_plugins: replace deprecated boost function
  This is related to `this gazebo issue <https://bitbucket.org/osrf/gazebo/issue/581/boost-shared_-_cast-are-deprecated-removed>`_

2.2.1 (2013-07-29)
------------------

2.2.0 (2013-07-29)
------------------
* Switched to pcl_conversions
* Remove find_package(SDF) from CMakeLists.txt
  It is sufficient to find gazebo, which will export the information
  about the SDFormat package.

2.1.5 (2013-07-18)
------------------
* gazebo_ros: fixed variable names in gazebo_ros_paths_plugin

2.1.4 (2013-07-14)
------------------

2.1.3 (2013-07-13)
------------------

2.1.2 (2013-07-12)
------------------
* Added author
* Tweak to make SDFConfig.cmake
* Cleaned up CMakeLists.txt for all gazebo_ros_pkgs
* Cleaned up gazebo_ros_paths_plugin
* 2.1.1

2.1.1 (2013-07-10 19:11)
------------------------
* Merge branch 'hydro-devel' of github.com:ros-simulation/gazebo_ros_pkgs into hydro-devel
* Reduced number of debug msgs
* Fixed physics dynamic reconfigure namespace
* gazebo_ros_api_plugin: set `plugin_loaded_` flag to true in
  GazeboRosApiPlugin::Load() function
* Actually we need `__init__.py`
* Cleaning up code
* Moved gazebo_interface.py from gazebo/ folder to gazebo_ros/ folder
* Removed searching for plugins under 'gazebo' pkg because of rospack warnings
* Minor print modification
* Added dependency to prevent missing msg header, cleaned up CMakeLists

2.1.0 (2013-06-27)
------------------
* gazebo_ros: added deprecated warning for packages that use gazebo as
  package name for exported paths
* Hiding some debug info
* gazebo_ros: use rosrun in debug script, as rospack find gazebo_ros returns the wrong path in install space
* Hide Model XML debut output to console
* gazebo_ros_api_plugin.h is no longer exposed in the include folder
* Added args to launch files, documentation
* Merge pull request `#28 <https://github.com/ros-simulation/gazebo_ros_pkgs/issues/28>`_ from osrf/no_roscore_handling
  Better handling of gazebo_ros run when no roscore started
* gazebo_ros: also support gazebo instead of gazebo_ros as package name for plugin_path, gazebo_model_path or gazebo_media_path exports
* gazebo_plugins/gazebo_ros: fixed install directories for include files and gazebo scripts
* changed comment location
* added block comments for walkChildAddRobotNamespace
* SDF and URDF now set robotNamespace for plugins
* Better handling of gazebo_ros run when no roscore started

2.0.2 (2013-06-20)
------------------
* Added Gazebo dependency
* changed the final kill to send a SIGINT and ensure only the last background process is killed.
* modified script to work in bash correctly (tested on ubuntu 12.04 LTS)

2.0.1 (2013-06-19)
------------------
* Incremented version to 2.0.1
* Fixed circular dependency, removed deprecated pkgs since its a stand alone pkg
* Shortened line lengths of function headers

2.0.0 (2013-06-18)
------------------
* Changed version to 2.0.0 based on gazebo_simulator being 1.0.0
* Updated package.xml files for ros.org documentation purposes
* Combined updateSDFModelPose and updateSDFName, added ability to spawn SDFs from model database, updates SDF version to lastest in parts of code, updated the tests
* Renamed Gazebo model to SDF model, added ability to spawn from online database
* Fixed really obvious error checking bug
* Deprecated -gazebo arg in favor of -sdf tag
* Reordered services and messages to be organized and reflect documentation. No code change
* Cleaned up file, addded debug info
* Merged changes from Atlas ROS plugins, cleaned up headers
* Small fixes per ffurrer's code review
* Deprecated warnings fixes
* Cleaned up comment blocks - removed from .cpp and added to .h
* Merged branches and more small cleanups
* Small compile error fix
* Standardized function and variable naming convention, cleaned up function comments
* Reduced debug output and refresh frequency of robot spawner
* Converted all non-Gazebo pointers to boost shared_ptrs
* Removed old Gazebo XML handling functions - has been replaced by SDF, various code cleanup
* Removed the physics reconfigure node handle, switched to async ROS spinner, reduced required while loops
* Fixed shutdown segfault, renamed `rosnode_` to `nh_`, made all member variables have `_` at end, formatted functions
* Added small comment
* adding install for gazebo_ros launchfiles
* Formatted files to be double space indent per ROS standards
* Started fixing thread issues
* Fixing install script names and adding gzserver and gdbrun to install command
* Fixed deprecated warnings, auto formatted file
* Cleaned up status messages
* Added -h -help --help arguemnts to spawn_model
* Removed broken worlds
* Removed deprecated namespace argument
* Using pkg-config to find the script installation path.
  Corrected a bash typo with client_final variable in gazebo script.
* Cleaning up world files
* Deprecated fix
* Moved from gazebo_worlds
* Cleaning up launch files
* Moved from gazebo_worlds
* Fixing renaming errors
* Updated launch and world files and moved to gazebo_ros
* Combined gzclient and gzserver
* Added finished loading msg
* All packages building in Groovy/Catkin
* Imported from bitbucket.org
