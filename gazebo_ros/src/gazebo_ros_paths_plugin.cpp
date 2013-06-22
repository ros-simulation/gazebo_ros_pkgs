/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: External interfaces for Gazebo
 * Author: John Hsu adapted original gazebo main.cc by Nate Koenig
 * Date: 25 Apr 2010
 * SVN: $Id: main.cc 8598 2010-03-22 21:59:24Z hsujohnhsu $
 */

#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#include <common/SystemPaths.hh>
#include <common/Plugin.hh>

#include <ros/ros.h>
#include <ros/package.h>

#include <map>

namespace gazebo
{

typedef std::vector<std::string> V_string;
typedef std::map<std::string, std::string> M_string;

class GazeboRosPathsPlugin : public SystemPlugin
{
public:
  GazeboRosPathsPlugin()
  {
    this->LoadPaths();
  }

  ~GazeboRosPathsPlugin()
  {
  };

  void Init()
  {
  }

  void Load(int argc, char** argv)
  {
  }

  /* copying ros::package::command() and ros::package::getPlugins() in here due to */
  /* a bug with compiler flag -Bsymbolic-functions                                 */
  /* see ticket https://code.ros.org/trac/ros/ticket/2977                          */
  /* this flag needs to be removed from new catkin debbuild before we can resume   */
  /* usage or ros::package::getPlugins() library from roslib (libroslib.so).       */
  void rosPackageCommandDebug(const std::string& cmd, V_string& output)
  {
    std::string out_string = ros::package::command(cmd);
    V_string full_list;
    boost::split(full_list, out_string, boost::is_any_of("\r\n"));

    // strip empties
    V_string::iterator it = full_list.begin();
    V_string::iterator end = full_list.end();
    for (; it != end; ++it)
    {
      if (!it->empty())
      {
        output.push_back(*it);
      }
    }
  }
  /* copying ros::package::command() and ros::package::getPlugins() in here due to */
  /* a bug with compiler flag -Bsymbolic-functions                                 */
  /* see ticket https://code.ros.org/trac/ros/ticket/2977                          */
  /* this flag needs to be removed from new catkin debbuild before we can resume   */
  /* usage or ros::package::getPlugins() library from roslib (libroslib.so).       */
  void rosPackageGetPluginsDebug(const std::string& package, const std::string& attribute, V_string& plugins)
  {
    M_string plugins_map;
    rosPackageGetPluginsDebug(package, attribute, plugins_map);
    M_string::iterator it = plugins_map.begin();
    M_string::iterator end = plugins_map.end();
    for (; it != end; ++it)
    {
      plugins.push_back(it->second);
    }
  }
  /* copying ros::package::command() and ros::package::getPlugins() in here due to */
  /* a bug with compiler flag -Bsymbolic-functions                                 */
  /* see ticket https://code.ros.org/trac/ros/ticket/2977                          */
  /* this flag needs to be removed from new catkin debbuild before we can resume   */
  /* usage or ros::package::getPlugins() library from roslib (libroslib.so).       */
  void rosPackageGetPluginsDebug(const std::string& package, const std::string& attribute, M_string& plugins)
  {
    V_string lines;
    rosPackageCommandDebug("plugins --attrib=" + attribute + " " + package, lines);

    V_string::iterator it = lines.begin();
    V_string::iterator end = lines.end();
    for (; it != end; ++it)
    {
      V_string tokens;
      boost::split(tokens, *it, boost::is_any_of(" "));

      if (tokens.size() >= 2)
      {
        std::string package = tokens[0];
        std::string rest = boost::join(V_string(tokens.begin() + 1, tokens.end()), " ");
        plugins[package] = rest;
      }
    }
  }
  void LoadPaths()
  {

    // We are now using system installed OGRE,
    // and sourcing gazebo installed setup.sh in ${CMAKE_INSTALL_PREFIX}/share/gazebo/setup.sh
    // setting Gazebo Path/Resources Configurations
    //   GAZEBO_RESOURCE_PATH, GAZEBO_PLUGIN_PATH and OGRE_RESOURCE_PATH for
    //   GazeboConfig::gazeboPaths, GazeboConfig::pluginPaths and GazeboConfig::ogrePaths
    // by adding paths to GazeboConfig based on ros::package
    // optional: setting environment variables according to ROS
    //           e.g. setenv("OGRE_RESOURCE_PATH",ogre_package_path.c_str(),1);

    // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
    gazebo::common::SystemPaths::Instance()->gazeboPathsFromEnv = false;
    std::vector<std::string> gazebo_media_paths;
    rosPackageGetPluginsDebug("gazebo","gazebo_media_path",gazebo_media_paths);
    for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
    {
      ROS_DEBUG("med path %s",iter->c_str());
      gazebo::common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
    }
    rosPackageGetPluginsDebug("gazebo_ros","gazebo_media_path",gazebo_media_paths);
    for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
    {
      ROS_DEBUG("med path %s",iter->c_str());
      gazebo::common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
    }

    // set gazebo plugins paths by adding all packages that exports "plugin_path" for gazebo
    gazebo::common::SystemPaths::Instance()->pluginPathsFromEnv = false;
    std::vector<std::string> plugin_paths;
    rosPackageGetPluginsDebug("gazebo","plugin_path",plugin_paths);
    for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
    {
      ROS_DEBUG("plugin path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddPluginPaths(iter->c_str());
    }
    rosPackageGetPluginsDebug("gazebo_ros","plugin_path",plugin_paths);
    for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
    {
      ROS_DEBUG("plugin path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddPluginPaths(iter->c_str());
    }

    // set model paths by adding all packages that exports "gazebo_model_path" for gazebo
    gazebo::common::SystemPaths::Instance()->modelPathsFromEnv = false;
    std::vector<std::string> model_paths;
    rosPackageGetPluginsDebug("gazebo","gazebo_model_path",model_paths);
    for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
    {
      ROS_DEBUG("model path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
    }
    rosPackageGetPluginsDebug("gazebo_ros","gazebo_model_path",model_paths);
    for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
    {
      ROS_DEBUG("model path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
    }

    // set .gazeborc path to something else, so we don't pick up default ~/.gazeborc
    std::string gazeborc = ros::package::getPath("gazebo_ros")+"/.do_not_use_gazeborc";
    setenv("GAZEBORC",gazeborc.c_str(),1);

  }
private:

};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosPathsPlugin)

}

