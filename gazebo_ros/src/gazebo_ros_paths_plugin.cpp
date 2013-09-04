/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* 
 * Author: John Hsu, Nate Koenig, Dave Coleman
 * Desc: External interfaces for Gazebo
 */

#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>

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

  /**
   * @brief Set Gazebo Path/Resources Configurations GAZEBO_MODEL_PATH, PLUGIN_PATH and 
            GAZEBO_MEDIA_PATH by adding paths to GazeboConfig based on ros::package
   */
  void LoadPaths()
  {
    // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
    gazebo::common::SystemPaths::Instance()->gazeboPathsFromEnv = false;
    std::vector<std::string> gazebo_media_paths;
    ros::package::getPlugins("gazebo_ros","gazebo_media_path",gazebo_media_paths);
    for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
    {
      ROS_DEBUG("med path %s",iter->c_str());
      gazebo::common::SystemPaths::Instance()->AddGazeboPaths(iter->c_str());
    }

    // set gazebo plugins paths by adding all packages that exports "plugin_path" for gazebo
    gazebo::common::SystemPaths::Instance()->pluginPathsFromEnv = false;
    std::vector<std::string> plugin_paths;
    ros::package::getPlugins("gazebo_ros","plugin_path",plugin_paths);
    for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
    {
      ROS_DEBUG("plugin path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddPluginPaths(iter->c_str());
    }

    // set model paths by adding all packages that exports "gazebo_model_path" for gazebo
    gazebo::common::SystemPaths::Instance()->modelPathsFromEnv = false;
    std::vector<std::string> model_paths;
    ros::package::getPlugins("gazebo_ros","gazebo_model_path",model_paths);
    for (std::vector<std::string>::iterator iter=model_paths.begin(); iter != model_paths.end(); iter++)
    {
      ROS_DEBUG("model path %s",(*iter).c_str());
      gazebo::common::SystemPaths::Instance()->AddModelPaths(iter->c_str());
    }

    // set .gazeborc path to something else, so we don't pick up default ~/.gazeborc
    std::string gazeborc = ros::package::getPath("gazebo_ros")+"/.do_not_use_gazeborc";
    setenv("GAZEBORC",gazeborc.c_str(),1);
  }

};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosPathsPlugin)

}

