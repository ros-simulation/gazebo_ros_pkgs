/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* 
 * Author: John Hsu, Nate Koenig, Dave Coleman
 * Desc: External interfaces for Gazebo
 */

#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/package.h>

#include <map>

namespace gazebo {

typedef std::vector<std::string> V_string;
typedef std::map<std::string, std::string> M_string;

class ROSPackageURISchemeResolver : public common::URISchemeResolver
{
  public: ROSPackageURISchemeResolver() {}

  public: virtual ~ROSPackageURISchemeResolver() {}

  public: virtual std::string GetHandledURIScheme()
  {
    return "model";
  }

  public: virtual std::string ResolveURI(const std::string &_uri)
  {
    int index = _uri.find("://");
    std::string filename;
    std::string prefix = _uri.substr(0, index);

    if (prefix != "model")
    {
        gzerr << "URI " << _uri << " with scheme '" << prefix
              << "' given to a resolver for scheme 'model'" << std::endl;
        return filename;
    }

    std::string suffix = _uri.substr(index + 3, _uri.size() - index - 3);
    int firstSlashIndex = suffix.find("/");

    if (firstSlashIndex == suffix.npos)
    { // we cannot handle URIs without a package name
      return filename;
    }

    std::string package = suffix.substr(0, firstSlashIndex);
    std::string pathInPackage = suffix.substr(firstSlashIndex + 1, suffix.size() - firstSlashIndex - 1);

    std::string packagePath = ros::package::getPath(package);
    if (packagePath.empty())
    {
      return filename;
    }

    filename = (boost::filesystem::path(packagePath) / pathInPackage).string();

    gzdbg << "Resolved " << _uri << " to " << filename << std::endl;

    return filename;
  }
};

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

    gazebo::common::URISchemeResolverPtr resolver;
    resolver.reset(new ROSPackageURISchemeResolver());
    gazebo::common::SystemPaths::Instance()->AddURISchemeResolver(resolver);
  }

};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosPathsPlugin)

}

