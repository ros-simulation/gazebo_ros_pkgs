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

#include <boost/thread/mutex.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <gazebo/common/SystemPaths.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>
#include "rosbag/player.h" //TODO(tfoote) remove dependency
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <map>

namespace gazebo
{

typedef std::vector<std::string> V_string;
typedef std::map<std::string, std::string> M_string;

class GazeboRosbagPlugin : public SystemPlugin
{
public:
  GazeboRosbagPlugin() :
    world_created_(false),
    last_tick_time_(0.0)
  {
    // this->LoadPaths();
  }

  ~GazeboRosbagPlugin()
  {
    // Physics Dynamic Reconfigure

    load_gazebo_rosbag_plugin_event_.reset();
    time_update_event_.reset();
  };

  void Init()
  {

  }

  void Load(int argc, char** argv)
  {
    ROS_DEBUG_STREAM_NAMED("rosbag_plugin","Load");

    // connect to sigint event
    // sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&GazeboRosbagPlugin::shutdownSignal,this));

    // setup ros related
    if (!ros::isInitialized())
      ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);
    else
      ROS_WARN_NAMED("rosbag_plugin", "Something other than this gazebo_rosbag plugin started ros::init(...), command line arguments may not be parsed properly.");

    //TODO parse bag filename here

    // check if the ros master is available - required
    while(!ros::master::check())
    {
      ROS_WARN_STREAM_NAMED("rosbag_plugin","No ROS master - start roscore to continue...");
      // wait 0.5 second
      usleep(500*1000); // can't use ROS Time here b/c node handle is not yet initialized
      // 
      // if(stop_)
      // {
      //   ROS_WARN_STREAM_NAMED("rosbag_plugin","Canceled loading Gazebo rosbag plugin by sigint event");
      //   return;
      // }
    }

    // below needs the world to be created first
    load_gazebo_rosbag_plugin_event_ = \
      gazebo::event::Events::ConnectWorldCreated(boost::bind(&GazeboRosbagPlugin::loadGazeboRosbagPlugin,this,_1));
    time_update_event_ = \
      gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosbagPlugin::onTimeUpdate,this));



    nh_.reset(new ros::NodeHandle("~")); // advertise topics and services in this node's namespace

    if(nh_->hasParam("/coordinated_playback_bagfile"))
    {
      nh_->getParam("/coordinated_playback_bagfile", bagfile_filename_);
      ROS_INFO_NAMED("rosbag_plugin", "Bagfile for playback: [%s]", bagfile_filename_.c_str());

      loadBagFiles();
      advertiseAllPublishers();
      seekToCurrentTime();
    }

    ROS_INFO_NAMED("rosbag_plugin", "Finished loading Gazebo Rosbag Plugin.");
  }

  void seekToCurrentTime()
  {
    ROS_INFO_NAMED("rosbag_plugin", "Seeking forward until we find a new message");
    //Seek to starting time
    boost::mutex::scoped_lock slock(view_lock_);
    it_ = view_.begin();
    for (; it_ != view_.end(); ++it_)
    {
      ros::Time message_time = it_->getTime();
      if (message_time >= current_tick_time_) {
        break;
      }
    }
    ROS_INFO_NAMED("rosbag_plugin", "Done: Seeking forward until we find a new message");
  }

  void loadBagFiles()
  {
    ROS_INFO_NAMED("rosbag_plugin", "loading bag file %s", bagfile_filename_.c_str());
    boost::shared_ptr<rosbag::Bag> bag = boost::make_shared<rosbag::Bag>();
    bag->open(bagfile_filename_, rosbag::bagmode::Read);
    
    boost::mutex::scoped_lock slock(view_lock_);
    bags_.push_back(bag);
    view_.addQuery(*bag); //TODO(tfoote) add support for multiple bags add them here.
  }

  // Advertise all of our messages
  void advertiseAllPublishers()
  {
    //TODO(tfoote) magic values
    std::string prefix = std::string("");
    uint32_t queue_size = 10;

    boost::mutex::scoped_lock slock(view_lock_);
    foreach(const rosbag::ConnectionInfo* c, view_.getConnections())
    {
      if(c->topic == std::string("/clock")){
        //Don't create a publisher for "/clock"
        continue;
      }
      ros::M_string::const_iterator header_iter = c->header->find("callerid");
      std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

      std::string callerid_topic = callerid + c->topic;

      PublisherMap::iterator pub_iter = publishers_.find(callerid_topic);
      if (pub_iter == publishers_.end()) {

          ros::AdvertiseOptions opts = createAdvertiseOptions(c, queue_size, prefix);

          ros::Publisher pub = nh_->advertise(opts);
          publishers_.insert(publishers_.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));

          pub_iter = publishers_.find(callerid_topic);
      }
    }
  }
  
  void playbackRecentMessages()
  {
    boost::mutex::scoped_lock slock(view_lock_);
    while (it_ != view_.end() && ros::ok())
    {
      std::string topic = it_->getTopic();
      std::string callerid = it_->getCallerId();
      ros::Time message_time = it_->getTime();
      // Skip clock gazebo will be publishing it
      if (topic == "/clock") {
        ROS_DEBUG_THROTTLE_NAMED(1.0, "rosbag_plugin", "I won't publish clock");
        it_++;
        continue;
      }
      
      // Publish until up to current time
      if (message_time < current_tick_time_) {
        PublisherMap::iterator pub_iter = publishers_.find(callerid + topic);
        ROS_ASSERT(pub_iter != publishers_.end());
        pub_iter->second.publish(*it_);
        ROS_DEBUG_NAMED("rosbag_plugin", "I published %s at %g", topic.c_str(), current_tick_time_.toSec());
        it_++;
      }
      else
      {
        break;
      }
    }
  }

  void onTimeUpdate()
  {
    // On time update 
    if (!world_) return;

    gazebo::common::Time currentTime = world_->SimTime();
    current_tick_time_.fromSec(currentTime.Double());

    // Time jumped backwards break out and restart.
    if (last_tick_time_ > current_tick_time_) {
      ROS_INFO_NAMED("rosbag_plugin", "Time jumped backwards resetting bag playback.");
      seekToCurrentTime();
    }
    last_tick_time_ = current_tick_time_;
    playbackRecentMessages();
  }

  void loadGazeboRosbagPlugin(std::string world_name)
  {
    ROS_INFO_NAMED("rosbag_plugin", "world name argument: [%s]",world_name.c_str());
    
    // make sure things are only called once
    lock_.lock();
    if (world_created_)
    {
      lock_.unlock();
      return;
    }

    // set flag to true and load this plugin
    world_created_ = true;
    lock_.unlock();

    world_ = gazebo::physics::get_world(world_name);
    ROS_INFO_NAMED("rosbag_plugin", "world created: [%s]",world_->Name().c_str());
    if (!world_)
    {
      ROS_FATAL_NAMED("rosbag_plugin", "cannot load gazebo rosbag server plugin, physics::get_world() fails to return world");
      return;
    }
    
  }

private:
  typedef std::map<std::string, ros::Publisher> PublisherMap;

  gazebo::physics::WorldPtr world_;
  bool world_created_;
  gazebo::event::ConnectionPtr load_gazebo_rosbag_plugin_event_;
  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  boost::mutex lock_;

  boost::shared_ptr<ros::NodeHandle> nh_;
  gazebo::event::ConnectionPtr time_update_event_;


  boost::shared_ptr<boost::thread> rosbag_playback_thread_;
  std::string bagfile_filename_;

  ros::Time current_tick_time_;
  ros::Time last_tick_time_;

  std::vector<boost::shared_ptr<rosbag::Bag>> bags_;
  boost::mutex view_lock_;
  rosbag::View view_;
  rosbag::View::iterator it_;
  
  PublisherMap publishers_;
  
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosbagPlugin)

}
