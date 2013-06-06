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
/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <gazebo_plugins/gazebo_ros_joint_trajectory.h>

#include "tf/tf.h"

namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosJointTrajectory::GazeboRosJointTrajectory()
{
  this->has_trajectory_ = false;
  this->trajectory_index = 0;
  this->joint_trajectory_.points.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosJointTrajectory::~GazeboRosJointTrajectory()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  // Finalize the controller
  this->rosnode_->shutdown();
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosJointTrajectory::Load( physics::WorldPtr _world, sdf::ElementPtr _sdf )
{
  // Get the world name.
  this->world_ = _world;

  //this->world_->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,0));

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("serviceName"))
  {
    // default
    this->service_name_ = "set_joint_trajectory";
  }
  else
    this->service_name_ = _sdf->GetElement("serviceName")->GetValueString();

  if (!_sdf->HasElement("topicName"))
  {
    // default
    this->topic_name_ = "set_joint_trajectory";
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_INFO("joint trajectory plugin missing <updateRate>, defaults to 0.0 (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->GetValueDouble();

  if (!ros::isInitialized())
  {
    ROS_ERROR("ros should have been initialized gazebo_ros_api_plugins, please load the server plugin.");
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  //this->tf_reference_link_name_ = tf::resolve(prefix, this->reference_link_name_);

  if (this->topic_name_ != "")
  {
    ros::SubscribeOptions trajectory_so = ros::SubscribeOptions::create<trajectory_msgs::JointTrajectory>(
      this->topic_name_,100, boost::bind( &GazeboRosJointTrajectory::SetTrajectory,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(trajectory_so);
  }

  if (this->service_name_ != "")
  {
    ros::AdvertiseServiceOptions srv_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointTrajectory>(
        this->service_name_,boost::bind(&GazeboRosJointTrajectory::SetTrajectory,this,_1,_2),
        ros::VoidPtr(), &this->queue_);
    this->srv_ = this->rosnode_->advertiseService(srv_aso);
  }
  
  this->last_time_ = this->world_->GetSimTime();

  // start custom queue for joint trajectory plugin ros topics
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosJointTrajectory::QueueThread,this ) );
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosJointTrajectory::UpdateStates, this));
}

////////////////////////////////////////////////////////////////////////////////
// set joint trajectory
void GazeboRosJointTrajectory::SetTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr& trajectory)
{
  gazebo_msgs::SetJointTrajectory::Request req;
  gazebo_msgs::SetJointTrajectory::Response res;
  // copy topic into request of service call
  req.joint_trajectory = *trajectory;
  if (!this->SetTrajectory(req,res))
    ROS_WARN("setting joint trajectory via topic returned failure");
}

bool GazeboRosJointTrajectory::SetTrajectory(const gazebo_msgs::SetJointTrajectory::Request& req, const gazebo_msgs::SetJointTrajectory::Response& res)
{
  boost::mutex::scoped_lock lock(this->update_mutex);

  this->model_pose_ = req.model_pose;
  this->set_model_pose_ = req.set_model_pose;

  this->reference_link_name_ = req.joint_trajectory.header.frame_id;
  // do this every time a new joint_trajectory is supplied, use header.frame_id as the reference_link_name_
  if (this->reference_link_name_ != "world" && this->reference_link_name_ != "/map" && this->reference_link_name_ != "map")
  {
    physics::EntityPtr ent = this->world_->GetEntity(this->reference_link_name_);
    if (ent)
      this->reference_link_ = boost::shared_dynamic_cast<physics::Link>(ent);
    if (!this->reference_link_)
    {
      ROS_ERROR("ros_joint_trajectory plugin specified a reference link [%s] that does not exist, aborting.\n",
                this->reference_link_name_.c_str());
      ROS_DEBUG("will set model [%s] configuration, keeping model root link stationary.",this->model_->GetName().c_str());
      return false;
    }
    else
      ROS_DEBUG("test: update model pose by keeping link [%s] stationary inertially",this->reference_link_->GetName().c_str());
  }

  this->model_ =  this->world_->GetModel(req.model_name);
  if (!this->model_) // look for it by frame_id name
  {
    this->model_ = this->reference_link_->GetParentModel();
    if (this->model_)
    {
      ROS_INFO("found model[%s] by link name specified in frame_id[%s]",this->model_->GetName().c_str(),req.joint_trajectory.header.frame_id.c_str());
    }
    else
    {
      ROS_WARN("no model found by link name specified in frame_id[%s],  aborting.",req.joint_trajectory.header.frame_id.c_str());
      return false;
    }
  }

  // copy joint configuration into a map
  this->joint_trajectory_ = req.joint_trajectory;

  // trajectory start time
  this->trajectory_start = gazebo::common::Time(req.joint_trajectory.header.stamp.sec,
                                                req.joint_trajectory.header.stamp.nsec);

  // update the joint_trajectory to play
  this->has_trajectory_ = true;
  // reset trajectory_index to beginning of new trajectory
  this->trajectory_index = 0;
  this->disable_physics_updates_ = req.disable_physics_updates;
  if (this->disable_physics_updates_)
  {
    this->physics_engine_enabled_ = this->world_->GetEnablePhysicsEngine();
    this->world_->EnablePhysicsEngine(false);
  }

  // create a joint with the world
  // if (this->reference_link_)
  //   this->FixLink(this->reference_link_);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// glue a link to the world by creating a fixed joint
void GazeboRosJointTrajectory::FixLink(physics::LinkPtr link)
{
  if (this->model_)
  {
    this->joint_ = this->world_->GetPhysicsEngine()->CreateJoint("revolute",this->model_);
    this->joint_->SetModel(this->model_);
    math::Pose pose = link->GetWorldPose();
    //this->joint_->Load(physics::LinkPtr(), link, pose.pos);
    this->joint_->Load(physics::LinkPtr(), link, pose);
    this->joint_->SetAxis(0, math::Vector3(0, 0, 0));
    this->joint_->SetHighStop(0, 0);
    this->joint_->SetLowStop(0, 0);
    this->joint_->SetAnchor(0, pose.pos);
    this->joint_->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// unglue a link to the world by destroying the fixed joint
void GazeboRosJointTrajectory::UnfixLink()
{
  this->joint_.reset();
}



////////////////////////////////////////////////////////////////////////////////
// Play the trajectory, update states
void GazeboRosJointTrajectory::UpdateStates()
{
  common::Time cur_time = this->world_->GetSimTime();

  // rate control
  if (this->update_rate_ > 0 && (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  {

    if (this->has_trajectory_ && this->model_)
    {
      boost::mutex::scoped_lock lock(this->update_mutex);
      // roll out trajectory via set model configuration
      if (cur_time >= this->trajectory_start)
      {
        // @todo:  consider a while loop until the trajectory "catches up" to the current time?
        if (this->trajectory_index < this->joint_trajectory_.points.size())
        {
          ROS_INFO("time [%f] updating configuration [%d/%lu]",cur_time.Double(),this->trajectory_index,this->joint_trajectory_.points.size());
          // make the service call to pause gazebo
          //bool is_paused = this->world_->IsPaused();
          //if (!is_paused) this->world_->SetPaused(true);

          // get reference link pose before updates
          math::Pose reference_pose(math::Vector3(this->model_pose_.position.x,
                                                  this->model_pose_.position.y,
                                                  this->model_pose_.position.z),
                                 math::Quaternion(this->model_pose_.orientation.w,
                                                  this->model_pose_.orientation.x,
                                                  this->model_pose_.orientation.y,
                                                  this->model_pose_.orientation.z));
          // if (this->reference_link_)
          // {
          //   reference_pose = this->reference_link_->GetWorldPose();
          // }

          // trajectory roll-out based on time:  set model configuration from trajectory message
          std::map<std::string, double> joint_position_map;
          unsigned int chain_size = this->joint_trajectory_.joint_names.size();
          if (chain_size == this->joint_trajectory_.points[this->trajectory_index].positions.size())
          {
            for (unsigned int i = 0; i < chain_size; ++i)
            {
              joint_position_map[this->joint_trajectory_.joint_names[i]] = 
                this->joint_trajectory_.points[this->trajectory_index].positions[i];
            }
            this->model_->SetJointPositions(joint_position_map);

            // set model pose?
            if (this->set_model_pose_)
            {
              this->model_->SetWorldPose(reference_pose);
            }
            // test: fix reference link location
            // if (this->reference_link_)
            //   this->model_->SetLinkWorldPose(reference_pose, this->reference_link_);
          }
          else
          {
            ROS_ERROR("point[%u] in JointTrajectory has different number of joint names[%u] and positions[%lu].",
                      this->trajectory_index, chain_size,
                      this->joint_trajectory_.points[this->trajectory_index].positions.size());
          }


          //this->world_->SetPaused(is_paused); // resume original pause-state
          gazebo::common::Time duration(this->joint_trajectory_.points[this->trajectory_index].time_from_start.sec,
                                        this->joint_trajectory_.points[this->trajectory_index].time_from_start.nsec);
          this->trajectory_start += duration; // reset start time for next trajectory point
          this->trajectory_index++; // increment to next trajectory point

          // save last update time stamp
          this->last_time_ = cur_time;
        }
        else // no more trajectory points
        {
          // trajectory finished
          this->reference_link_.reset();
          this->has_trajectory_ = false;
          if (this->disable_physics_updates_)
            this->world_->EnablePhysicsEngine(this->physics_engine_enabled_);
        }
      }
    }
  } // mutex lock

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosJointTrajectory::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosJointTrajectory);

}
