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

/**
 *  \author Jonathan Bohren
 *  \desc   A "hand-of-god" plugin which drives a floating object around based
 *  on the location of a TF frame. This plugin is useful for connecting human input
 *  devices to "god-like" objects in a Gazebo simulation.
 */

#include <gazebo_plugins/gazebo_ros_hand_of_god.h>
#include <ros/ros.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosHandOfGod);

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosHandOfGod::GazeboRosHandOfGod() :
    ModelPlugin(),
    robot_namespace_(""),
    frame_id_("hog"),
    kl_(200),
    ka_(200)
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosHandOfGod::~GazeboRosHandOfGod()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosHandOfGod::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
  {
    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    // Get sdf parameters
    if(_sdf->HasElement("robotNamespace")) {
      this->robot_namespace_ = _sdf->Get<std::string>("robotNamespace") + "/";
    }  

    if(_sdf->HasElement("frameId")) {
      this->frame_id_ = _sdf->Get<std::string>("frameId");
    }

    if(_sdf->HasElement("kl")) {
      this->kl_ = _sdf->Get<double>("kl");
    }
    if(_sdf->HasElement("ka")) {
      this->ka_ = _sdf->Get<double>("ka");
    }

    if(_sdf->HasElement("linkName")) {
      this->link_name_ = _sdf->Get<std::string>("linkName");
    } else {
      ROS_FATAL_STREAM("The hand-of-god plugin requires a `linkName` parameter tag");
      return;
    }

    // Store the model
    model_ = _parent;

    // Disable gravity for the hog
    model_->SetGravityMode(false);

    // Get the floating link
    floating_link_ = model_->GetLink(link_name_);
    if(!floating_link_) {
      ROS_ERROR("Floating link not found!");
      const std::vector<physics::LinkPtr> &links = model_->GetLinks();
      for(unsigned i=0; i < links.size(); i++) {
        ROS_ERROR_STREAM(" -- Link "<<i<<": "<<links[i]->GetName());
      }
      return;
    }

    cl_ = 2.0 * sqrt(kl_*floating_link_->GetInertial()->GetMass());
    ca_ = 2.0 * sqrt(ka_*floating_link_->GetInertial()->GetIXX());

    // Create the TF listener for the desired position of the hog
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());

    // Register update event handler
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboRosHandOfGod::GazeboUpdate, this));
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosHandOfGod::GazeboUpdate()
  {
    // Get TF transform relative to the /world link
    geometry_msgs::TransformStamped hog_desired_tform;
    static bool errored = false;
    try{
      hog_desired_tform = tf_buffer_->lookupTransform("world", frame_id_+"_desired", ros::Time(0));
      errored = false;
    } catch (tf2::TransformException ex){
      if(!errored) {
        ROS_ERROR("%s",ex.what());
        errored = true;
      } 
      return;
    }

    // Convert TF transform to Gazebo Pose
    const geometry_msgs::Vector3 &p = hog_desired_tform.transform.translation;
    const geometry_msgs::Quaternion &q = hog_desired_tform.transform.rotation;
    gazebo::math::Pose hog_desired(
        gazebo::math::Vector3(p.x, p.y, p.z),
        gazebo::math::Quaternion(q.w, q.x, q.y, q.z));

    // Relative transform from actual to desired pose
    gazebo::math::Pose world_pose = floating_link_->GetDirtyPose();
    gazebo::math::Vector3 err_pos = hog_desired.pos - world_pose.pos;
    // Get exponential coordinates for rotation
    gazebo::math::Quaternion err_rot =  (world_pose.rot.GetAsMatrix4().Inverse() * hog_desired.rot.GetAsMatrix4()).GetRotation();
    gazebo::math::Quaternion not_a_quaternion = err_rot.GetLog();

    floating_link_->AddForce(
        kl_ * err_pos - cl_ * floating_link_->GetWorldLinearVel());

    floating_link_->AddRelativeTorque(
        ka_ * gazebo::math::Vector3(not_a_quaternion.x, not_a_quaternion.y, not_a_quaternion.z) - ca_ * floating_link_->GetRelativeAngularVel());

    // Convert actual pose to TransformStamped message
    geometry_msgs::TransformStamped hog_actual_tform;

    hog_actual_tform.header.frame_id = "world";
    hog_actual_tform.header.stamp = ros::Time::now();

    hog_actual_tform.child_frame_id = frame_id_ + "_actual";

    hog_actual_tform.transform.translation.x = world_pose.pos.x;
    hog_actual_tform.transform.translation.y = world_pose.pos.y;
    hog_actual_tform.transform.translation.z = world_pose.pos.z;

    hog_actual_tform.transform.rotation.w = world_pose.rot.w;
    hog_actual_tform.transform.rotation.x = world_pose.rot.x;
    hog_actual_tform.transform.rotation.y = world_pose.rot.y;
    hog_actual_tform.transform.rotation.z = world_pose.rot.z;
    
    tf_broadcaster_->sendTransform(hog_actual_tform);
  }

}
