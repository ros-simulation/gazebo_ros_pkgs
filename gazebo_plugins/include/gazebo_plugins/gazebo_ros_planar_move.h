/*
 * Copyright 2013 Open Source Robotics Foundation
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
 * Desc: Simple model controller that uses a twist message to move a robot on
 *       the xy plane.
 * Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

#ifndef GAZEBO_ROS_PLANAR_MOVE_HH
#define GAZEBO_ROS_PLANAR_MOVE_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class GazeboRosPlanarMove : public ModelPlugin {

    public: 
      GazeboRosPlanarMove();
      ~GazeboRosPlanarMove();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double y_;
      double rot_;
      bool alive_;
      double recover_roll_velocity_p_gain_;
      double recover_pitch_velocity_p_gain_;
      double recover_z_velocity_p_gain_;
      double x_velocity_limit_max_;
      double x_velocity_limit_min_;
      double y_velocity_limit_max_;
      double y_velocity_limit_min_;
      double rot_velocity_limit_max_;
      double rot_velocity_limit_min_;
      double x_acceleration_limit_max_;
      double x_acceleration_limit_min_;
      double y_acceleration_limit_max_;
      double y_acceleration_limit_min_;
      double rot_acceleration_limit_max_;
      double rot_acceleration_limit_min_;
      double joint_state_idel_sec_;

      common::Time last_time_;
      common::Time last_cmd_subscribe_time_;
      common::Time last_odom_publish_time_;
      math::Pose last_odom_pose_;
      math::Vector3 linear_cmd_;
      math::Vector3 angular_cmd_;
      math::Vector3 last_linear_cmd_;
      math::Vector3 last_angular_cmd_;

     //
     gazebo::physics::RayShapePtr rayShape_;
     //
     void GetNearestEntityBelow(physics::EntityPtr _fromEntity,
                                double &_distBelow,
                                std::string &_entityName)
     {
       gazebo::physics::EntityPtr entityBelow = _fromEntity;
       math::Vector3 start = _fromEntity->GetWorldPose().pos;
       math::Vector3 end = start;
       while ( entityBelow && ( entityBelow->GetParentModel() == _fromEntity->GetParentModel() ) ) {
         end.z -= 1000;
         rayShape_->SetPoints(start, end); // Set the ray based on starting and ending points relative to the body.
         rayShape_->GetIntersection(_distBelow, _entityName);
         entityBelow = parent_->GetWorld()->GetEntity(_entityName);
         start.z -= (_distBelow + 0.00001);
       }
       _distBelow += 0.00001;
       //std::cerr << " +_entityName " << _entityName << ", dist = " << _distBelow << ", start " << start.z << ", end " << end.z << std::endl;
    }

  };

}

#endif /* end of include guard: GAZEBO_ROS_PLANAR_MOVE_HH */
