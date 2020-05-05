/*
 * Copyright 2020 Open Source Robotics Foundation
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

/* Desc: A node to publish tf from gazebo model state for Gazebo
 * Author: Kei Okada
 * Date: May 5 2020
 */

// Based on:
//  https://github.com/masaeedu/me597/blob/master/lab3/src/sim_pose_publisher.cpp
//  https://github.com/SUTURO/euroc_video/blob/master/gazebo_model_state_publisher/src/gazebo_model_state_publisher_node.cpp
#include <gazebo_ros/gazebo_ros_pose_publisher.h>

tf::TransformBroadcaster *br;
std::string base_frame;
std::string tf_prefix;

// Callback function for the Position topic (SIMULATION)
void model_pose_callback(const gazebo_msgs::ModelStates &msg) {
  int i = 0;
  tf::Transform *tform;
  for (auto name : msg.name) {
    ROS_INFO_STREAM_ONCE("receved " << name << " model poses");

    tform = new tf::Transform;
    tform->setOrigin(tf::Vector3(msg.pose[i].position.x, msg.pose[i].position.y,
                                 msg.pose[i].position.z));
    tform->setRotation(
        tf::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                       msg.pose[i].orientation.z, msg.pose[i].orientation.w));
    br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), base_frame,
                                           tf_prefix + name));
    i++;
  }
}

void link_pose_callback(const gazebo_msgs::LinkStates &msg) {
  int i = 0;
  tf::Transform *tform;
  for (auto name : msg.name) {
    ROS_INFO_STREAM_ONCE("receved " << name << " link poses");

    tform = new tf::Transform;
    tform->setOrigin(tf::Vector3(msg.pose[i].position.x, msg.pose[i].position.y,
                                 msg.pose[i].position.z));
    tform->setRotation(
        tf::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                       msg.pose[i].orientation.z, msg.pose[i].orientation.w));
    br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), base_frame,
                                           tf_prefix + name));
    i++;
  }
}

int main(int argc, char **argv) {
  boost::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber model_pose_sub;
  ros::Subscriber link_pose_sub;
  bool enable_ros_network_ = true;
  bool publish_model_pose_ = true;
  bool publish_link_pose_ = false;
  int update_rate_ = 40;
  base_frame = "world";

  ros::init(argc, argv, "gazebo_ros_model_pose_publisher");
  nh_.reset(new ros::NodeHandle("~"));

  if (nh_->hasParam("enable_ros_network")) {
    nh_->getParam("enable_ros_network", enable_ros_network_);
  }
  if (nh_->hasParam("publish_model_pose")) {
    nh_->getParam("publish_model_pose", publish_model_pose_);
  }
  if (nh_->hasParam("publish_link_pose")) {
    nh_->getParam("publish_link_pose", publish_link_pose_);
  }
  if (nh_->hasParam("update_rate")) {
    nh_->getParam("update_rate", update_rate_);
  }
  if (nh_->hasParam("base_frame")) {
    nh_->getParam("base_frame", base_frame);
  }
  if (nh_->hasParam("tf_prefix")) {
    nh_->getParam("tf_prefix", tf_prefix);
  }

  if (!enable_ros_network_) {
    ROS_INFO_NAMED("api_plugin", "ROS gazebo topics/services are disabled");
    return -1;
  }

  // setup transform broadcaster
  br = new tf::TransformBroadcaster;

  // Subscribe to the desired topics and assign callbacks
  if (publish_model_pose_) {
    ROS_INFO_STREAM("publish model poses wrt. " << base_frame << " frame and "
                                                << tf_prefix << " TF prefix");
    model_pose_sub =
        nh_->subscribe("/gazebo/model_states", 1, model_pose_callback);
  }
  if (publish_link_pose_) {
    ROS_INFO_STREAM("publish link poses wrt. " << base_frame << " frame and "
                                               << tf_prefix << " TF prefix");
    link_pose_sub =
        nh_->subscribe("/gazebo/link_states", 1, link_pose_callback);
  }

  ros::Rate loop_rate(update_rate_); // 40Hz update rate

  while (ros::ok()) {
    loop_rate.sleep(); // Maintain the loop rate
    ros::spinOnce();   // Check for new messages
  }

  return 0;
}
