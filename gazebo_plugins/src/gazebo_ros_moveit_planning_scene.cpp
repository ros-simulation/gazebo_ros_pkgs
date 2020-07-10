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
 * Desc: A plugin which publishes the gazebo world state as a MoveIt! planning scene
 * Author: Jonathan Bohren
 * Date: 15 May 2014
 */

#include <algorithm>
#include <assert.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/common/common.hh>

#include <gazebo_plugins/gazebo_ros_moveit_planning_scene.h>

namespace gazebo
{

static std::string get_id(const physics::LinkPtr &link) {
  return link->GetModel()->GetName() + "." + link->GetName();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMoveItPlanningScene);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosMoveItPlanningScene::GazeboRosMoveItPlanningScene()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMoveItPlanningScene::~GazeboRosMoveItPlanningScene()
{
#if GAZEBO_MAJOR_VERSION >= 8
#else
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
#endif

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMoveItPlanningScene::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  this->model_name_ = _model->GetName();

  // load parameters
  if (_sdf->HasElement("robotNamespace")) {
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  } else {
    this->robot_namespace_ = "";
  }

  if (!_sdf->HasElement("robotName")) {
    this->robot_name_ = _model->GetName();
  } else {
    this->robot_name_ = _sdf->GetElement("robotName")->Get<std::string>();
  }

  if (!_sdf->HasElement("topicName")) {
    this->topic_name_ = "planning_scene";
  }  else {
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("sceneName")) {
    this->scene_name_ = "";
  } else {
    this->scene_name_ = _sdf->GetElement("sceneName")->Get<std::string>();
  }

  if (!_sdf->HasElement("updatePeriod")) {
    this->publish_period_ = ros::Duration(0.0);
  } else {
    this->publish_period_ = ros::Duration(_sdf->GetElement("updatePeriod")->Get<double>());
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_.reset(new ros::NodeHandle(this->robot_namespace_));

  last_publish_time_ = ros::Time(0,0);

  planning_scene_pub_ = this->rosnode_->advertise<moveit_msgs::PlanningScene>(
      this->topic_name_, 1,
      boost::bind(&GazeboRosMoveItPlanningScene::subscriber_connected, this));

  // Custom Callback Queue for service
  this->callback_queue_thread_ = boost::thread(
      boost::bind(&GazeboRosMoveItPlanningScene::QueueThread, this));

  // Create service server
  ros::AdvertiseServiceOptions aso;
  boost::function<bool(std_srvs::Empty::Request&, std_srvs::Empty::Response&)> srv_cb =
    boost::bind(&GazeboRosMoveItPlanningScene::PublishPlanningSceneCB, this, _1, _2);
  aso.init("publish_planning_scene", srv_cb);
  aso.callback_queue = &this->queue_;

  publish_planning_scene_service_ = this->rosnode_->advertiseService(aso);

  // Publish the full scene on the next update
  publish_full_scene_ = true;

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosMoveItPlanningScene::UpdateCB, this));
}

void GazeboRosMoveItPlanningScene::subscriber_connected()
{
  boost::mutex::scoped_lock lock(this->mutex_);
  publish_full_scene_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Manually publish the full planninc scene
bool GazeboRosMoveItPlanningScene::PublishPlanningSceneCB(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& resp)
{
  boost::mutex::scoped_lock lock(this->mutex_);

  // Set flag to re-publish full scene
  publish_full_scene_ = true;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMoveItPlanningScene::UpdateCB()
{
  boost::mutex::scoped_lock lock(this->mutex_);

  using namespace gazebo::common;
  using namespace gazebo::physics;

  if(!publish_full_scene_) {
    if(publish_period_.isZero() || (ros::Time::now() - last_publish_time_) < publish_period_) {
      return;
    } else {
      last_publish_time_ = ros::Time::now();
    }
  }

  // Iterate through the tracked models and clear their dynamic information
  // This also sets objects to be removed if they currently aren't in the scene
  for(std::map<std::string,moveit_msgs::CollisionObject>::iterator object_it = collision_object_map_.begin();
      object_it != collision_object_map_.end();
      ++object_it)
  {
    // Convenience reference
    moveit_msgs::CollisionObject &object = object_it->second;

    // Mark for removal unless it's found in the gazebo world
    object.operation = moveit_msgs::CollisionObject::REMOVE;

    // Clear shape information
    object.primitives.clear();
    object.primitive_poses.clear();

    object.meshes.clear();
    object.mesh_poses.clear();

    object.planes.clear();
    object.plane_poses.clear();
  }

  // Iterate over all the models currently in the world
#if GAZEBO_MAJOR_VERSION >= 8
  std::vector<ModelPtr> models = this->world_->Models();
#else
  std::vector<ModelPtr> models = this->world_->GetModels();
#endif
  for(std::vector<ModelPtr>::const_iterator model_it = models.begin();
      model_it != models.end();
      ++model_it)
  {
    const ModelPtr &model = *model_it;
    const std::string model_name = model->GetName();

    // Don't declare collision objects for the robot links
    if(model_name == model_name_) {
      continue;
    }

    // Iterate over all links in the model, and add collision objects from each one
    // This adds meshes and primitives to:
    //  object.meshes,
    //  object.primitives, and
    //  object.planes
    std::vector<LinkPtr> links = model->GetLinks();
    for(std::vector<LinkPtr>::const_iterator link_it = links.begin();
        link_it != links.end();
        ++link_it)
    {
      const LinkPtr &link = *link_it;
      const std::string id = get_id(link);

      // Check if the gazebo model is already in the collision object map
      std::map<std::string, moveit_msgs::CollisionObject>::iterator found_collision_object =
        collision_object_map_.find(id);

      // Create a new collision object representing this link if it's not in the map
      if(found_collision_object == collision_object_map_.end() || publish_full_scene_){

        ROS_INFO("Creating collision object for model %s, this=%s, objects=%u",
                 model_name.c_str(),
                 model_name_.c_str(),
                 (unsigned int)collision_object_map_.size());

        moveit_msgs::CollisionObject new_object;
        new_object.id = id;
        new_object.header.frame_id = "world";
        new_object.operation = moveit_msgs::CollisionObject::ADD;

        collision_object_map_[id] = new_object;
        ROS_DEBUG_STREAM_NAMED("GazeboRosMoveItPlanningScene","Adding object: "<<id);
      } else {
        collision_object_map_[id].operation = moveit_msgs::CollisionObject::MOVE;
        ROS_DEBUG_STREAM_NAMED("GazeboRosMoveItPlanningScene","Moving object: "<<id);
      }

      // Get a reference to the object from the map
      moveit_msgs::CollisionObject &object = collision_object_map_[id];

#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d link_pose = link->WorldPose();
#else
      ignition::math::Pose3d link_pose = link->GetWorldPose().Ign();
#endif
      geometry_msgs::Pose link_pose_msg;
      link_pose_msg.position.x = link_pose.Pos().X();
      link_pose_msg.position.y = link_pose.Pos().Y();
      link_pose_msg.position.z = link_pose.Pos().Z();
      link_pose_msg.orientation.x = link_pose.Rot().X();
      link_pose_msg.orientation.y = link_pose.Rot().Y();
      link_pose_msg.orientation.z = link_pose.Rot().Z();
      link_pose_msg.orientation.w = link_pose.Rot().W();
      //ROS_DEBUG_STREAM_NAMED("GazeboRosMoveItPlanningScene",model_name << " (link): " <<link_pose_msg);

      // Get all the collision objects for this link
      std::vector<CollisionPtr> collisions = link->GetCollisions();
      for(std::vector<CollisionPtr>::const_iterator coll_it = collisions.begin();
          coll_it != collisions.end();
          ++coll_it)
      {
        const CollisionPtr &collision = *coll_it;
        const ShapePtr shape = collision->GetShape();

        // NOTE: In gazebo 2.2.2 Collision::GetWorldPose() does not work
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d collision_pose = collision->InitialRelativePose() + link_pose;
#else 
        ignition::math::Pose3d collision_pose = collision->GetInitialRelativePose().Ign() + link_pose;
#endif
        geometry_msgs::Pose collision_pose_msg;
        collision_pose_msg.position.x = collision_pose.Pos().X();
        collision_pose_msg.position.y = collision_pose.Pos().Y();
        collision_pose_msg.position.z = collision_pose.Pos().Z();
        collision_pose_msg.orientation.x = collision_pose.Rot().X();
        collision_pose_msg.orientation.y = collision_pose.Rot().Y();
        collision_pose_msg.orientation.z = collision_pose.Rot().Z();
        collision_pose_msg.orientation.w = collision_pose.Rot().W();
        //ROS_DEBUG_STREAM_NAMED("GazeboRosMoveItPlanningScene",model_name << " (collision): " <<collision_pose_msg);

        // Always add pose information
        if(shape->HasType(Base::MESH_SHAPE)) {
          //object.mesh_poses.push_back(collision_pose_msg);
          boost::shared_ptr<MeshShape> mesh_shape = boost::dynamic_pointer_cast<MeshShape>(shape);
          std::string uri = mesh_shape->GetMeshURI();
          const Mesh *mesh = MeshManager::Instance()->GetMesh(uri);
          if(!mesh) {
              gzwarn << "Shape has mesh type but mesh could not be retried from the MeshManager. Loading ad-hoc!" << std::endl;
              gzwarn << " mesh uri: " <<uri<< std::endl;

              // Load the mesh ad-hoc if the manager doesn't have it
              // this happens with model:// uris
              mesh = MeshManager::Instance()->Load(uri);

              if(!mesh) {
                gzwarn << "Mesh could not be loded: " << uri << std::endl;
                continue;
              }
            } 

          // Iterate over submeshes
          unsigned n_submeshes = mesh->GetSubMeshCount();

          for(unsigned m=0; m < n_submeshes; m++) {

            const SubMesh *submesh = mesh->GetSubMesh(m);
            unsigned n_vertices = submesh->GetVertexCount();

            switch(submesh->GetPrimitiveType()) 
            {
              case SubMesh::POINTS:
              case SubMesh::LINES:
              case SubMesh::LINESTRIPS:
                // These aren't supported
                break;
              case SubMesh::TRIANGLES:
              case SubMesh::TRISTRIPS:
                object.mesh_poses.push_back(collision_pose_msg);
                break;
              case SubMesh::TRIFANS:
                // Unsupported
                gzerr << "TRIFANS not supported yet." << std::endl;
                break;
            };
          }
        } else if(shape->HasType(Base::PLANE_SHAPE)) {
          object.plane_poses.push_back(collision_pose_msg);
        } else { //if (!shape->HasType(Base::MESH_SHAPE)) {
          object.primitive_poses.push_back(collision_pose_msg);
        }

        // Only add geometric info if we haven't published it before
        if(object.operation == moveit_msgs::CollisionObject::ADD || object.operation == moveit_msgs::CollisionObject::APPEND)
        {
          if(shape->HasType(Base::MESH_SHAPE))
          {
            // Get the mesh structure from the mesh shape
            boost::shared_ptr<MeshShape> mesh_shape = boost::dynamic_pointer_cast<MeshShape>(shape);
            std::string name = mesh_shape->GetName();
            std::string uri = mesh_shape->GetMeshURI();
#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Vector3d scale = mesh_shape->Scale();
#else
            ignition::math::Vector3d scale = mesh_shape->GetScale().Ign();
#endif
            const Mesh *mesh = MeshManager::Instance()->GetMesh(uri);

            gzwarn << " mesh scale: " <<scale<< std::endl;
            if(!mesh) {
              gzwarn << "Shape has mesh type but mesh could not be retried from the MeshManager. Loading ad-hoc!" << std::endl;
              gzwarn << " mesh uri: " <<uri<< std::endl;

              // Load the mesh ad-hoc if the manager doesn't have it
              // this happens with model:// uris
              mesh = MeshManager::Instance()->Load(uri);

              if(!mesh) {
                gzwarn << "Mesh could not be loded: " << uri << std::endl;
                continue;
              }
            }

            // Iterate over submeshes
            unsigned n_submeshes = mesh->GetSubMeshCount();

            for(unsigned m=0; m < n_submeshes; m++) {

              const SubMesh *submesh = mesh->GetSubMesh(m);
              unsigned n_vertices = submesh->GetVertexCount();

              switch(submesh->GetPrimitiveType())
              {
                case SubMesh::POINTS:
                case SubMesh::LINES:
                case SubMesh::LINESTRIPS:
                  // These aren't supported
                  break;
                case SubMesh::TRIANGLES:
                  {
                    //gzwarn<<"TRIANGLES"<<std::endl;
                    shape_msgs::Mesh mesh_msg;
                    mesh_msg.vertices.resize(n_vertices);
                    mesh_msg.triangles.resize(n_vertices/3);

                    for(size_t v=0; v < n_vertices; v++) {

                      const int index = submesh->GetIndex(v);
                      const ignition::math::Vector3d vertex = submesh->Vertex(v);

                      mesh_msg.vertices[index].x = vertex.X() * scale.X();
                      mesh_msg.vertices[index].y = vertex.Y() * scale.Y();
                      mesh_msg.vertices[index].z = vertex.Z() * scale.Z();

                      mesh_msg.triangles[v/3].vertex_indices[v%3] = index;
                    }

                    object.meshes.push_back(mesh_msg);
                    //object.mesh_poses.push_back(collision_pose_msg);
                    break;
                  }
                case SubMesh::TRISTRIPS:
                  {
                    //gzwarn<<"TRISTRIPS"<<std::endl;
                    shape_msgs::Mesh mesh_msg;
                    mesh_msg.vertices.resize(n_vertices);
                    mesh_msg.triangles.resize(n_vertices-2);

                    for(size_t v=0; v < n_vertices; v++) {
                      const int index = submesh->GetIndex(v);
                      const ignition::math::Vector3d vertex = submesh->Vertex(v);

                      mesh_msg.vertices[index].x = vertex.X() * scale.X();
                      mesh_msg.vertices[index].y = vertex.Y() * scale.Y();
                      mesh_msg.vertices[index].z = vertex.Z() * scale.Z();

                      if(v < n_vertices-2) mesh_msg.triangles[v].vertex_indices[0] = index;
                      if(v > 0 && v < n_vertices-1) mesh_msg.triangles[v-1].vertex_indices[1] = index;
                      if(v > 1) mesh_msg.triangles[v-2].vertex_indices[2] = index;
                    }

                    object.meshes.push_back(mesh_msg);
                    //object.mesh_poses.push_back(collision_pose_msg);
                    break;
                  }
                case SubMesh::TRIFANS:
                  // Unsupported
                  gzerr << "TRIFANS not supported yet." << std::endl;
                  break;
              };
            }
          }
          else if(shape->HasType(Base::PLANE_SHAPE))
          {
            // Plane
            boost::shared_ptr<PlaneShape> plane_shape = boost::dynamic_pointer_cast<PlaneShape>(shape);
            shape_msgs::Plane plane_msg;

#if GAZEBO_MAJOR_VERSION >= 8
            plane_msg.coef[0] = plane_shape->Normal().X();
            plane_msg.coef[1] = plane_shape->Normal().Y();
            plane_msg.coef[2] = plane_shape->Normal().Z();
#else
            plane_msg.coef[0] = plane_shape->GetNormal().Ign().X();
            plane_msg.coef[1] = plane_shape->GetNormal().Ign().Y();
            plane_msg.coef[2] = plane_shape->GetNormal().Ign().Z();
#endif
            plane_msg.coef[3] = 0; // This should be handled by the position of the collision object

            object.planes.push_back(plane_msg);
          }
          else
          {
            // Solid primitive
            shape_msgs::SolidPrimitive primitive_msg;

            if(shape->HasType(Base::BOX_SHAPE)) {

              boost::shared_ptr<BoxShape> box_shape = boost::dynamic_pointer_cast<BoxShape>(shape);

              primitive_msg.type = primitive_msg.BOX;
              primitive_msg.dimensions.resize(3);
#if GAZEBO_MAJOR_VERSION >= 8
              primitive_msg.dimensions[0] = box_shape->Size().X();
              primitive_msg.dimensions[1] = box_shape->Size().Y();
              primitive_msg.dimensions[2] = box_shape->Size().Z();
#else
              primitive_msg.dimensions[0] = box_shape->GetSize().Ign().X();
              primitive_msg.dimensions[1] = box_shape->GetSize().Ign().Y();
              primitive_msg.dimensions[2] = box_shape->GetSize().Ign().Z();
#endif
            } else if(shape->HasType(Base::CYLINDER_SHAPE)) {

              boost::shared_ptr<CylinderShape> cylinder_shape = boost::dynamic_pointer_cast<CylinderShape>(shape);

              primitive_msg.type = primitive_msg.CYLINDER;
              primitive_msg.dimensions.resize(2);
              primitive_msg.dimensions[0] = cylinder_shape->GetLength();
              primitive_msg.dimensions[1] = cylinder_shape->GetRadius();

            } else if(shape->HasType(Base::SPHERE_SHAPE)) {

              boost::shared_ptr<SphereShape> sphere_shape = boost::dynamic_pointer_cast<SphereShape>(shape);

              primitive_msg.type = primitive_msg.SPHERE;
              primitive_msg.dimensions.resize(1);
              primitive_msg.dimensions[0] = sphere_shape->GetRadius();

            } else {
              // HEIGHTMAP_SHAPE, MAP_SHAPE, MULTIRAY_SHAPE, RAY_SHAPE
              // Unsupported
              continue;
            }

            object.primitives.push_back(primitive_msg);
          }
          ROS_INFO("model %s has %zu links",model_name.c_str(),links.size());
          ROS_INFO("model %s has %zu meshes, %zu mesh poses",
                   model_name.c_str(),
                   object.meshes.size(),
                   object.mesh_poses.size());
        }
      }
    }
  }

  // Clear the list of collision objects
  planning_scene_msg_.name = scene_name_;
  planning_scene_msg_.robot_model_name = robot_name_;
  planning_scene_msg_.is_diff = true;
  planning_scene_msg_.world.collision_objects.clear();

  // Iterate through the objects we're already tracking
  for(std::map<std::string,moveit_msgs::CollisionObject>::iterator object_it = collision_object_map_.begin();
      object_it != collision_object_map_.end();)
  {
    // Update stamp
    object_it->second.header.stamp = ros::Time::now();
    // Add the object to the planning scene message
    planning_scene_msg_.world.collision_objects.push_back(object_it->second);

    // Actually remove objects from being tracked
    if(object_it->second.operation == moveit_msgs::CollisionObject::REMOVE) {
      // Actually remove the collision object from the map
      ROS_DEBUG_STREAM_NAMED("GazeboRosMoveItPlanningScene","Removing object: "<<object_it->second.id);
      collision_object_map_.erase(object_it++);
    } else {
      ++object_it;
    }

  }

  planning_scene_pub_.publish(planning_scene_msg_);

  // no more full scene
  publish_full_scene_ = false;
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosMoveItPlanningScene::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
