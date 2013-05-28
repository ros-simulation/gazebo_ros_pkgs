/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <boost/bind.hpp>
#include "physics/physics.hh"
#include "transport/Node.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"

namespace gazebo
{
  class MoveModelTest : public ModelPlugin
  {
    public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
      // Get then name of the parent model
      //std::string modelName = _sdf->GetParent()->GetValueString("name");

      // Get the world name.
      this->world = _parent->GetWorld();

      // Get a pointer to the model
      //this->model = this->world->GetModel(modelName);
      this->model = _parent;

      // Error message if the model couldn't be found
      if (!this->model)
        gzerr << "Unable to get parent model\n";

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateStart(
          boost::bind(&MoveModelTest::OnUpdate, this));
      gzdbg << "plugin model name: " << this->model->GetName() << "\n";


      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->GetName());
      this->statsSub = this->node->Subscribe("~/world_stats", &MoveModelTest::OnStats, this);

    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // do something on update
      this->simTime  = this->world->GetSimTime();

      math::Pose orig_pose = this->model->GetWorldPose();

      math::Pose new_pose = orig_pose;
      //new_pose.pos.x = -1.0 + 0.5*sin(0.01*this->simTime.Double());
      new_pose.rot.SetFromEuler(math::Vector3(0,0,this->simTime.Double()));

      //if (this->simTime.Double() > 20.0)
        this->model->SetWorldPose( new_pose );

      gzdbg << "plugin simTime [" << this->simTime.Double() << "] update new_pose [" << new_pose << "] orig pose [" << orig_pose << "]\n";
    }

    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
    {
      this->simTime  = msgs::Convert( _msg->sim_time() );

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
    private: physics::WorldPtr world;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveModelTest)
}
