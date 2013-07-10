/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
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

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

// Boost
#include <boost/bind.hpp>

// ROS
#include <ros/ros.h>

#include <pluginlib/class_loader.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/common.hh>

// ros_control
#include <controller_manager/controller_manager.h>
#include <gazebo_ros_control/robot_hw_sim.h>

namespace gazebo_ros_control
{

class RosControlPlugin : public gazebo::ModelPlugin
{
public:

  virtual ~RosControlPlugin()
  {
    // Disconnect from gazebo events
    gazebo::event::Events::DisconnectWorldUpdateBegin(update_connection_);
  }

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO_STREAM("Loading ros_control_gazebo_plugin...");

    // Save pointers to the model
    parent_model_ = parent;
    sdf_ = sdf;

    // ros callback queue for processing subscription
    deferred_load_thread_ = boost::thread(
      boost::bind(&RosControlPlugin::loadThread, this));
  }

  // Load in seperate thread from Gazebo in case ROS is blocking
  void loadThread()
  {
    // Error message if the model couldn't be found
    if (!parent_model_)
    {
      ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
      return;
    }

    // Initialize ROS interface, if necessary (if we aren't using gazebo_ros_api_plugin)
    if(!ros::isInitialized())
    {
      ROS_ERROR_STREAM("Not loading plugin since ROS hasn't been "
        << "properly initialized.  Try starting gazebo with ros plugin:\n"
        << "  gazebo -s libgazebo_ros_api_plugin.so\n");

      return;
    }

    // Debug output
    /*
      ROS_DEBUG_STREAM_NAMED("ros_control","Plugin XML:");
      std::string temp;
      sdf_->PrintValues(temp);
      std::cout << temp;
    */

    // Get namespace for nodehandle
    // TODO: "topicNamespace" or "rosNamespace"
    if(sdf_->HasElement("robotNamespace")) {
      robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
    } else {
      // TODO: Why not make this default?
      robot_namespace_ = parent_model_->GetName(); // default
    }

    // Get robot_description ROS param name
    if (sdf_->HasElement("robotParam")) {
      robot_description_ = sdf_->GetElement("robotParam")->Get<std::string>();
    } else {
      robot_description_ = "robot_description"; // default
    }

    // Prefix robot description with namespace
    //robot_description_ = robot_namespace_ + "/" + robot_description_;
    //ROS_DEBUG_STREAM_NAMED("loadThread","Using rosparam parameter \""<<robot_description_<<"\" for robot model description.");

    // Get the robot simulation interface type
    if(sdf_->HasElement("robotSimType")) {
      robot_hw_sim_type_str_ = sdf_->Get<std::string>("robotSimType");
    } else {
      robot_hw_sim_type_str_ = "gazebo_ros_control/DefaultRobotHWSim";
      ROS_DEBUG_STREAM_NAMED("loadThread","Using default plugin for RobotHWSim (none specified in URDF/SDF)\""<<robot_hw_sim_type_str_<<"\"");
    }

    // Get the Gazebo simulation period
    ros::Duration sim_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod());

    // Decide the plugin control period
    if(sdf_->HasElement("controlPeriod"))
    {
      control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

      // Check the period against the simulation period
      if( control_period_ < sim_period )
      {
        ROS_ERROR_STREAM("Desired controller update period ("<<control_period_
          <<" s) is faster than the gazebo simulation period ("<<sim_period<<" s).");
      }
      else if( control_period_ > sim_period )
      {
        ROS_WARN_STREAM("Desired controller update period ("<<control_period_
          <<" s) is slower than the gazebo simulation period ("<<sim_period<<" s).");
      }
    }
    else
    {
      control_period_ = sim_period;
      ROS_DEBUG_STREAM("Control period not found in URDF/SDF, defaulting to Gazebo period of " 
        << control_period_);
    }


    /*
    // Get the joints this plugin is supposed to control
    if(sdf_->HasElement("joint"))
    {
    // get all available joints
    sdf::ElementPtr element_it = sdf_->GetElement("joint");

    while(element_it != sdf::ElementPtr() ) // do while not null
    {
    ROS_DEBUG_STREAM_NAMED("load","Parsed from plugin SDF joint w/name '"
    << element_it->Get<std::string>() << "'");

    // Add joint to vector
    joints_.push_back( element_it->Get<std::string>() );

    element_it = element_it->GetNextElement("joint");
    }
    }
    */

    // Get parameters/settings for controllers from ROS param server
    model_nh_ = ros::NodeHandle(robot_namespace_);
    nh_ = ros::NodeHandle();
    ROS_INFO("Starting gazebo_ros_control plugin in namespace: %s", robot_namespace_.c_str());

    // Read urdf from ros parameter server then
    // setup actuators and mechanism control node.
    // This call will block if ROS is not properly initialized.
    if (!loadControllerManagerFromURDF())
    {
      ROS_ERROR("Error parsing URDF in gazebo_ros_control plugin, plugin not active.\n");
      return;
    }

    // Load the RobotHWSim abstraction to interface the controllers with the
    // gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim>
          ("gazebo_ros_control",
            "gazebo_ros_control::RobotHWSim"));

      robot_hw_sim_ = robot_hw_sim_loader_->createInstance(robot_hw_sim_type_str_);

      if(!robot_hw_sim_->initSim(model_nh_, parent_model_, joints_))
      {
        ROS_FATAL("Could not initialize robot simulation interface");
        return;
      }

      // Create the controller manager
      ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
      controller_manager_.reset
        (new controller_manager::ControllerManager(robot_hw_sim_.get(), model_nh_));

      // Listen to the update event. This event is broadcast every simulation iteration.
      update_connection_ =
        gazebo::event::Events::ConnectWorldUpdateBegin
        (boost::bind(&RosControlPlugin::Update, this));

    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM("Failed to create robot simulation interface loader: "<<ex.what());
    }

    ROS_INFO("Loaded gazebo_ros_control.");
  }

  // Called by the world update start event
  void Update()
  {
    // Get the simulation time and period
    gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
    ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
    ros::Duration sim_period = sim_time_ros - last_sim_time_ros_;

    // Check if we should update the controllers
    if(sim_period >= control_period_) {
      // Store this simulation time
      last_sim_time_ros_ = sim_time_ros;

      // Update the robot simulation with the state of the gazebo model
      robot_hw_sim_->readSim(sim_time_ros, sim_period);

      // Compute the controller commands
      controller_manager_->update(sim_time_ros, sim_period);

      // Update the gazebo model with the result of the controller
      // computation
      robot_hw_sim_->writeSim(sim_time_ros, sim_period);
    }
  }

  // Get the URDF XML from the parameter server
  std::string getURDF(std::string param_name) const
  {
    std::string urdf_string;

    // search and wait for robot_description on param server
    while (urdf_string.empty())
    {
      std::string search_param_name;
      if (nh_.searchParam(param_name, search_param_name))
      {
        ROS_INFO_ONCE("gazebo_ros_control plugin is waiting for model"
          " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

        nh_.getParam(search_param_name, urdf_string);
      }
      else
      {
        ROS_INFO("gazebo_ros_control plugin is waiting for model"
          " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

        nh_.getParam(param_name, urdf_string);
      }

      usleep(100000);
    }
    ROS_DEBUG_STREAM_NAMED("gazebo_ros_control", "Recieved urdf from param server, parsing...");

    return urdf_string;
  }

  // Get Transmissions from the URDF
  bool loadControllerManagerFromURDF()
  {
    std::string urdf_string = getURDF(robot_description_);

    // initialize TiXmlDocument doc with a string
    TiXmlDocument doc;
    if (!doc.Parse(urdf_string.c_str()) && doc.Error())
    {
      ROS_ERROR("Could not load the gazebo_ros_control plugin's"
        " configuration file: %s\n", urdf_string.c_str());
      return false;
    }

    // debug
    //doc.Print();
    //std::cout << *(doc.RootElement()) << std::endl;

    /*
    // Pulls out the list of actuators used in the robot configuration.
    struct GetActuators : public TiXmlVisitor
    {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
    if (elt.ValueStr() == "actuator" && elt.Attribute("name"))
    actuators.insert(elt.Attribute("name"));
    return true;
    }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    // Places that found actuators into the hardware interface.
    std::set<std::string>::iterator it;
    for (it = get_actuators.actuators.begin();
    it != get_actuators.actuators.end(); ++it)
    {
    std::cout << " adding actuator " << (*it) << std::endl;
    //pr2_hardware_interface::Actuator* pr2_actuator =
    //  new pr2_hardware_interface::Actuator(*it);
    //pr2_actuator->state_.is_enabled_ = true;
    //hardware_interface_.addActuator(pr2_actuator);
    }
    */

    // Find joints in transmission tags
    TiXmlElement *root = doc.RootElement();

    // Constructs the transmissions by parsing custom xml.
    TiXmlElement *tran_it = NULL;
    for (tran_it = root->FirstChildElement("transmission"); tran_it;
         tran_it = tran_it->NextSiblingElement("transmission"))
    {
      // Load transmission attributes
      std::string tran_type;
      if(tran_it->Attribute("type"))
      {
        tran_type = tran_it->Attribute("type");
        // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Found tranmission of type '" << tran_type << "'");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No type attribute for tranmission tag!");
        continue;
      }

      std::string tran_name;
      if(tran_it->Attribute("name"))
      {
        tran_name = tran_it->Attribute("name");
        // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Found tranmission of name '" << tran_name << "'");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No name attribute for tranmission tag!");
        continue;
      }

      // Load joint attributes
      TiXmlElement *joint_child = tran_it->FirstChildElement("joint");
      if(!joint_child)
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No joint element found for tranmission "
          << tran_name);
        continue;
      }

      std::string joint_name;
      if(joint_child->Attribute("name"))
      {
        joint_name = joint_child->Attribute("name");
        // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Found joint of name '" << joint_name << "'");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No name attribute for joint in tranmission "
          << tran_name);
        continue;
      }

      // Load actuator attributes
      TiXmlElement *actuator_child = tran_it->FirstChildElement("actuator");
      if(!actuator_child)
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No actuator element found for tranmission "
          << tran_name);
        continue;
      }

      std::string actuator_name;
      if(actuator_child->Attribute("name"))
      {
        actuator_name = actuator_child->Attribute("name");
        // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Found actuator of name '" << actuator_name << "'");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No name attribute for actuator in tranmission "
          << tran_name);
        continue;
      }

      std::string actuator_type;
      if(actuator_child->Attribute("type"))
      {
        actuator_type = actuator_child->Attribute("type");
        // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Found actuator of type '" << actuator_type << "'");
      }
      else
      {
        ROS_WARN_STREAM_NAMED("ros_control_plugin","No type attribute for actuator "
          << actuator_name);
        continue;
      }

      // \todo read mechanical reduction tag

      // \todo load transmission

      // Copy data into joint struct
      JointData joint_data(joint_name, actuator_type);
      joints_.push_back(joint_data);

    } // end for transmission elements

    return true;
  }

private:

  // Node Handles
  ros::NodeHandle nh_; // no namespace
  ros::NodeHandle model_nh_; // namespaces to robot name

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;

  // deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<gazebo_ros_control::RobotHWSim> > robot_hw_sim_loader_;
  void load_robot_hw_sim_srv();

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Joints in this plugin's scope
  std::vector<JointData> joints_;

  // Robot simulator interface
  std::string robot_hw_sim_type_str_;
  boost::shared_ptr<gazebo_ros_control::RobotHWSim> robot_hw_sim_;

  // Controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_sim_time_ros_;

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RosControlPlugin);
}
