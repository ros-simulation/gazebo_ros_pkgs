
#include <boost/bind.hpp>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/common.hh>

#include <controller_manager/controller_manager.h>
#include <ros_control_gazebo/robot_sim.h>

namespace ros_control_gazebo_plugin
{   
  
  class RosControlGazeboPlugin : public gazebo::ModelPlugin
  {
    public: 
      virtual ~RosControlGazeboPlugin() {
        ros::shutdown();
      }

      // Overloaded Gazebo entry point
      void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) 
      {
        // Store the pointer to the model
        model_ = parent;

        // Initialize ROS interface, if necessary
        if(!ros::isInitialized()) 
        {
          // Note: If we don't disable the SIGINT handler, gazebo will not exit
          int node_argc = 0;
          char **node_argv = NULL;
          ros::init(node_argc, node_argv, "gazebo",
                    ros::init_options::NoSigintHandler);

          // Start a spinner thread for handling callbacks
          spinner_.reset(new ros::AsyncSpinner(1));
          spinner_->start();
        }

        // Get namespace for nodehandle
        std::string ns = std::string("ros_control/")+model_->GetName();
        if(sdf->HasElement("ns")) {
          ns = sdf->GetElement("ns")->GetValueString();
        }

        // Get the robot simulation interface type
        if(sdf->HasElement("robotSimType")) {
          robot_sim_type_str_ = sdf->GetValueString("robotSimType");
        } else {
          ROS_FATAL_STREAM("RobotSim sub-class type not found in URDF/SDF");
          return;
        }

        // Get the controller period
        if(sdf->HasElement("controlPeriod")) {
          control_period_ = ros::Duration(sdf->GetValueDouble("controlPeriod"));
          // Get the simulation period
          ros::Duration sim_period
            (model_->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod());
          
          // Check the period against the simulation period
          if( control_period_ < sim_period ) {
            ROS_ERROR_STREAM("Desired controller update period ("<<control_period_
                <<" s) is faster than the gazebo simulation period ("
                <<sim_period<<" s).");
          } else if( control_period_ > sim_period ) {
            ROS_WARN_STREAM("Desired controller update period ("<<control_period_
                <<" s) is slower than the gazebo simulation period ("
                <<sim_period<<" s).");
          }
        } else {
          ROS_FATAL_STREAM("Control period not found in URDF/SDF");
          return;
        }

        // Get the ROS paarameters
        model_nh_ = ros::NodeHandle(ns);

        // Load the RobotSim abstraction to interface the controllers with the
        // gazebo model
        try {
          robot_sim_loader_.reset
            (new pluginlib::ClassLoader<ros_control_gazebo::RobotSim>
             ("ros_control_gazebo",
              "ros_control_gazebo::RobotSim"));
          robot_sim_ = robot_sim_loader_->createInstance(robot_sim_type_str_);
          if(!robot_sim_->initSim(model_nh_, model_)) {
            ROS_FATAL("Could not initialize robot simulation interface");
            return;
          }

          // Create the controller manager
          controller_manager_.reset
            (new controller_manager::ControllerManager(robot_sim_.get(), model_nh_));

          // Listen to the update event. This event is broadcast every
          // simulation iteration.
          update_connection_ = 
            gazebo::event::Events::ConnectWorldUpdateStart
            (boost::bind(&RosControlGazeboPlugin::Update, this));
        } catch(pluginlib::LibraryLoadException &ex) {
          ROS_FATAL_STREAM("Failed to create robot simulation interface loader: "<<ex.what());
        }
      }

      // Called by the world update start event
      void Update()
      {
        // Get the simulation time and period
        gazebo::common::Time gz_time_now = model_->GetWorld()->GetSimTime();
        ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
        ros::Duration sim_period = sim_time_ros - last_sim_time_ros_;

        // Check if we should update the controllers
        if(sim_period >= control_period_) {
          // Store this simulation time
          last_sim_time_ros_ = sim_time_ros;

          // Update the robot simulation with the state of the gazebo model
          robot_sim_->readSim(sim_time_ros, sim_period);

          // Compute the controller commands
          controller_manager_->update(sim_time_ros, sim_period);

          // Update the gazebo model with the result of the controller
          // computation
          robot_sim_->writeSim(sim_time_ros, sim_period);
        }
      }

    private:
      // Namespace
      ros::NodeHandle model_nh_;
      boost::shared_ptr<ros::AsyncSpinner> spinner_;

      // Pointer to the model
      gazebo::physics::ModelPtr model_;

      // Pointer to the update event connection
      gazebo::event::ConnectionPtr update_connection_;

      // Interface loader
      boost::shared_ptr<pluginlib::ClassLoader<ros_control_gazebo::RobotSim> >
        robot_sim_loader_;
      void load_robot_sim_srv();
      
      // Robot simulator interface
      std::string robot_sim_type_str_;
      boost::shared_ptr<ros_control_gazebo::RobotSim> robot_sim_;

      // Controller manager
      boost::shared_ptr<controller_manager::ControllerManager>
        controller_manager_;

      // Timing
      ros::Duration control_period_;
      ros::Time last_sim_time_ros_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RosControlGazeboPlugin);
}
