
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros_control_gazebo/robot_sim.h>

namespace ros_control_gazebo_plugin
{   
  class RosControlGazeboPlugin : public gazebo::ModelPlugin
  {
    public: 
      
      void Load(gazebo::physics::ModelPtr parent, gazebo::sdf::ElementPtr /*sdf*/) 
      {
        // Store the pointer to the model
        model_ = parent;
        
        // Get the ROS paarameters
        model_nh_ = ros::NodeHandle("ros_control/"+model_->getName());

        // Get the full class type for the robot sim interface
        if(!model_nh_.getParam("robot_sim_type", robot_sim_type_str_)) {
          ROS_FATAL_STREAM("RobotSim type ROS parameter not found at
                           "<<model_nh_.getNamespace()<<"/robot_sim_type");
          return;
        }

        // Get the simulation period
        ros::Duration sim_period(model_->GetWorld()->GetPhysicsEngine()->GetUpdatePeriod());

        if(!model_nh_.getParam("control_period", control_period_)) {
          ROS_FATAL_STREAM("Controller update period ROS parameter not found at
                           "<<model_nh_.getNamespace()<<"/control_period");
          return;
        } else if( control_period_ < sim_period ) {
          ROS_ERROR_STREAM("Desired controller update period ("<<control_period_
                           <<" s) is faster than the gazebo simulation period ("
                           <<sim_period<<" s).");
        } else if( control_period > sim_period ) {
          ROS_WARN_STREAM("Desired controller update period ("<<control_period_
                           <<" s) is slower than the gazebo simulation period ("
                           <<sim_period<<" s).");
        }

        // Load the RobotSim abstraction to interface the controllers with the gazebo model
        robot_sim_loader_.reset
          (new pluginlib::ClassLoader<ros_control_gazebo::RobotSim>
           ("ros_control_gazebo",
            "ros_control_gazebo::RobotSim"));
        robot_sim_.reset(robot_sim_loader_->createInstance(robot_sim_type_str_));

        // Create the controller manager
        controller_manager_.reset(new ros_control::ControllerManager(robot_sim_, model_nh_));

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        update_connection_ = event::Events::ConnectWorldUpdateStart
          (boost::bind(&RosControlGazeboPlugin::update, this));
      }

      // Called by the world update start event
      void update()
      {
        // Get the simulation time and period
        gazebo::common::Time sim_time_gz = model_->GetWorld()->GetSimTime();
        ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
        ros::Duration sim_period = sim_time_ros - last_sim_time_ros_;

        // Check if we should update the controllers
        if(sim_period >= control_period_) {
          // Store this simulation time
          last_sim_time_ros_ = sim_time_ros;

          // Update the robot simulation with the state of the gazebo model
          robot_sim_->read_state(model_);

          // Compute the controller commands
          controller_manager_.update(sim_time_ros, sim_period);

          // Update the gazebo model with the result of the controller computation
          robot_hw_sim_->write_state(model_);
        }
      }

    private:
      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      // Interface loader
      boost::shared_ptr<pluginlib::ClassLoader<ros_control_gazebo::RobotSim> > robot_sim_loader_;
      void load_robot_sim_srv();
      
      // Robot simulator interface
      std::string robot_sim_type_str_;
      boost::shared_ptr<ros_control_gazebo::RobotSim> robot_sim_;

      // Controller manager
      boost::shared_ptr<ros_control::ControllerManager> controller_manager_;

      // Timing
      ros::Duration control_period_;
      ros::Time last_sim_time_ros_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RosControlGazeboPlugin);
}
