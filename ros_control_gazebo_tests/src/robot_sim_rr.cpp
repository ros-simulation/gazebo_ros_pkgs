#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros_control_gazebo/robot_sim.h>

#include <angles/angles.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace ros_control_gazebo_tests {

  class RobotSimRR : public ros_control_gazebo::RobotSim
  {
  public:
    RobotSimRR() :
      n_dof_(2),
      joint_name_(n_dof_),
      joint_position_(n_dof_),
      joint_velocity_(n_dof_),
      joint_effort_(n_dof_),
      joint_effort_command_(n_dof_),
      joint_velocity_command_(n_dof_)
    {

      joint_name_[0] = "joint1";
      joint_name_[1] = "joint2";

      for(unsigned int j=0; j < n_dof_; j++) 
      {
        joint_position_[j] = 1.0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 0.1;
        joint_effort_command_[j] = 0.0;
        joint_velocity_command_[j] = 0.0;

        // Register joints
        js_interface_.registerJoint(joint_name_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]);
        ej_interface_.registerJoint(js_interface_.getJointStateHandle(joint_name_[j]), &joint_effort_command_[j]);
        vj_interface_.registerJoint(js_interface_.getJointStateHandle(joint_name_[j]), &joint_velocity_command_[j]);
      }

      // Register interfaces
      registerInterface(&js_interface_);
      registerInterface(&ej_interface_);
      registerInterface(&vj_interface_);
    }

    bool initSim(ros::NodeHandle nh, gazebo::physics::ModelPtr model)
    {
      // Get the gazebo joints that correspond to the robot joints
      for(unsigned int j=0; j < n_dof_; j++) 
      {
        ROS_INFO_STREAM("Getting pointer to gazebo joint: "<<joint_name_[j]);
        gazebo::physics::JointPtr joint = model->GetJoint(joint_name_[j]);
        if (joint) 
        {
          sim_joints_.push_back(joint);
        } 
        else 
        {
          ROS_ERROR_STREAM("This robot has a joint named \""<<joint_name_[j]<<"\" which is not in the gazebo model.");
          return false;
        }
      }

      return true;
    }

    void readSim(ros::Time time, ros::Duration period)
    {
      for(unsigned int j=0; j < n_dof_; j++) 
      {
        // Gazebo has an interesting API...
        joint_position_[j] += angles::shortest_angular_distance
          (joint_position_[j], sim_joints_[j]->GetAngle(0).Radian());
        joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
        joint_effort_[j] = sim_joints_[j]->GetForce(0u);
      }
    }

    void writeSim(ros::Time time, ros::Duration period) 
    {
      for(unsigned int j=0; j < n_dof_; j++) 
      {
        // Gazebo has an interesting API...
        sim_joints_[j]->SetForce(0,joint_effort_command_[j]);
      }
    }

  private:
    unsigned int n_dof_;

    hardware_interface::JointStateInterface    js_interface_;
    hardware_interface::EffortJointInterface   ej_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

    std::vector<std::string> joint_name_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_velocity_command_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;
  };

}

PLUGINLIB_DECLARE_CLASS(ros_control_gazebo_tests, RobotSimRR, ros_control_gazebo_tests::RobotSimRR, ros_control_gazebo::RobotSim)
