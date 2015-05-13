#ifndef GAZEBO_ROS_MIMIC_JOINT_HH
#define GAZEBO_ROS_MIMIC_JOINT_HH

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ros/ros.h>

namespace gazebo
{
/** \defgroup MimicJoint XML Reference and Example

\brief Ros Gazebo Ros MimicJoint Plugin.
This model allow to mimic a joint without the use of the dynamics

Example Usage:

\verbatim
<gazebo>
  <plugin name="mimic_joint" filename="libgazebo_ros_mimic_plugin.so">
    <jointName>joint_name</jointName>
    <mimicJoint>mimic_joint_name</mimicJoint>
    <multiplier>1.0</multiplier>
    <offset>0.0</offset>
    <pgain>1.0</pgain>
  </plugin>
</gazebo>
\endverbatim
\{
*/


/// \brief GazeboRosFT controller
/// This is a controller that simulates a 6 dof force sensor
class MimicJoint: public ModelPlugin
{
  /// \brief Constructor
  /// \param parent The parent entity must be a Model
  public: MimicJoint();

  /// \brief Destructor
  public: virtual ~MimicJoint();

  /// \brief Load the controller
  public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

  /// \brief Update the controller
  protected: virtual void UpdateChild();
  
  /// \brief multiplier
  private: double multiplier_;

  /// \brief offset
  private: double offset_;

  /// \brief proportionnal gain
  private: double p_;

  /// \brief A pointer to the Gazebo joint
  private: physics::JointPtr joint_;
  
  /// \brief A pointer to the Gazebo joint
  private: physics::JointPtr mimic_joint_;
  
  /// \brief A pointer to the Gazebo model
  private: physics::ModelPtr model_;
  
  /// \brief A pointer to the Gazebo world
  private: physics::WorldPtr world_;

  /// \brief store bodyname
  private: std::string joint_name_;

  /// \brief store name of mimic joint
  private: std::string mimic_joint_name_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;

};

/** \} */
/// @}


}

#endif
