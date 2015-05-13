#include <gazebo_plugins/gazebo_ros_mimic_joint.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MimicJoint);

MimicJoint::MimicJoint()
{
  this->joint_.reset();
  this->mimic_joint_.reset();
}

MimicJoint::~MimicJoint()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
}

void MimicJoint::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();


  if (!_sdf->HasElement("jointName"))
  {
    ROS_FATAL("mimic_joint missing <jointName>, cannot proceed");
    return;
  }
  else
    this->joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();

  if (!_sdf->HasElement("mimicJoint"))
  {
    ROS_FATAL("mimic_joint missing <mimicJoint>, cannot proceed");
    return;
  }
  else
    this->mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();

  if (!_sdf->HasElement("offset"))
  {
    ROS_INFO("mimic_joint missing <offset>, set default to 0.0");
    this->offset_ = 0.0;
  }
  else
    this->offset_ = _sdf->GetElement("offset")->Get<double>();

  if (!_sdf->HasElement("multiplier"))
  {
    ROS_INFO("mimic_joint missing <multiplier>, set default to 1.0");
    this->multiplier_ = 1.0;
  }
  else
    this->multiplier_ = _sdf->GetElement("multiplier")->Get<double>();

  if (!_sdf->HasElement("pgain"))
  {
    ROS_INFO("mimic_joint missing <pgain>, set default to 1.0");
    this->p_ = 1.0;
  }
  else
    this->p_ = _sdf->GetElement("pgain")->Get<double>();


  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicJoint::UpdateChild, this));
  gzdbg << "Plugin model name: " << modelName << "\n";

  this->joint_ = model_->GetJoint(joint_name_);
  this->mimic_joint_ = model_->GetJoint(mimic_joint_name_);
}

void MimicJoint::UpdateChild()
{
  const double desired_angle = this->joint_->GetAngle(0).Radian()*this->multiplier_ + this->offset_;
  const double error = desired_angle - this->mimic_joint_->GetAngle(0).Radian();
  const double force = this->p_*error;
  this->mimic_joint_->SetForce(0, force);
  //this->mimic_joint_->SetPosition(0, this->joint_->GetAngle(0).Radian()*this->multiplier_ + this->offset_);
}
