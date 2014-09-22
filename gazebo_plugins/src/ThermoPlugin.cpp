#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/subscribe_options.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo {
class ThermoPlugin: public ModelPlugin {

public:
	// Initialize
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		// Store the pointer to the model
		this->model = _parent;

		// read option args in sdf tags
		this->robot_name = "robot";
		if (_sdf->HasElement("robotname")) {
			this->link_name = _sdf->Get<std::string>("robotname");
		}
		this->link_name = "root";
		if (_sdf->HasElement("linkname")) {
			this->link_name = _sdf->Get<std::string>("linkname");
		}
		this->joint_name = "";
		if (_sdf->HasElement("jointname")) {
			this->joint_name = _sdf->Get<std::string>("jointname");
		}
		this->electric_resistance = this->parseThermoParam(_sdf, "electric_resistance", 1.16);
		this->inner_thermo_resistance = this->parseThermoParam(_sdf, "inner_thermo_resistance", 1.93);
		this->coil_thermo_conductance = this->parseThermoParam(_sdf, "coil_thermo_conductance", 21.55);
		this->outer_thermo_resistance = this->parseThermoParam(_sdf, "outer_thermo_resistance", 4.65);
		this->case_thermo_conductance = this->parseThermoParam(_sdf, "case_thermo_conductance", 240.86);
		this->atomosphere_thermo_conductance = this->parseThermoParam(_sdf, "atomosphere_thermo_conductance", 1000.0);
		this->A_vs_Nm = this->parseThermoParam(_sdf, "A_vs_Nm", 0.1);
		this->atomosphere_temperature = this->parseThermoParam(_sdf, "atomosphere_temperature", 300);
		this->case_temperature = this->parseThermoParam(_sdf, "case_temperature", 300);
		this->coil_temperature = this->parseThermoParam(_sdf, "coil_temperature", 300);
		this->thermal_calcuration_step = this->parseThermoParam(_sdf, "thermal_calcuration_step", 10);
		this->thermal_calcuration_cnt = this->thermal_calcuration_step ;
		this->tau = 0;

		// find root link
		this->joint = this->model->GetJoint(this->joint_name);
		this->link = this->model->GetLink(this->link_name);
		if (!this->link) { this->link = this->joint->GetChild(); }
		this->world = this->model->GetWorld();
		this->lastUpdateTime = this->world->GetSimTime() ;

		// Make sure the ROS node for Gazebo has already been initialized
		if (!ros::isInitialized()) {
			gzerr
					<< "A ROS node for Gazebo has not been initialized, unable to load plugin. "
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)";
			return;
		}
		// ros node
		this->rosNode = new ros::NodeHandle("");
		// ros callback queue for processing subscription
		this->deferredLoadThread = boost::thread(
				boost::bind(&ThermoPlugin::DeferredLoad, this));
	}

	void DeferredLoad() {
		// publish multi queue
		this->pmq.startServiceThread();

		// ros topic publications
		this->pubTorqueQueue = this->pmq.addPub<std_msgs::Float32>();
		this->pubTorque = this->rosNode->advertise<std_msgs::Float32>(
				 "/" + this->robot_name + "/thermo_plugin/" + this->joint_name + "/torque", 100, true);
		this->pubCoilThermoQueue = this->pmq.addPub<std_msgs::Float32>();
		this->pubCoilThermo = this->rosNode->advertise<std_msgs::Float32>(
				"/" + this->robot_name + "/thermo_plugin/" + this->joint_name + "/thermo/coil", 100, true);
		this->pubCaseThermoQueue = this->pmq.addPub<std_msgs::Float32>();
			this->pubCaseThermo = this->rosNode->advertise<std_msgs::Float32>(
					"/" + this->robot_name + "/thermo_plugin/" + this->joint_name + "/thermo/case", 100, true);


		// Listen to the update event.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&ThermoPlugin::OnUpdate, this, _1));

		gzmsg << "ThermoPlugin was loaded !" << std::endl;
	}

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
		physics::JointPtr j = this->joint ;
		physics::JointWrench w = j->GetForceTorque(0u);
		math::Vector3 a = j->GetGlobalAxis(0u);
		math::Vector3 m = this->link->GetWorldPose().rot * w.body2Torque;
		this->tau += (float)a.Dot(m);

		if ( --this->thermal_calcuration_cnt > 0 ) return ;

		this->tau /= this->thermal_calcuration_step;
		this->thermal_calcuration_cnt = this->thermal_calcuration_step;
		common::Time curTime = this->world->GetSimTime();
		this->PublishThermo(curTime, this->lastUpdateTime);
		this->lastUpdateTime = curTime ;
		this->tau = 0;
	}

	// Link has only 1 joint, and the joint has only 1 axis
	void PublishThermo(const common::Time &_curTime, const common::Time &_lastTime) {
		std_msgs::Float32 tor, the1, the2 ;
		float abs_tau  ;
		if ( this->tau > 0 ){
			abs_tau = this->tau ;
		} else {
			abs_tau = -this->tau ;
		}

		float dt = (_curTime - _lastTime).Float();
		float dT = (this->atomosphere_temperature - this->case_temperature)/this->outer_thermo_resistance + (this->coil_temperature - this->case_temperature)/this->inner_thermo_resistance;
		float dTin = (this->case_temperature - this->coil_temperature)/this->inner_thermo_resistance + abs_tau * this->A_vs_Nm * this->electric_resistance;
		float dTout = (this->case_temperature - this->atomosphere_temperature)/this->outer_thermo_resistance;
		this->case_temperature += dT / this->case_thermo_conductance * dt;
		this->coil_temperature += dTin / this->coil_thermo_conductance * dt;
		this->atomosphere_temperature += dTout / this->atomosphere_thermo_conductance * dt;

		tor.data = this->tau ;
		this->pubTorqueQueue->push(tor, this->pubTorque);
		the1.data = this->case_temperature;
		this->pubCaseThermoQueue->push(the1, this->pubCaseThermo);
		the2.data = this->coil_temperature;
		this->pubCoilThermoQueue->push(the2, this->pubCoilThermo);

		//std::cout << "(" << this->coil_temperature << "," << this->case_temperature << "," << this->atomosphere_temperature << ") in " << dt << "sec" << std::endl;
	}

private:
	physics::ModelPtr model;
	physics::WorldPtr world;
	std::string robot_name;
	std::string joint_name;
	std::string link_name;
	physics::LinkPtr link;
	physics::JointPtr joint;
	event::ConnectionPtr updateConnection;
	common::Time lastUpdateTime;
	int thermal_calcuration_step, thermal_calcuration_cnt;
	float tau;

	float electric_resistance, inner_thermo_resistance, outer_thermo_resistance;
	float coil_thermo_conductance, case_thermo_conductance, atomosphere_thermo_conductance;
	float A_vs_Nm, atomosphere_temperature, case_temperature, coil_temperature;

	ros::NodeHandle* rosNode;
	PubMultiQueue pmq;
	boost::mutex mutex;
	boost::thread deferredLoadThread;

	ros::Publisher pubTorque;
	PubQueue<std_msgs::Float32>::Ptr pubTorqueQueue;
	ros::Publisher pubCoilThermo;
	PubQueue<std_msgs::Float32>::Ptr pubCoilThermoQueue;
	ros::Publisher pubCaseThermo;
	PubQueue<std_msgs::Float32>::Ptr pubCaseThermoQueue;

//	ros::Publisher pubMoment;
//	PubQueue<geometry_msgs::Vector3>::Ptr pubMomentQueue;

	float parseThermoParam(sdf::ElementPtr _sdf, std::string name, float defo){
		float ret ;
		if (_sdf->HasElement(name)) {
			ret = _sdf->Get<float>(name);
		} else {
			ret = defo;
		}
		std::cout << " [thermo plugin] " << name << " = " << ret << " for " << this->joint_name << std::endl;
		return ret ;
	}
};

GZ_REGISTER_MODEL_PLUGIN(ThermoPlugin)
}

