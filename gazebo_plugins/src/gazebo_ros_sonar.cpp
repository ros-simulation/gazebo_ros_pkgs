#include <algorithm>
#include <string>
#include <assert.h>

#include <iostream>
#include <fstream>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/SonarSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gazebo_plugins/gazebo_ros_sonar.hpp"

namespace gazebo_plugins
{
  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosSonar::GazeboRosSonar()
  {
    this->seed = 0;

    // Write a file to disk, including includes
    // TODO: remove this
    std::ofstream myfile;
    myfile.open("/tmp/sonar.txt");
    myfile << "Sonar plugin loaded" << std::endl;
    myfile.close();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosSonar::~GazeboRosSonar()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosSonar::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // load plugin
    SonarPlugin::Load(_parent, this->sdf);
    gzwarn << "ROS2 SonarPlugin Loaded" << std::endl;

    // Get the world name.
    std::string worldName = _parent->WorldName();
    this->world_ = gazebo::physics::get_world(worldName);

    // save pointers
    this->sdf = _sdf;

    this->last_update_time_ = gazebo::common::Time(0);

    this->parent_sonar_sensor_ =
        std::dynamic_pointer_cast<gazebo::sensors::SonarSensor>(_parent);

    if (!this->parent_sonar_sensor_)
      gzthrow("GazeboRosSonar controller requires a Sonar Sensor as its parent");

    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Sonar");

    // Parse the model.sdf file (html) //////////////////////////////////////////
    if (!this->sdf->HasElement("frameName"))
    {
      RCLCPP_INFO(rclcpp::get_logger("range"), "Range plugin missing <frameName>, defaults to /world");
      this->frame_name_ = "/world";
    }
    else
      this->frame_name_ = this->sdf->Get<std::string>("frameName");

    RCLCPP_INFO(rclcpp::get_logger("sonar"),
                "#########    frame name : %s  #############", this->frame_name_.c_str());

    if (!this->sdf->HasElement("topicName"))
    {
      RCLCPP_INFO(rclcpp::get_logger("sonar"), "Sonar plugin missing <topicName>, defaults to /sonar");
      this->topic_name_ = "/sonar";
    }
    else
      this->topic_name_ = this->sdf->Get<std::string>("topicName");

    if (!this->sdf->HasElement("fov"))
    {
      RCLCPP_WARN(rclcpp::get_logger("sonar"), "Sonar plugin missing <fov>, defaults to 0.05");
      this->fov_ = 0.05;
    }
    else
      this->fov_ = _sdf->GetElement("fov")->Get<double>();

    if (!this->sdf->HasElement("gaussianNoise"))
    {
      RCLCPP_WARN(rclcpp::get_logger("sonar"), "Sonar plugin missing <gaussianNoise>, defaults to 0.0");
      this->gaussian_noise_ = 0;
    }
    else
      this->gaussian_noise_ = this->sdf->Get<double>("gaussianNoise");

    //////////////////////////////////////////////////////////////////////////////

    this->sonar_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
    // TODO: why does this not work -> https://github.com/ros2/rclcpp/pull/522
    if (!rclcpp::ok())
    {
      RCLCPP_FATAL_STREAM(
          rclcpp::get_logger("sonar"),
          "A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("sonar"),
                "Starting Sonar Plugin (ns = %s)", this->robot_namespace_.c_str());
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
        boost::bind(&GazeboRosSonar::LoadThread, this));
  }

  /**
   * The LoadThread method is called by the newly created thread and is responsible
   * for creating the ROS publisher that will send Sonar measurements to the ROS network.
   * The method uses the pmq (ros::CallbackQueue) to process the messages published by Gazebo.
   * The Sonar measurements are represented by sensor_msgs::Range messages, which are published
   * to the ROS topic defined by the topicName parameter in the SDF file.
   */
  void GazeboRosSonar::LoadThread()
  {
    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = gazebo_ros::Node::Get(this->sdf);
    // this->rosnode_ = rclcpp::Node::make_shared(this->robot_namespace_);

    this->tf_prefix_ = this->robot_namespace_;
    RCLCPP_INFO(this->rosnode_->get_logger(),
                "Sonar Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
                this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

    // resolve tf prefix
    this->frame_name_ = this->tf_prefix_ + "/" + this->frame_name_;

    if (this->topic_name_ != "")
    {

      rclcpp::PublisherOptions options;
      // FIXME: modify using API: https://docs.ros2.org/galactic/api/rclcpp/structrclcpp_1_1PublisherEventCallbacks.html
      // options.callback_group = this->get_callback_group()->get_ptr();
      // options.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo & info)
      // { };
      // options.event_callbacks.connected = [this]()
      // { SonarConnect(); };
      // options.event_callbacks.disconnected = [this]()
      // { SonarDisconnect(); };

      this->pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::Range>(this->topic_name_, 1, options);
      this->pub_queue_ = this->pmq.addPub<sensor_msgs::msg::Range>();

      // TODO: it seems like ROS2 doesn't support the functionality of callbacks
      // when a subscriber connects/disconnects to a publisher, so we publish regardless
      SonarConnect();

    } else {
      RCLCPP_WARN(
        this->rosnode_->get_logger(), "Sonar plugin missing <topicName>. Not publishing to ROS.");
    }

    // sensor generation off by default
    this->parent_sonar_sensor_->SetActive(false);
  }

  void GazeboRosSonar::SonarConnect()
  {
    this->sonar_connect_count_++;
    // this->parent_sonar_sensor_->SetActive(true);
    if (this->sonar_connect_count_ == 1)
      this->sonar_sub_ =
          this->gazebo_node_->Subscribe(this->parent_sonar_sensor_->Topic(),
                                        &GazeboRosSonar::OnScan, this);
  }

  void GazeboRosSonar::SonarDisconnect()
  {
    this->sonar_connect_count_--;
    if (this->sonar_connect_count_ == 0)
      this->sonar_sub_.reset();
  }

  /**
   * Forward the Gazebo Sonar measurement to the ROS topic.
   *
   * TODO: can we override the parent's OnUpdate method instead?
   *       see https://github.com/arpg/Gazebo/blob/master/plugins/SonarPlugin.hh
   */
  void GazeboRosSonar::OnScan(ConstSonarStampedPtr &_msg)
  {
    sensor_msgs::msg::Range range_msg_;
    range_msg_.header.stamp = rclcpp::Time(_msg->time().sec(), _msg->time().nsec());
    range_msg_.header.frame_id = this->frame_name_;

    range_msg_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

    range_msg_.field_of_view = fov_;
    range_msg_.max_range = this->parent_sonar_sensor_->RangeMax();
    range_msg_.min_range = this->parent_sonar_sensor_->RangeMin();

    range_msg_.range = _msg->sonar().range();
    if (range_msg_.range < range_msg_.max_range)
      range_msg_.range = std::min(range_msg_.range + this->GaussianKernel(0, gaussian_noise_), parent_sonar_sensor_->RangeMax());

    this->pub_queue_->push(range_msg_, *this->pub_);
  }

  /**
   * Utility for adding noise
   */
  double GazeboRosSonar::GaussianKernel(double mu, double sigma)
  {
    // using Box-Muller transform to generate two independent standard
    // normally disbributed normal variables see wikipedia

    // normalized uniform random variable
    double U = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    // normalized uniform random variable
    double V = static_cast<double>(rand_r(&this->seed)) /
               static_cast<double>(RAND_MAX);

    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

    // there are 2 indep. vars, we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
  }

  // extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin * RegisterPlugin();
  // gazebo::SensorPlugin * RegisterPlugin()
  // {
  //   return (gazebo::SonarPlugin *)(new GazeboRosSonar());
  // }

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)
}
