#include <ros/ros.h>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo_msgs/SpawnModel.h>

#include <gtest/gtest.h>
#include <iostream>

#include <tf/tf.h>

double curX_;
double curY_;
double curHeading_;
bool vehicleSpawned_;

class RawGazeboFixture : public gazebo::ServerFixture
{
    public: virtual void SetUp()
    {}

    public: virtual void TearDown()
    {}
};

std::string get_std_string()
{
  std::ostringstream stream;
  stream
    << "<?xml version='1.0' ?>"
    << "<sdf version='1.5'>"
    <<   "<world name='default'>"
    <<     "<model name='test_model'>"
    <<       "<pose>0.0 -1.0 1.0 0 0 0</pose>"
    <<       "<static>true</static>"
    <<     "</model>"
    <<   "</world>"
    << "</sdf>";
  return stream.str();
}

TEST (RawGazeboFixture, SpawnSDFTest)
{ 
  ros::NodeHandle nh;
  ros::ServiceClient modelSpawnSrv = 
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

  gazebo_msgs::SpawnModelRequest  model_req;
  gazebo_msgs::SpawnModelResponse model_res;
  gazebo_msgs::SpawnModel model_srv;

  model_req.model_name      = "test_model";
  model_req.model_xml       = get_std_string();
  model_req.robot_namespace = "";
  model_req.reference_frame = "world";
  
  model_srv.request  = model_req;
  model_srv.response = model_res;

  modelSpawnSrv.waitForExistence();
  modelSpawnSrv.call(model_srv);
  
  // check msgs return value
  EXPECT_TRUE(model_srv.response.success);
  this->HasEntity(model_req.model_name);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "spawn_delete-test");
  ros::Time::init();

  return RUN_ALL_TESTS();
}
