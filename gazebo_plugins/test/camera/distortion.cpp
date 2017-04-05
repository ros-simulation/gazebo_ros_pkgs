#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

class DistortionTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
  }

  ros::NodeHandle nh_;
  image_transport::Subscriber cam_sub_;
  bool has_new_image_;
  ros::Time image_stamp_;
public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    image_stamp_ = msg->header.stamp;
    has_new_image_ = true;
  }
};


TEST_F(DistortionTest, cameraDistortionTest)
{
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe("camera1/image_raw", 1,
                          &DistortionTest::imageCallback,
                          dynamic_cast<DistortionTest*>(this));

  while (!has_new_image_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  //FIXME: change this
  EXPECT_LT(2, 3);
  cam_sub_.shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_camera_distortion_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
