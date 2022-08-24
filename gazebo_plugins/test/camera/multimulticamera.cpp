#include <gtest/gtest.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class MultiMultiCameraTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
  }

  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> cam1_sub_;
  message_filters::Subscriber<sensor_msgs::Image> cam2_sub_;
  message_filters::Subscriber<sensor_msgs::Image> cam3_sub_;

  bool has_new_image_;
  ros::Time image1_stamp_;
  ros::Time image2_stamp_;
  ros::Time image3_stamp_;
  std::string frame1_, frame2_, frame3_;

public:
  void imageCallback(
      const sensor_msgs::ImageConstPtr& msg1,
      const sensor_msgs::ImageConstPtr& msg2,
      const sensor_msgs::ImageConstPtr& msg3)
  {
    image1_stamp_ = msg1->header.stamp;
    image2_stamp_ = msg2->header.stamp;
    image3_stamp_ = msg3->header.stamp;
    frame1_ = msg1->header.frame_id;
    frame2_ = msg2->header.frame_id;
    frame3_ = msg3->header.frame_id;
    has_new_image_ = true;
  }
};

// Test if the camera image is published at all, and that the timestamp
// is not too long in the past.
TEST_F(MultiMultiCameraTest, cameraSubscribeTest)
{
  // image_transport::ImageTransport it(nh_);
  // cam_left_sub_.subscribe(it, "stereo/camera/left/image_raw", 1);
  // cam_right_sub_.subscribe(it, "stereo/camera/right/image_raw", 1);
  // sync_ = message_filters::Synchronizer<MySyncPolicy>(
  //     MySyncPolicy(4), cam_left_sub_, cam_right_sub_);
  cam1_sub_.subscribe(nh_, "multi/camera_1/image_raw", 1);
  cam2_sub_.subscribe(nh_, "multi/camera_2/image_raw", 1);
  cam3_sub_.subscribe(nh_, "multi/camera_3/image_raw", 1);
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(
      cam1_sub_, cam2_sub_, cam3_sub_, 4);
  sync.registerCallback(boost::bind(&MultiMultiCameraTest::imageCallback,
      dynamic_cast<MultiMultiCameraTest*>(this), _1, _2, _3));
#if 0
  // wait for gazebo to start publishing
  // TODO(lucasw) this isn't really necessary since this test
  // is purely passive
  bool wait_for_topic = true;
  while (wait_for_topic)
  {
    // @todo this fails without the additional 0.5 second sleep after the
    // publisher comes online, which means on a slower or more heavily
    // loaded system it may take longer than 0.5 seconds, and the test
    // would hang until the timeout is reached and fail.
    if (cam_sub_.getNumPublishers() > 0)
       wait_for_topic = false;
    ros::Duration(0.5).sleep();
  }
#endif

  while (!has_new_image_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  double sync_diff1 = (image1_stamp_ - image2_stamp_).toSec();
  double sync_diff2 = (image2_stamp_ - image3_stamp_).toSec();
  EXPECT_EQ(sync_diff1, 0.0);
  EXPECT_EQ(sync_diff2, 0.0);

  EXPECT_EQ("cam1", frame1_);
  EXPECT_EQ("cam2", frame2_);
  EXPECT_EQ("cam3", frame3_);

  // This check depends on the update period being much longer
  // than the expected difference between now and the received image time
  // TODO(lucasw)
  // this likely isn't that robust - what if the testing system is really slow?
  double time_diff = (ros::Time::now() - image1_stamp_).toSec();
  ROS_INFO_STREAM(time_diff);
  EXPECT_LT(time_diff, 1.0);
  // cam_sub_.shutdown();

  // make sure nothing is subscribing to image_trigger topic
  // there is no easy API, so call getSystemState
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = ros::this_node::getName();
  EXPECT_TRUE(ros::master::execute("getSystemState", args, result, payload, true));
  // [publishers, subscribers, services]
  // subscribers in index 1 of payload
  for (int i = 0; i < payload[1].size(); ++i)
  {
    // [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
    // topic name i is in index 0
    std::string topic = payload[1][i][0];
    EXPECT_EQ(topic.find("image_trigger"), std::string::npos);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_multimulticamera_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
