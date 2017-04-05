// Core includes
#include <fstream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// Test includes
#include <gtest/gtest.h>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

// Camera calibration is adapted from
// http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
using namespace cv;

class Settings {
 public:
  Settings() : goodInput(false) {}
  enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
  enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

  void write(FileStorage& fs) const {                      //Write serialization for this class
    fs << "{" << "BoardSize_Width"  << boardSize.width
       << "BoardSize_Height" << boardSize.height
       << "Square_Size"         << squareSize
       << "Calibrate_Pattern" << patternToUse
       << "Calibrate_FixAspectRatio" << aspectRatio
       << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
       << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

       << "Show_UndistortedImage" << showUndistorted

       << "Input_FlipAroundHorizontalAxis" << flipVertical
       << "Input_Delay" << delay
       << "}";
  }
  void read(const FileNode& node) {                        //Read serialization for this class
    node["BoardSize_Width" ] >> boardSize.width;
    node["BoardSize_Height"] >> boardSize.height;
    node["Calibrate_Pattern"] >> patternToUse;
    node["Square_Size"]  >> squareSize;
    node["Calibrate_FixAspectRatio"] >> aspectRatio;
    node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
    node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
    node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
    node["Show_UndistortedImage"] >> showUndistorted;
    node["Input_Delay"] >> delay;
    interprate();
  }
  void interprate() {
    goodInput = true;
    if (boardSize.width <= 0 || boardSize.height <= 0) {
      std::cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << std::endl;
      goodInput = false;
    }
    if (squareSize <= 10e-6) {
      std::cerr << "Invalid square size " << squareSize << std::endl;
      goodInput = false;
    }

    flag = 0;
    if (calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
    if (aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


    calibrationPattern = NOT_EXISTING;
    if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
    if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
    if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
    if (calibrationPattern == NOT_EXISTING) {
      std::cerr << " Inexistent camera calibration mode: " << patternToUse << std::endl;
      goodInput = false;
    }

  }
 public:
  Size boardSize;            // The size of the board -> Number of items by width and height
  Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
  float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
  float aspectRatio;         // The aspect ratio
  int delay;                 // In case of a video input
  bool calibZeroTangentDist; // Assume zero tangential distortion
  bool calibFixPrincipalPoint;// Fix the principal point at the center
  bool flipVertical;          // Flip the captured images around the horizontal axis
  bool showUndistorted;       // Show undistorted images after calibration
  std::string input;               // The input ->

  int cameraID;
  bool goodInput;
  int flag;

 private:
  std::string patternToUse;


};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings()) {
  if (node.empty())
    x = default_value;
  else
    x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool tryCalibration(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                    std::vector<std::vector<Point2f> > imagePoints );

std::vector<double> run(std::string inputSettingsFile, std::vector<Mat> images) {
  std::vector<double> coeffs;

  Settings s;
  FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
  if (!fs.isOpened()) {
    std::cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << std::endl;
    return coeffs;
  }
  fs["Settings"] >> s;
  fs.release();                                         // close Settings file

  if (!s.goodInput) {
    std::cout << "Invalid input detected. Application stopping. " << std::endl;
    return coeffs;
  }

  std::vector<std::vector<Point2f> > imagePoints;
  Mat cameraMatrix, distCoeffs;
  Size imageSize;
  int mode = CAPTURING;
  clock_t prevTimestamp = 0;
  const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
  const char ESC_KEY = 27;

  int nextImage = 0;

  for(auto it = images.begin(); it != images.end(); ++it)
  {
    Mat view = *it;

    imageSize = view.size();  // Format input image.
    if ( s.flipVertical )    flip( view, view, 0 );

    std::vector<Point2f> pointBuf;

    bool found;
    switch ( s.calibrationPattern ) { // Find feature points on the input format
    case Settings::CHESSBOARD:
      found = findChessboardCorners( view, s.boardSize, pointBuf,
                                     CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
      break;
    case Settings::CIRCLES_GRID:
      found = findCirclesGrid( view, s.boardSize, pointBuf );
      break;
    case Settings::ASYMMETRIC_CIRCLES_GRID:
      found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
      break;
    default:
      found = false;
      break;
    }

    if ( found) {              // If done with success,
      // improve the found corners' coordinate accuracy for chessboard
      if ( s.calibrationPattern == Settings::CHESSBOARD) {
        Mat viewGray;
        cvtColor(view, viewGray, COLOR_BGR2GRAY);
        cornerSubPix( viewGray, pointBuf, Size(11, 11),
                      Size(-1, -1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1 ));
      }

      imagePoints.push_back(pointBuf);

      // Draw the corners.
      drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
    }
    else
    {
      std::cerr << "corners not found!" << std::endl;
    }

    //----------------------------- Output Text ------------------------------------------------
    std::string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
    int baseLine = 0;
    Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
    Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

    if ( mode == CAPTURING ) {
      if (s.showUndistorted)
        msg = format( "%d/%d Undist", (int)imagePoints.size(), images.size() );
      else
        msg = format( "%d/%d", (int)imagePoints.size(), images.size() );
    }

    putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

    //------------------------- Video capture  output  undistorted ------------------------------
    if ( mode == CALIBRATED && s.showUndistorted ) {
      Mat temp = view.clone();
      undistort(temp, view, cameraMatrix, distCoeffs);
    }

    //------------------------------ Show image and check for input commands -------------------
    imshow("Image View", view);
    char key = waitKey(s.delay);

    if ( key  == ESC_KEY )
      break;
  }
  
  if ( imagePoints.size() > 0 )
  {
    if (tryCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints)){
      std::cerr << "calibration complete" << std::endl;
    }
    else
    {
      std::cerr << "calibration failed" << std::endl;
    }
  } else {
    std::cerr << "error, images exhausted but no points in imagePoints" << std::endl;
  }

  // -----------------------Show the undistorted image for the image list ------------------------
  if (s.showUndistorted ) {
    
    Mat view, rview, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map1, map2);

    for(int i = 0; i < images.size(); i++ )
    {
        view = images[i];
        if(view.empty())
            continue;
        remap(view, rview, map1, map2, INTER_LINEAR);
        imshow("Image View", rview);
        char c = (char)waitKey();
        if( c  == ESC_KEY || c == 'q' || c == 'Q' )
            break;
    }
  }

  for (int i=0; i < distCoeffs.rows; ++i)
  {
    coeffs.push_back(distCoeffs.at<double>(i,0));
  }

  return coeffs;
}

static double computeReprojectionErrors( const std::vector<std::vector<Point3f> >& objectPoints,
    const std::vector<std::vector<Point2f> >& imagePoints,
    const std::vector<Mat>& rvecs, const std::vector<Mat>& tvecs,
    const Mat& cameraMatrix , const Mat& distCoeffs,
    std::vector<float>& perViewErrors) {
  std::vector<Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for ( i = 0; i < (int)objectPoints.size(); ++i ) {
    projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                   distCoeffs, imagePoints2);
    err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float) std::sqrt(err * err / n);
    totalErr        += err * err;
    totalPoints     += n;
  }

  return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, std::vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/) {
  corners.clear();

  switch (patternType) {
  case Settings::CHESSBOARD:
  case Settings::CIRCLES_GRID:
    for ( int i = 0; i < boardSize.height; ++i )
      for ( int j = 0; j < boardSize.width; ++j )
        corners.push_back(Point3f(float( j * squareSize ), float( i * squareSize ), 0));
    break;

  case Settings::ASYMMETRIC_CIRCLES_GRID:
    for ( int i = 0; i < boardSize.height; i++ )
      for ( int j = 0; j < boardSize.width; j++ )
        corners.push_back(Point3f(float((2 * j + i % 2)*squareSize), float(i * squareSize), 0));
    break;
  default:
    break;
  }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            std::vector<std::vector<Point2f> > imagePoints, std::vector<Mat>& rvecs, std::vector<Mat>& tvecs,
                            std::vector<float>& reprojErrs,  double& totalAvgErr) {

  cameraMatrix = Mat::eye(3, 3, CV_64F);
  if ( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
    cameraMatrix.at<double>(0, 0) = 1.0;

  distCoeffs = Mat::zeros(8, 1, CV_64F);

  std::vector<std::vector<Point3f> > objectPoints(1);
  calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

  objectPoints.resize(imagePoints.size(), objectPoints[0]);

  //Find intrinsic and extrinsic camera parameters
  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                               distCoeffs, rvecs, tvecs, s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

  totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                          rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

  return ok;
}


bool tryCalibration(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs, std::vector<std::vector<Point2f> > imagePoints ) {
  std::vector<Mat> rvecs, tvecs;
  std::vector<float> reprojErrs;
  double totalAvgErr = 0;

  bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                           reprojErrs, totalAvgErr);
  std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
            << ". avg re projection error = "  << totalAvgErr ;

  return ok;
}




////////////////////////////////////////////////////////
// Original test harness 
////////////////////////////////////////////////////////
class DistortionTest : public testing::Test {
 protected:
  virtual void SetUp() {
    has_all_images_ = false;
  }

  ros::NodeHandle nh_;
  std::vector<image_transport::Subscriber> cam_subs_;
  // the boolean is whether or not that topic has received image information
  std::map<std::string, bool> topics_acquired_;
  std::vector<sensor_msgs::ImageConstPtr> images_;
  bool has_all_images_;
 public:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, const std::string& topic) {
    std::cerr << "got an image on topic " << topic << std::endl;
    if (topics_acquired_[topic] == false) {
      topics_acquired_[topic] = true;
      images_.push_back(msg);
    }

    //TODO: this boolean check could be elegantly replaced by a lambda, I think
    bool all = true;
    for (auto it = topics_acquired_.begin(); it != topics_acquired_.end(); ++it) {
      all &= it->second;
    }
    has_all_images_ = all;
  }
};


TEST_F(DistortionTest, cameraDistortionTest) {
  const int NUM_CAMERAS = 8;
  for(int i=0; i < NUM_CAMERAS; ++i) {
    std::stringstream stream;
    stream << "camera" << (i+1) << "/image_raw";
    topics_acquired_.insert(std::pair<std::string, bool>(stream.str(), false));
  }

  image_transport::ImageTransport trans(nh_);
  for (auto it = topics_acquired_.begin(); it != topics_acquired_.end(); ++it) {
    cam_subs_.push_back(
      trans.subscribe(
        it->first,
        10,
        boost::bind(
          &DistortionTest::imageCallback,
          dynamic_cast<DistortionTest*>(this),
          _1,
          it->first
        )
      )
    );
  }

  while (!has_all_images_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  std::cerr << "got all images!" << std::endl;

  ros::NodeHandle nh_private_("~");
  std::string settings_file;
  nh_private_.getParam("calibration_settings_file", settings_file);

  std::cerr << "will load calibration settings from file path " << settings_file << std::endl;
  // ensure that the calibration settings file exists
  std::ifstream f(settings_file.c_str());
  assert(f.good());


  std::vector<Mat> images;

  for (auto it = images_.begin(); it != images_.end(); ++it)
  {
    images.push_back(Mat(cv_bridge::toCvCopy(*it)->image));
  }

  std::vector<double> coeffs = run(settings_file, images);
  std::cerr << "coeffs: " << std::endl;
  for(auto it = coeffs.begin(); it != coeffs.end(); ++it)
  {
    std::cerr << '\t' << *it << std::endl;
  }

  EXPECT_EQ(coeffs.size(), 5);
  const double THRESHOLD = 1e-1;
  EXPECT_NEAR(coeffs[0], -0.5, THRESHOLD);
  EXPECT_NEAR(coeffs[1], 0.0, THRESHOLD);
  EXPECT_NEAR(coeffs[2], 0.0, THRESHOLD);
  EXPECT_NEAR(coeffs[3], 0.0, THRESHOLD);
  EXPECT_NEAR(coeffs[4], 0.0, THRESHOLD);
  for (auto it = cam_subs_.begin(); it != cam_subs_.end(); ++it) {
    it->shutdown();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_camera_distortion_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
