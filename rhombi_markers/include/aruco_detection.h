#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "aruco.h"
//#include <cvdrawingutils.h>

class ArucoDetection {
public:
  ArucoDetection();  // Constructor
  ~ArucoDetection(); // Destructor

  void initialize(); // Initialize ArUco detection
  int DICTIONARY = 1;				//ArUco dict to use: ARUCO_MIP_36h12=1, ARUCO=2, ARUCO_MIP_25h7=3, ARUCO_MIP_16h3=4 etc.
  geometry_msgs::PoseArray pose_publishing;

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

#endif  // ARUCO_DETECTION_H
