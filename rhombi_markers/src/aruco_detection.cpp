#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseArray.h>
#include "aruco.h"

using namespace std;
using namespace cv;

//#define _DICTIONARY aruco::DIC
#define RATE 30

geometry_msgs::PoseArray pose_publishing;
aruco::MarkerDetector Mdetector;
aruco::CameraParameters CamParam;
//aruco::Dictionary dictionary;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Image processing callback implementation
  ROS_INFO("Processing image");

  cv::Mat image;
try {
  // Convert the ROS image message to cv::Mat format
  image = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()));
} catch (const cv::Exception& e) {
  ROS_ERROR("Failed to convert ROS image message to cv::Mat format: %s", e.what());
  return;
}

// Perform ArUco marker detection on the image
std::vector<aruco::Marker> markers;
Mdetector.detect(image, markers, CamParam, 0.03, false);

// Visualize the detected ArUco markers
cv::Mat markerImage = image.clone();
uint8_t i=0;
for (const aruco::Marker& marker : markers) {
  std::cout << "Marker " << ++i << ":" << marker.Rvec << " " << marker.Tvec << std::endl;
  // Calculate pose for each marker
  marker.draw(markerImage, cv::Scalar(0, 0, 255), 2);
  //marker.d
}

// Display the image with ArUco marker overlay
cv::imshow("ArUco Detection", markerImage);
cv::waitKey(1);
  }


int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detection");
  ros::start();
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Rate loop_rate(RATE);

  // Load camera parameters if available

    cv::Mat cameraMatrix = (Mat1d(3, 3) << 2.1742285497994644e+03 ,0.0, 1.0585124352335515e+03, 0.0, 2.1834371437552527e+03, 8.3380515034517452e+02, 0.0, 0.0, 1.0);
    cv::Mat distCoeffs = (Mat1d(1, 5) << -2.3646638455924696e-01, 1.8538700606070738e-01, -1.4635424579430090e-03, 3.6616013209817078e-03, -8.9736280250555356e-02);

  CamParam.CameraMatrix = cameraMatrix;
  CamParam.Distorsion = distCoeffs;

  // Configure marker detection parameters
  aruco::MarkerDetector::Params detector_params;
  detector_params.detectEnclosedMarkers(true);

  Mdetector.setDictionary("ARUCO_MIP_36h12");  
  Mdetector.setParameters(detector_params);

  // Subscribe to the image topic
  image_transport::Subscriber sub = it.subscribe("/camera_array/cam0/image_raw", 1, imageCallback);

  ros::Publisher PosePub = nh.advertise<geometry_msgs::PoseArray>("/pose_stamped", 1);

  ROS_INFO("Starting detection");

  while (ros::ok()) {
    if (!pose_publishing.poses.empty()) {
      // Publish the detected poses
      PosePub.publish(pose_publishing);
      pose_publishing.poses.clear();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
