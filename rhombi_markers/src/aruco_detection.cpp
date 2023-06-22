#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
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
for (const aruco::Marker& marker : markers) {
  marker.draw(markerImage, cv::Scalar(0, 0, 255), 2);
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
  /*
    string camera_params_file = "/home/surgrob/projects/cam/src/spinnaker_sdk_camera_driver/params/gapter1.yaml";
    cv::FileStorage fs(camera_params_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Failed to open camera parameters file: %s", camera_params_file.c_str());
        return -1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    fs["intrinsic_coeffs"] >> cameraMatrix;
    fs["distortion_coeffs"] >> distCoeffs;

    fs.release();  // Close the file

  CamParam.CameraMatrix = cameraMatrix;
  CamParam.Distorsion = distCoeffs;
  */

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
