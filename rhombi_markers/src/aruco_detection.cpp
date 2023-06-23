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

#define RATE 30

geometry_msgs::PoseArray pose_publishing;
aruco::MarkerDetector Mdetector;
aruco::CameraParameters CamParam;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("Processing image");

  cv::Mat image;
  try {
    // Convert the ROS image message to cv::Mat format
    image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  } catch (const cv::Exception& e) {
    ROS_ERROR("Failed to convert ROS image message to cv::Mat format: %s", e.what());
    return;
  }

  // Perform ArUco marker detection on the image
  std::vector<aruco::Marker> markers;
  Mdetector.detect(image, markers, CamParam, 0.03, false);

  // Visualize the detected ArUco markers
  cv::Mat markerImage = image.clone();
  uint8_t i = 0;
  for (const aruco::Marker& marker : markers) {
    std::cout << "Marker " << ++i << ":" << marker.Rvec << " " << marker.Tvec << std::endl;
    // Calculate pose for each marker
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

  aruco::MarkerDetector::Params detector_params;
  detector_params.detectEnclosedMarkers(true);
  Mdetector.setDictionary("ARUCO_MIP_36h12");
  Mdetector.setParameters(detector_params);

  sensor_msgs::CameraInfoConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_array/cam0/camera_info", nh);
  if (camera_info != NULL) {
    sensor_msgs::CameraInfo cam_inf = *camera_info;
    CamParam.setParams(cv::Mat(3, 3, CV_64F, &cam_inf.K[0]), cv::Mat(1, 5, CV_64F, &cam_inf.D[0]), Size(cam_inf.width, cam_inf.height));
  } else {
    ROS_INFO("Camera info file received is empty. Exiting...");
    return -1;
  }

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
