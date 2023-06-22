#include "aruco_detection.h"

ArucoDetection::ArucoDetection() : nh_(""), it_(nh_) {
  // Constructor implementation
}

ArucoDetection::~ArucoDetection() {
  // Destructor implementation
}

void ArucoDetection::initialize() {
  // Initialization implementation

  // Subscribe to the image topic
  image_sub_ = it_.subscribe("/camera_array/cam0/image_raw", 1, &ArucoDetection::imageCallback, this);
  // Set ArUco dictionary
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

void ArucoDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // Image processing callback implementation
  ROS_INFO("processing image");

  // Convert the ROS image message to OpenCV image format using cv_bridge
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    // Perform ArUco marker detection on the image
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, markerCorners, markerIds, parameters);

    // Visualize the detected ArUco markers
    cv::aruco::drawDetectedMarkers(cv_ptr->image, markerCorners, markerIds);

    // Display the image with ArUco marker overlay
    cv::imshow("ArUco Detection", cv_ptr->image);
    cv::waitKey(1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "aruco_detection_node");
  ros::Rate rate(10);

  ArucoDetection aruco_detection;
  aruco_detection.initialize();

  ROS_INFO("initialized");

  ros::spinOnce();
  
  return 0;
}
