// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.

/*
 * monocular_pose_estimator.cpp
 *
 * Created on: 04 22, 2022
 * Author: yuying.jin
 */

/** \file monocular_pose_estimator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */

#include "monocular_pose_estimator/monocular_pose_estimator.h"

namespace monocular_pose_estimator {

/**
 * Constructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::MPENode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), have_camera_info_(false) {
  // params_.fromNodeHandle(nh_private);

  nh_private_.param("stargazer_config", stargazer_config_, std::string("123"));

  nh_private_.param("threshold", threshold_, 20);
  nh_private_.param("tight_filter_size", tight_filter_size_, 3);
  nh_private_.param("wide_filter_size", wide_filter_size_, 11);
  nh_private_.param("maxRadiusForPixelCluster", maxRadiusForPixelCluster_, 3);
  nh_private_.param("minPixelForCluster", minPixelForCluster_, 1);
  nh_private_.param("maxPixelForCluster", maxPixelForCluster_, 1000);
  nh_private_.param("maxRadiusForCluster", maxRadiusForCluster_, 40);
  nh_private_.param("minPointsPerLandmark", minPointsPerLandmark_, 5);
  nh_private_.param("maxPointsPerLandmark", maxPointsPerLandmark_, 9);

  landmarkFinder = std::make_unique<monocular_pose_estimator::LandmarkFinder>(
      stargazer_config_, threshold_, tight_filter_size_, wide_filter_size_,
      maxRadiusForPixelCluster_, minPixelForCluster_, maxPixelForCluster_,
      maxRadiusForCluster_, minPointsPerLandmark_, maxPointsPerLandmark_);

  // Initialize subscribers
  image_sub_ =
      nh_.subscribe("/usb_cam/image_raw", 1, &MPENode::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("/usb_cam/camera_info", 1,
                                   &MPENode::cameraInfoCallback, this);

  // Initialize pose publisher
  pose_pub_ = nh_.advertise<monocular_pose_estimator::marker>(
      "/infrared_landmark/estimated_pose", 1);

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);
}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::~MPENode() {}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MPENode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
  if (!have_camera_info_) {
    cam_info_ = *msg;

    // Calibrated camera
    trackable_object_.camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
    trackable_object_.camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
    trackable_object_.camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
    trackable_object_.camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
    trackable_object_.camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
    trackable_object_.camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
    trackable_object_.camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
    trackable_object_.camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
    trackable_object_.camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
    trackable_object_.camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
    trackable_object_.camera_distortion_coeffs_ = cam_info_.D;

    have_camera_info_ = true;
    ROS_INFO("Camera calibration information obtained.");
  }
}

/**
 * The callback function that is executed every time an image is received. It
 * runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void MPENode::imageCallback(const sensor_msgs::Image::ConstPtr &image_msg) {

  // Check whether already received the camera calibration data
  if (!have_camera_info_) {
    ROS_WARN("No camera info yet...");
    return;
  }

  // Import the image from ROS message to OpenCV mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr =
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat image = cv_ptr->image;

  std::vector<monocular_pose_estimator::ImgLandmark> detected_img_landmarks;
  landmarkFinder->DetectLandmarks(image, detected_img_landmarks);
  if (detected_img_landmarks.size() == 0) {
    ROS_ERROR("NO Infrared landmark detected!");
    return;
  }

  // makers in the image
  double time_to_predict = image_msg->header.stamp.toSec();

  const bool found_body_pose = trackable_object_.estimateBodyPose(
      time_to_predict, detected_img_landmarks);

  if (found_body_pose) // Only output the pose, if the pose was updated (i.e.
  {
    // Eigen::Matrix4d transform = trackable_object.getPredictedPose();
    Matrix6d cov = trackable_object_.getPoseCovariance();
    Eigen::Matrix4d transform = trackable_object_.getPredictedPose();
    unsigned int markerId = trackable_object_.getMarkerId();

    ROS_DEBUG_STREAM("The transform: \n" << transform);
    ROS_DEBUG_STREAM("The covariance: \n" << cov);

    // Convert transform to PoseWithCovarianceStamped message
    predicted_pose_.header.stamp = image_msg->header.stamp;
    predicted_pose_.id = markerId;
    predicted_pose_.pose.pose.position.x = transform(0, 3);
    predicted_pose_.pose.pose.position.y = transform(1, 3);
    predicted_pose_.pose.pose.position.z = transform(2, 3);
    Eigen::Quaterniond orientation =
        Eigen::Quaterniond(transform.block<3, 3>(0, 0));
    predicted_pose_.pose.pose.orientation.x = orientation.x();
    predicted_pose_.pose.pose.orientation.y = orientation.y();
    predicted_pose_.pose.pose.orientation.z = orientation.z();
    predicted_pose_.pose.pose.orientation.w = orientation.w();

    // Add covariance to PoseWithCovarianceStamped message
    for (unsigned i = 0; i < 6; ++i) {
      for (unsigned j = 0; j < 6; ++j) {
        predicted_pose_.pose.covariance.elems[j + 6 * i] = cov(i, j);
      }
    }

    // Publish the pose
    pose_pub_.publish(predicted_pose_);
  } else { // If pose was not updated
    ROS_WARN("Unable to resolve a pose.");
  }

  // publish visualization image
  if (image_pub_.getNumSubscribers() > 0) {
    cv::Mat visualized_image = image.clone();
    cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);
    if (found_body_pose) {
      trackable_object_.augmentImage(visualized_image);
    }

    // Publish image for visualization
    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.header = image_msg->header;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = visualized_image;

    image_pub_.publish(visualized_image_msg.toImageMsg());
  }
}

} // namespace monocular_pose_estimator
