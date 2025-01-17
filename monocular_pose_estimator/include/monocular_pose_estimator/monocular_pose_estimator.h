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
 * monocular_pose_estimator_node.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Matthias Fässler
 */

/** \file monocular_pose_estimator_node.h
 * \brief File containing the definition of the Monocular Pose Estimator Node
 * class
 *
 */

#ifndef MONOCULAR_POSE_ESTIMATOR_NODE_H_
#define MONOCULAR_POSE_ESTIMATOR_NODE_H_

#include "ros/ros.h"

#include <memory>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <monocular_pose_estimator/MonocularPoseEstimatorConfig.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "monocular_pose_estimator/marker.h"
#include "monocular_pose_estimator_lib/landmark_finder.h"
#include "monocular_pose_estimator_lib/pose_estimator.h"

namespace monocular_pose_estimator {

class MPENode {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::Publisher image_pub_; //!< The ROS image publisher that
                                         //!< publishes the visualisation image
  ros::Publisher
      pose_pub_; //!< The ROS publisher that publishes the estimated pose.

  ros::Subscriber image_sub_; //!< The ROS subscriber to the raw camera image
  ros::Subscriber camera_info_sub_; //!< The ROS subscriber to the camera info

  dynamic_reconfigure::Server<
      monocular_pose_estimator::MonocularPoseEstimatorConfig>
      dr_server_; //!< The dynamic reconfigure server
  // dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType
  // cb_; //!< The dynamic reconfigure callback type

  monocular_pose_estimator::marker
      predicted_pose_; //!< The ROS message variable for the estimated pose and
                       //!< covariance of the object

  bool have_camera_info_; //!< The boolean variable that indicates whether the
                          //!< camera calibration parameters have been obtained
                          //!< from the camera
  sensor_msgs::CameraInfo
      cam_info_; //!< Variable to store the camera calibration parameters

  PoseEstimator trackable_object_; //!< Declaration of the object whose pose
                                   //!< will be estimated

  std::unique_ptr<monocular_pose_estimator::LandmarkFinder> landmarkFinder;

  std::string stargazer_config_;
  int threshold_, tight_filter_size_, wide_filter_size_,
      maxRadiusForPixelCluster_, minPixelForCluster_, maxPixelForCluster_,
      maxRadiusForCluster_, minPointsPerLandmark_, maxPointsPerLandmark_;

public:
  MPENode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  MPENode() : MPENode(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~MPENode();

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr &image_msg);

  void dynamicParametersCallback(
      monocular_pose_estimator::MonocularPoseEstimatorConfig &config,
      uint32_t level);
};

} // namespace monocular_pose_estimator

#endif /* MONOCULAR_POSE_ESTIMATOR_NODE_H_ */
