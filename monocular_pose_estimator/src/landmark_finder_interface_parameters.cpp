#include "monocular_pose_estimator/landmark_finder_interface_parameters.h"
#include "monocular_pose_estimator/ros_utils.h"

namespace monocular_pose_estimator {

void LandmarkFinderInterfaceParameters::fromNodeHandle(
    const ros::NodeHandle &nh) {

  getParam(nh, "stargazer_config", stargazer_config);

  getParam(nh, "threshold", cfg.threshold);
  getParam(nh, "tight_filter_size", cfg.tight_filter_size);
  getParam(nh, "wide_filter_size", cfg.wide_filter_size);
  getParam(nh, "maxRadiusForPixelCluster", cfg.maxRadiusForPixelCluster);
  getParam(nh, "minPixelForCluster", cfg.minPixelForCluster);
  getParam(nh, "maxPixelForCluster", cfg.maxPixelForCluster);
  getParam(nh, "maxRadiusForCluster", cfg.maxRadiusForCluster);
  getParam(nh, "minPointsPerLandmark", cfg.minPointsPerLandmark);
  getParam(nh, "maxPointsPerLandmark", cfg.maxPointsPerLandmark);
}

void LandmarkFinderInterfaceParameters::fromConfig(const Config &config,
                                                   const uint32_t &) {
  cfg = config;
}
} // namespace monocular_pose_estimator
