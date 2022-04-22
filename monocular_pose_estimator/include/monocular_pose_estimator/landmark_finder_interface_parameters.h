#ifndef LANDMARK_FINDER_INTERFACE_PARAMETERS_H_
#define LANDMARK_FINDER_INTERFACE_PARAMETERS_H_

#include <monocular_pose_estimator/LandmarkFinderConfig.h>
#include <ros/node_handle.h>
#include <string>

namespace monocular_pose_estimator {

struct LandmarkFinderInterfaceParameters {

  using Config = LandmarkFinderConfig;

  void fromNodeHandle(const ros::NodeHandle &);
  void fromConfig(const Config &, const uint32_t & = 0);

  std::string stargazer_config;

  Config cfg;
};
} // namespace monocular_pose_estimator
#endif
