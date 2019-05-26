#ifndef VISUAL_OBSERVATION_MANAGER_H_
#define VISUAL_OBSERVATION_MANAGER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Eigen>
#include "glog/logging.h"
#include "ultimate_msckf_vio/common_data/common_data.h"
#include "ultimate_msckf_vio/feature_bundle.h"
#include "ultimate_msckf_vio/ekf_state.h"
#include "ultimate_msckf_vio/visual_manager_interface.h"


namespace ultimate_msckf_vio {

constexpr int kMaxSlideWindowSize = 16;
constexpr int kLostTrackThreshold = 2;

using std::vector;
using std::deque;
using std::map;
using sensor_msgs::PointCloudConstPtr;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class VisualObservationManager : public VisualManagerInterface {
 public:
  VisualObservationManager():
    frame_count_(0),
    keyframe_count_(0) {}

  void Initialize(EkfStated* ekf_state) {
    ekf_state_ = ekf_state;
  }

  bool AddNewKeyFrame(const ImageInfo&);

  void FindCompletedFeatureBundles(
      vector<FeatureBundle>* completed_feature_bundles);

  int keyframe_count() {
    return keyframe_count_;
  }

  bool ReachMaximumWindowSize() {
    return keyframe_ids_.size() >= kMaxSlideWindowSize;
  }

  bool ShouldMarginalize();

  void MarginalizeOldestKeyframe();



 private:


  // keyframes in slide window
  // vector<PointCloudConstPtr> keyframes;

  // id of keyframes
  deque<int> keyframe_ids_;

  // features in each keyframe
  map<int, vector<int>> keyframe_to_features; // maybe list<vector<int>> ?

  // feature id , every feature's unique id
  vector<int> features_id_;

  // aligned to features_id_
  vector<bool> feature_optimized_;

  // feature bundles, basicly record keyframes that observed this feature
  map<int /*feature_id*/, FeatureBundle> feature_bundles_;


  int frame_count_;
  int keyframe_count_;
  int num_keyframes_;  // always equal to keyframe_ids_.size()

  // listener
  EkfStated* ekf_state_;
};

typedef std::shared_ptr<VisualObservationManager> VisualObservationManagerPtr;
typedef std::shared_ptr<const VisualObservationManager> VisualObservationManagerConstPtr;

}

#endif // VISUAL_OBSERVATION_MANAGER_H_
