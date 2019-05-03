#include "ultimate_msckf_vio/visual_observation_manager.h"

namespace ultimate_msckf_vio {

bool VisualObservationManager::AddNewKeyFrame(const ImageInfo& image) {
  ROS_INFO_STREAM("AddNewKeyFrame" << keyframe_count_;);
  keyframe_count_++;
  keyframe_ids_.push_back(keyframe_count_);
  for (auto feaid_info : image) {
    int feature_id = feaid_info.first;
    Eigen::Matrix<double, 7, 1> feature_info = feaid_info.second.back().second;
    Vector2d feature_uv = Vector2d(feature_info(3), feature_info(4));
    features_id_.push_back(feature_id);
    keyframe_to_features[keyframe_count_].emplace_back(feature_id);
    feature_optimized_.push_back(false);

    // mantain feature buldles , create new bundle or add point to old bundle
    if (feature_bundles_.find(feature_id) == feature_bundles_.end()) {
      feature_bundles_.insert(std::make_pair(feature_id,
                                             FeatureBundle(feature_id,
                                                           keyframe_count_,
                                                           feature_uv)));
    } else {
      feature_bundles_[feature_id].AddObservedKeyframe(keyframe_count_,
                                                       feature_uv);
    }
  }

//  // debug
//  auto cur_kf = keyframe_to_features[keyframe_count_];
//  for(auto fea_id : cur_kf) {
//    ROS_INFO_STREAM("fea_id : " << fea_id);
//  }

  // ekf_state and cov agument
  ekf_state_->AugmentStateAndCovariance();

  ROS_INFO_STREAM("feature_bundles_ size: " << feature_bundles_.size(););
  return true;
}



bool VisualObservationManager::ShouldMarginalize() {
  // for now, just marginalize when reache the maximum window size
  return keyframe_ids_.size() >= kSlideWindowMaxSize;
}


void VisualObservationManager::MarginalizeOldestKeyframe() {
  ekf_state_->MarginalizeOldestKeyframe();
}


}
