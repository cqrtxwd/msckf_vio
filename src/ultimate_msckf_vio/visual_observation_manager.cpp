#include "ultimate_msckf_vio/visual_observation_manager.h"

namespace ultimate_msckf_vio {

bool VisualObservationManager::AddNewKeyFrame(const ImageInfo& image) {
  ROS_INFO_STREAM("AddNewKeyFrame" << keyframe_count_;);
  keyframe_count_++;
  num_keyframes_++;
  keyframe_ids_.push_back(keyframe_count_);
  for (auto feaid_info : image) {
    int feature_id = feaid_info.first;
    Eigen::Matrix<double, 7, 1> feature_info = feaid_info.second.back().second;
    Vector2d feature_uv = Vector2d(feature_info(3), feature_info(4));

    // add info to manager
    int cur_keyframe_id = keyframe_count_;
    features_id_.push_back(feature_id);
    keyframe_to_features[cur_keyframe_id].emplace_back(feature_id);
    feature_optimized_.push_back(false);

    // mantain feature buldles , create new bundle or add point to old bundle
    if (feature_bundles_.find(feature_id) == feature_bundles_.end()) {
      // if not find , create new bundle
      feature_bundles_.insert(std::make_pair(feature_id,
                                             FeatureBundle(feature_id,
                                                           cur_keyframe_id,
                                                           feature_uv,
                                                           this)));
    } else {
      // if find, add feature to
      feature_bundles_[feature_id].AddObservedKeyframe(cur_keyframe_id,
                                                       feature_uv);
    }
  }

//  // debug
//  auto cur_kf = keyframe_to_features[keyframe_count_];
//  for(auto fea_id : cur_kf) {
//    LOG(INFO) << "fea_id : " << fea_id;
//  }



  LOG(INFO) << "feature_bundles_ size: " << feature_bundles_.size();
  return true;
}



bool VisualObservationManager::ShouldMarginalize() {
  // for now, just marginalize when reache the maximum window size
  return keyframe_ids_.size() >= kMaxSlideWindowSize;
}


void VisualObservationManager::MarginalizeOldestKeyframe() {
  ekf_state_->MarginalizeOldestKeyframe();
}

void VisualObservationManager::FindCompletedFeatureBundles(
    vector<FeatureBundle>* completed_feature_bundles) {
  CHECK(completed_feature_bundles!=NULL)
      << "completed_feature_bundles is null";
  for (auto fea_id_to_bundle : feature_bundles_) {
    auto feature_bundle = fea_id_to_bundle.second;
    if(keyframe_count_ - feature_bundle.LastTrackKeyframeId()
       >= kLostTrackThreshold) {
      feature_bundle.SetReady();
      completed_feature_bundles->push_back(feature_bundle);
    }
  }
}


}
