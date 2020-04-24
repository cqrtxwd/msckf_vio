#include "ultimate_msckf_vio/vio_initializer.h"

#include "ultimate_msckf_vio/data_manager.h"


namespace ultimate_msckf_vio {

VIOInitializer::VIOInitializer(DataManager* data_manager_ptr)
  : is_initialized_(false) {
  CHECK(data_manager_ptr != nullptr) << "data manager haven't initialized";
    data_manager_.reset(data_manager_ptr);
}

bool VIOInitializer::is_initialized() {
  return is_initialized_;
}

bool VIOInitializer::TryInitialize() {
//  LOG(INFO) << "try initialize";
  auto num_frames = data_manager_->frames_.size();
  if (num_frames < kNumFramesToInitialize) {
    return false;
  }

  // find 2 frames has most matches, build initial point cloud
  for (int i = 0; i < num_frames - 1; i++) {
    for (int j = i + 1; j < num_frames; j++) {
      LOG(INFO) << i << " -> " << j;
      std::vector<cv::DMatch> tmp_good_matches;
      data_manager_->feature_tracker_->
          FindFeatureMatchesBetweenFrames(i, j, &tmp_good_matches);
    }
  }


  is_initialized_ = true;
  return true;
}

}

