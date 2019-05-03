#ifndef FEATURE_BUNDLE_H_
#define FEATURE_BUNDLE_H_

#include <deque>
#include "eigen3/Eigen/Eigen"
#include "ros/ros.h"

namespace ultimate_msckf_vio {

using Eigen::Vector2d;
using std::deque;
using std::map;
using std::vector;

// a single feature point observed by multi-keyframe, and impose  multi
// constrains to these keyframes

class FeatureBundle {
 public:
  FeatureBundle() : ready_to_optimize_(false), optimized_(false) {}

  FeatureBundle(int feature_id, int keyframe_id, Vector2d feature_uv)
      : feature_id_(feature_id), ready_to_optimize_(false), optimized_(false) {
    observed_keframes_id_.push_back(keyframe_id);
    observed_uv_.push_back(feature_uv);
  }

  void AddObservedKeyframe(int keyframe_id, Vector2d feature_uv) {
    observed_keframes_id_.push_back(keyframe_id);
    observed_uv_.push_back(feature_uv);
  }

  int KeyframeNum() {
    return observed_keframes_id_.size();
  }

  // for debug
  void PrintInfo() {
    ROS_INFO_STREAM("observed_keframes_id_ size : " << observed_keframes_id_.size(););
    for (int id :  observed_keframes_id_) {
      std::cout << " observed by kf : " << id;
    }
    std::cout << "\n";
  }

 private:
  int feature_id_;

  vector<int> observed_keframes_id_;

  vector<Vector2d> observed_uv_;

  bool ready_to_optimize_;

  bool optimized_;

  //  vector<int> feature_indexs_;
};

}  // namespace ultimate_msckf_vio

#endif  // FEATURE_BUNDLE_H_
