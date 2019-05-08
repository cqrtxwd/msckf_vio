#ifndef FEATURE_BUNDLE_H_
#define FEATURE_BUNDLE_H_

#include <deque>
#include "eigen3/Eigen/Eigen"
#include "ros/ros.h"
#include "ultimate_msckf_vio/visual_manager_interface.h"

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

  FeatureBundle(int feature_id,
                int keyframe_id,
                Vector2d feature_uv,
                VisualManagerInterface* visual_manager_listener)
      : feature_id_(feature_id),
        ready_to_optimize_(false),
        optimized_(false),
        visual_manager_listener_(visual_manager_listener) {
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

  //  how long this feature lost track consistantly, when lost track more than
  //  2 or 3 times, consider it lost track, and this bundle ready to optimaize
  int lost_count_;

  //  vector<int> feature_indexs_;

  //listener
  VisualManagerInterface* visual_manager_listener_;

};

}  // namespace ultimate_msckf_vio

#endif  // FEATURE_BUNDLE_H_