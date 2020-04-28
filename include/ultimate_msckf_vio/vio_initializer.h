#ifndef VIO_INITIALIZER_H
#define VIO_INITIALIZER_H

#include <memory>
#include "ultimate_msckf_vio/frame.h"
#include "ultimate_msckf_vio/utility/pose.h"

namespace ultimate_msckf_vio {

constexpr int kNumFramesToInitialize = 20;

class DataManager;

class VIOInitializer {
 public:
  VIOInitializer() = delete;

  VIOInitializer(DataManager* data_manager_ptr);

  bool is_initialized();

  bool TryInitialize();

  bool SolveEpipolarGeometryOf2Frames(
      const Frame& frame0,
      const Frame& frame1,
      Pose3d* result_pose);

  bool SolveEpipolarGeometryOf2Frames(
      const Frame& frame0,
      const Frame& frame1,
      const std::vector<cv::DMatch>& feature_matches,
      Pose3d* result_pose);

  bool SolveEpipolarGeometryOf2Frames(
      const int& frame0_id,
      const int& frame1_id,
      const std::vector<cv::DMatch>& feature_matches,
      Pose3d* result_pose);

 private:

  std::shared_ptr<DataManager> data_manager_;
  bool is_initialized_;
};

}


#endif // VIO_INITIALIZER_H
