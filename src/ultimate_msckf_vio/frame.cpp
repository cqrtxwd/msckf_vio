#include "ultimate_msckf_vio/frame.h"

namespace ultimate_msckf_vio {

Frame::Frame(const int tmp_id,
             const ros::Time& tmp_image_timestamp,
             std::vector<cv::KeyPoint>& tmp_keypoints,
             cv::Mat& tmp_descriptors) {
  id = tmp_id;
  timestamp = tmp_image_timestamp;
  keypoints = std::move(tmp_keypoints);
  descriptors = std::move(tmp_descriptors);
  undistort_keypoints.reserve(keypoints.size());
}

Frame::Frame(const int tmp_id,
             const ros::Time& tmp_image_timestamp,
             std::vector<cv::Point2d>& tmp_undistort_keypoints,
             cv::Mat& tmp_descriptors) {
  id = tmp_id;
  timestamp = tmp_image_timestamp;
  undistort_keypoints = std::move(tmp_undistort_keypoints);
  descriptors = std::move(tmp_descriptors);
  keypoints.reserve(undistort_keypoints.size());
}
}


