#include "ultimate_msckf_vio/feature_tracker.h"

namespace ultimate_msckf_vio {

void FeatrueTracker::ProcessImage(cv_bridge::CvImage current_image) {
  previous_time_ = current_time_;
  current_time_ = current_image.header.stamp.toSec();
  previous_image_ = current_image_;
  current_image_ = current_image.image;
  if(previous_image_.empty()) {
    return;
  }
  // LKoptflow
  vector<uchar> status;
  vector<float> err;
  cv::calcOpticalFlowPyrLK(previous_image_, current_image_, previous_points_,
                           current_points_, status, err);
  ROS_INFO_STREAM("image count: " << image_count_++);
  ROS_INFO_STREAM("image time: " << current_time_);
}

}  // namespace ultimate_msckf_vio