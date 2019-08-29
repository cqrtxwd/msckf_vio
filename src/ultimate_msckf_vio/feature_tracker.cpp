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

  // clahe
  cv::Mat clahe_img;
  cv::Ptr<cv::CLAHE> cv_clahe = cv::createCLAHE(3);
  cv_clahe->apply(current_image.image, clahe_img);

  // find first initial feature points
  cv::goodFeaturesToTrack(clahe_img, previous_points_, 150, 0.01, 30);

  LOG(INFO) << "feature size: " << previous_points_.size();





  // // LKoptflow
  // vector<uchar> status;
  // vector<float> err;
  // cv::calcOpticalFlowPyrLK(previous_image_, current_image_, previous_points_,
  //                          current_points_, status, err);
  // ROS_INFO_STREAM("image count: " << image_count_++);
  // ROS_INFO_STREAM("image time: " << current_time_);
}

}  // namespace ultimate_msckf_vio
