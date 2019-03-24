#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"

namespace ultimate_msckf_vio {

using std::vector;

class FeatrueTracker {
 public:
  FeatrueTracker() : image_count_(0), current_time_(-1), previous_time_(-1) {}

  void ProcessImage(cv_bridge::CvImage current_image);

 private:
  int image_count_;
  double current_time_;
  double previous_time_;
  cv::Mat current_image_;
  cv::Mat previous_image_;
  vector<cv::Point2f> current_points_;
  vector<cv::Point2f> previous_points_;
};

}  // namespace ultimate_msckf_vio

#endif