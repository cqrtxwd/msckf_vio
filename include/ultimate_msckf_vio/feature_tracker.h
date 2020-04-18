#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "ultimate_msckf_vio/parameter_reader.h"

namespace ultimate_msckf_vio {

constexpr int kMaxKeyPointsNumEachImage = 2000;
constexpr int kMinMatchNumInImage = 300;

class FeatureTracker {
 public:
  FeatureTracker(ParameterReader* parameter_reader)
    : image_count_(0), current_time_(-1), previous_time_(-1) {
    parameter_reader_.reset(parameter_reader);
  }

  void ProcessImage(cv_bridge::CvImage current_image);

  void OnReceiveNormalizedImage(const cv::Mat& normalized_image,
                                std::vector<cv::KeyPoint>* key_points,
                                cv::Mat* descriptors);

  // this func only for showing
  void UndistortRawImage(const cv::Mat& raw_image, cv::Mat* undistort_image);

 private:
  bool ComputeOrbFeaturePoints(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* descriptors);

  bool FindFeatureMatchsByOrb(const cv::Mat& image_0,
                              const cv::Mat& image_1,
                              const int min_feature_num = kMinMatchNumInImage);

  bool FindFeatureMatchsByLK();


  int image_count_;
  double current_time_;
  double previous_time_;
  cv::Mat current_image_;
  cv::Mat previous_image_;
  double cur_time_;
  double prev_time_;
  std::vector<cv::Point2f> current_points_;
  std::vector<cv::Point2f> previous_points_;
  std::shared_ptr<ParameterReader> parameter_reader_;
//  cv::Ptr<cv::ORB> orb_;
};

}  // namespace ultimate_msckf_vio

#endif
