#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include "ultimate_msckf_vio/parameter_reader.h"
#include "ultimate_msckf_vio/frame.h"

namespace ultimate_msckf_vio {

constexpr int kMaxKeyPointsNumEachImage = 2000;
constexpr int kMinMatchNumInImage = 300;


class DataManager;

class FeatureTracker {
 public:
  FeatureTracker(DataManager* data_manager);

  static cv::Point2d PixelToCam(const cv::Point2d& pixel_point,
                                const Eigen::Matrix3d& K);

  void ProcessImage(cv_bridge::CvImage current_image);

  void OnReceiveNormalizedImage(const cv::Mat& normalized_image,
                                std::vector<cv::KeyPoint>* key_points,
                                cv::Mat* descriptors);

  // this func only for showing
  void UndistortRawImage(const cv::Mat& raw_image, cv::Mat* undistort_image);

  void FindFeatureMatchesBetweenFrames(const int frame0_id,
                                       const int frame1_id,
                                       std::vector<cv::DMatch>* good_matches);

  void FindFeatureMatchesBetweenFrames(const Frame& frame0,
                                       const Frame& frame1,
                                       std::vector<cv::DMatch>* good_matches);

  void UndistortFrameKeyPoints(Frame* frame);

  void UndistortFrameKeyPoints(int frame_id);



 private:
  bool ComputeOrbFeaturePoints(const cv::Mat& image,
                               std::vector<cv::KeyPoint>* keypoints,
                               cv::Mat* descriptors);

  void FindGoodFeatureMatches(const cv::Mat& prev_descriptors,
                              const cv::Mat& cur_descriptors,
                              std::vector<cv::DMatch>* good_matchs);

  bool FindFeatureMatchsByOrb(const cv::Mat& image_0,
                              const cv::Mat& image_1,
                              const int min_feature_num = kMinMatchNumInImage);

  bool FindFeatureMatchsByLK();

  std::vector<cv::Point2d> UndistortKeyPoints(
      const std::vector<cv::KeyPoint>& keypoints);

  cv::Point2d UndistortSinglePoint(const cv::KeyPoint& distorted_point);


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
  std::shared_ptr<DataManager> data_manager_;
//  cv::Ptr<cv::ORB> orb_;
};

}  // namespace ultimate_msckf_vio

#endif
