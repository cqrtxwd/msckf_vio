#include "ultimate_msckf_vio/feature_tracker.h"

namespace ultimate_msckf_vio {

void FeatureTracker::ProcessImage(cv_bridge::CvImage current_image) {
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

}

void FeatureTracker::OnReceiveNormalizedImage(
    const cv::Mat& normalized_image,
    std::vector<cv::KeyPoint>* key_points,
    cv::Mat* descriptors) {
  LOG(INFO) << "normalized image coming";

  if (current_image_.empty()) {
    current_image_ = normalized_image;
    return;
  }
  previous_image_ = current_image_;
  current_image_ = normalized_image;
  // compute orb feature
  ComputeOrbFeaturePoints(normalized_image, key_points, descriptors);

//  FindFeatureMatchsByOrb(previous_image_, current_image_);
}

void FeatureTracker::UndistortRawImage(
    const cv::Mat& raw_image, cv::Mat* undistort_image) {
  LOG(INFO) << "UndistortRawImage ";
  cv::Mat cam_intrinsic = cv::Mat::eye(3, 3, CV_64F);
//  cam_intrinsic.at<double>(0, 0) = parameter_reader_->intrinsic_mat(0, 0);
//  cv::initUndistortRectifyMap();



}

bool FeatureTracker::ComputeOrbFeaturePoints(
    const cv::Mat& image,
    std::vector<cv::KeyPoint>* keypoints,
    cv::Mat* descriptors) {
  auto orb = cv::ORB::create(kMaxKeyPointsNumEachImage);
  orb->detect(image, *keypoints);
  orb->compute(image, *keypoints, *descriptors);
  return true;
}

bool FeatureTracker::FindFeatureMatchsByOrb(const cv::Mat& previous_image,
                                            const cv::Mat& current_image,
                                            const int min_feature_num) {
  if (previous_image_.empty() || current_image_.empty()) {
    return false;
  }

  // detect
  std::vector<cv::KeyPoint> prev_keypoints, cur_keypoints;
  cv::Mat prev_descriptors, cur_descriptors;
  ComputeOrbFeaturePoints(previous_image, &prev_keypoints, &prev_descriptors);
  ComputeOrbFeaturePoints(current_image, &cur_keypoints, &cur_descriptors);

  std::vector<cv::DMatch> matche_pairs;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(prev_descriptors, cur_descriptors, matche_pairs);

  double min_dist = 1000;
  double max_dist = 0;
  double coarse_good_match_threshold = 50;
  std::vector<cv::DMatch> good_matchs;
  for (auto& match : matche_pairs) {
//    min_dist = match.distance < min_dist ? match.distance : min_dist;
//    max_dist = match.distance > max_dist ? match.distance : max_dist;
    if (match.distance <= coarse_good_match_threshold) {
      good_matchs.push_back(std::move(match));
    }
  }
  std::sort(good_matchs.begin(), good_matchs.end());

  // ensure at least min_feature_num = 300 matches in each image
  std::vector<cv::DMatch> final_good_matchs =
      std::vector<cv::DMatch>(good_matchs.begin(),
                              good_matchs.begin() + min_feature_num);

  cv::Mat match_show_image;
  cv::drawMatches(previous_image, prev_keypoints,
                  current_image, cur_keypoints,
                  good_matchs, match_show_image);
  cv::imshow("matches found" , match_show_image);
  cv::waitKey(5);
  LOG(INFO) << "found good match " << final_good_matchs.size();

}

}  // namespace ultimate_msckf_vio
