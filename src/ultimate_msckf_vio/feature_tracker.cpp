#include "ultimate_msckf_vio/feature_tracker.h"
#include "ultimate_msckf_vio/data_manager.h"

namespace ultimate_msckf_vio {

FeatureTracker::FeatureTracker(DataManager* data_manager)
  : image_count_(0), current_time_(-1), previous_time_(-1) {
  CHECK(data_manager != nullptr)
      << "data manager haven't initalize";
  data_manager_.reset(data_manager);
  CHECK(data_manager_->parameter_reader_ != nullptr)
      << "parameter_reader in data manager haven't initalize";
  parameter_reader_.reset(data_manager_->parameter_reader_.get());
}

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

void FeatureTracker::FindFeatureMatchesBetweenFrames(
    const int frame0_id, const int frame1_id,
    std::vector<cv::DMatch>* good_matches) {
  std::lock_guard<std::mutex> lock(data_manager_->frame_mutex_);
  CHECK(std::max(frame0_id, frame1_id) <= data_manager_->frames_.size())
      << "input frame id invalid, input id : " << frame0_id << ", "<< frame1_id
      << "while max frame id is : " << data_manager_->frames_.size();
//  LOG(INFO) << "find " << frame0_id << " " << frame1_id;
  LOG(INFO) << "data_manager_->frames_.size() " << data_manager_->frames_.size();
  FindGoodFeatureMatches(data_manager_->frames_[frame0_id].descriptors,
                         data_manager_->frames_[frame1_id].descriptors,
                         good_matches);
  LOG(INFO) << "good_matches" << good_matches->size();
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

void FeatureTracker::FindGoodFeatureMatches(
    const cv::Mat& prev_descriptors,
    const cv::Mat& cur_descriptors,
    std::vector<cv::DMatch>* good_matchs) {
  std::vector<cv::DMatch> match_pairs;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(prev_descriptors, cur_descriptors, match_pairs);
  LOG(INFO) << "init match size: " << match_pairs.size();

  double coarse_good_match_threshold = 50.0;
  std::vector<cv::DMatch> coarse_good_matchs;
  for (auto& match : match_pairs) {
    if (match.distance <= coarse_good_match_threshold) {
      coarse_good_matchs.push_back(std::move(match));
    }
  }
  if (coarse_good_matchs.size() < 50) {
    LOG(WARNING) << "coarse_good_matchs size < 50 , num is"
                 << coarse_good_matchs.size();
  }

  std::sort(coarse_good_matchs.begin(), coarse_good_matchs.end());

  // ensure at least kMinMatchNumInImage(default 300) matches in each image
   *good_matchs =
      std::vector<cv::DMatch>(coarse_good_matchs.begin(),
                              coarse_good_matchs.begin() + kMinMatchNumInImage);
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
