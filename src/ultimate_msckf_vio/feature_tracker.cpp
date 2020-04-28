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

cv::Point2d FeatureTracker::PixelToCam(const cv::Point2d& pixel_point,
                                       const Eigen::Matrix3d& K) {
  Eigen::Vector3d pixel_point_eigen(pixel_point.x, pixel_point.y, 1.0);
  Eigen::Vector3d cam_point_eigen = K.inverse() * pixel_point_eigen;

  cv::Point2d cam_point(cam_point_eigen.x(), cam_point_eigen.y());
  return cam_point;
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
  FindFeatureMatchesBetweenFrames(data_manager_->GetFrameById(frame0_id),
                                  data_manager_->GetFrameById(frame1_id),
                                  good_matches);
}

void FeatureTracker::FindFeatureMatchesBetweenFrames(
    const Frame& frame0, const Frame& frame1,
    std::vector<cv::DMatch>* good_matches) {
  FindGoodFeatureMatches(frame0.descriptors,
                         frame1.descriptors,
                         good_matches);
}

void FeatureTracker::UndistortFrameKeyPoints(Frame* frame) {
  frame->undistort_keypoints = std::move(UndistortKeyPoints(frame->keypoints));
}

void FeatureTracker::UndistortFrameKeyPoints(int frame_id) {
  CHECK(frame_id < data_manager_->frames_.size() && frame_id >= 0)
      << "frame id invalid !!!";
  auto& frame = data_manager_->frames_[frame_id];
  UndistortFrameKeyPoints(&frame);
  LOG(INFO) << "undistort_keypoints " << frame.undistort_keypoints.size();
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

  CHECK(match_pairs.size() > 0) << "no feature match between 2 frame !!!";

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
  *good_matchs = std::move(coarse_good_matchs);

//  std::sort(coarse_good_matchs.begin(), coarse_good_matchs.end());

//  // ensure at least kMinMatchNumInImage(default 300) matches in each image
//   *good_matchs =
//      std::vector<cv::DMatch>(coarse_good_matchs.begin(),
//                              coarse_good_matchs.begin() + kMinMatchNumInImage);
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

std::vector<cv::Point2d> FeatureTracker::UndistortKeyPoints(
    const std::vector<cv::KeyPoint>& keypoints) {
  CHECK(!keypoints.empty()) << "keypoints to be undistort is empty !!!";
  std::vector<cv::Point2d> undistort_keypoints;
  undistort_keypoints.reserve(keypoints.size());
  for (auto distorted_keypoint : keypoints) {
    undistort_keypoints.push_back(
          std::move(UndistortSinglePoint(distorted_keypoint)));
  }
  return undistort_keypoints;
}

cv::Point2d FeatureTracker::UndistortSinglePoint(
    const cv::KeyPoint& distorted_point) {
  // inverse distortion model
  double k1 = parameter_reader_->distortion_k1;
  double k2 = parameter_reader_->distortion_k2;
  double p1 = parameter_reader_->distortion_p1;
  double p2 = parameter_reader_->distortion_p2;
  Matrix3d K_inv = parameter_reader_->intrinsic_mat.inverse();
  double u = distorted_point.pt.x;
  double v = distorted_point.pt.y;
  Eigen::Vector3d norm_p = K_inv * Eigen::Vector3d(u, v, 1);

  double x_distort = norm_p.x();
  double y_distort = norm_p.y();

  double xy_distort = x_distort * y_distort;
  double x2_distort = x_distort * x_distort;
  double y2_distort = y_distort * y_distort;
  double r2_distort = x2_distort + y2_distort;
  double r4_distort = r2_distort * r2_distort;
  double rad_dist = k1 * r2_distort + k2 * r4_distort;
  double ten_dist_x = 2 * p1 * xy_distort + p2 * (r2_distort + 2 * x2_distort);
  double ten_dist_y = p1 * (r2_distort + 2 * y2_distort) + 2 * p2 * xy_distort;
  double dist_x = (1 + rad_dist) * x_distort + ten_dist_x;
  double dist_y = (1 + rad_dist) * y_distort + ten_dist_y;
  double inv_denom =
      1 / (1 + 4 * k1 * r2_distort
           + 6 * k2 * r4_distort
           + 8 * p1 * y_distort
           + 8 * p2 * x_distort);

  double x_undistort = x_distort - inv_denom * dist_x;
  double y_undistort = y_distort - inv_denom * dist_y;

  cv::Point2d undistort_point(x_undistort, y_undistort);
  return undistort_point;
}

}  // namespace ultimate_msckf_vio
