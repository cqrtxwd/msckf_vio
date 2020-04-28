#include "ultimate_msckf_vio/vio_initializer.h"

#include "ultimate_msckf_vio/data_manager.h"

#include "opencv2/core/eigen.hpp"


namespace ultimate_msckf_vio {

VIOInitializer::VIOInitializer(DataManager* data_manager_ptr)
  : is_initialized_(false) {
  CHECK(data_manager_ptr != nullptr) << "data manager haven't initialized";
    data_manager_.reset(data_manager_ptr);
}

bool VIOInitializer::is_initialized() {
  return is_initialized_;
}

bool VIOInitializer::TryInitialize() {
//  LOG(INFO) << "try initialize";
  auto num_frames = data_manager_->frames_.size();
  if (num_frames < kNumFramesToInitialize) {
    return false;
  }

  // find 2 frames has most matches, build initial point cloud
  std::vector<cv::DMatch> init_matches;
  int init_frame0 = -1, init_frame1 = -1;
  int max_match_num = 0;
  for (int i = 0; i < num_frames - 1; i++) {
    for (int j = i + 1; j < num_frames; j++) {
      std::vector<cv::DMatch> tmp_good_matches;
      data_manager_->feature_tracker_->
          FindFeatureMatchesBetweenFrames(i, j, &tmp_good_matches);
      if (tmp_good_matches.size() > max_match_num) {
        max_match_num = tmp_good_matches.size();
        init_frame0 = i;
        init_frame1 = j;
        init_matches = std::move(tmp_good_matches);
      }
    }
  }
  LOG(INFO) << "use frame "
            << init_frame0 << " and " << init_frame1
            <<  " to build initial point cloud, match num: "
            << init_matches.size();

  // build init point cloud
  // compute init pose of 2 init frame
  data_manager_->feature_tracker_->UndistortFrameKeyPoints(init_frame0);
  data_manager_->feature_tracker_->UndistortFrameKeyPoints(init_frame1);

  Pose3d relative_pose;
  SolveEpipolarGeometryOf2Frames(
        init_frame0, init_frame1, init_matches, &relative_pose);
  LOG(INFO) << "relative_pose " << relative_pose.rotation().coeffs() << " "
            << relative_pose.translation().transpose();

  // get init point cloud by triangulation
  std::vector<cv::Point2d> point0, point1;
  Frame frame0 = data_manager_->GetFrameById(init_frame0);
  Frame frame1 = data_manager_->GetFrameById(init_frame1);
  auto K = data_manager_->parameter_reader_->intrinsic_mat;
  for (auto match : init_matches) {
    cv::Point2d tmp_point0 = frame0.keypoints[match.queryIdx].pt;
    cv::Point2d tmp_point1 = frame1.keypoints[match.trainIdx].pt;
    point0.push_back(
          FeatureTracker::PixelToCam(tmp_point0, K));
    point1.push_back(
          FeatureTracker::PixelToCam(tmp_point1, K));
  }

  Eigen::Matrix<double, 3, 4> pose0_mat, pose1_mat;
  pose0_mat << Matrix3d::Identity(), Vector3d(0, 0, 0);
  pose1_mat << relative_pose.rotation().toRotationMatrix(),
               relative_pose.translation();
  cv::Mat pose0_cvmat, pose1_cvmat;
  cv::eigen2cv(pose0_mat, pose0_cvmat);
  cv::eigen2cv(pose1_mat, pose1_cvmat);

  cv::Mat points_3d;
  cv::triangulatePoints(pose0_cvmat, pose1_cvmat,
                        point0, point1,
                        points_3d);

  for (int i = 0; i < points_3d.cols; i++) {
    Eigen::Vector4d point;
    cv::cv2eigen(points_3d.col(i), point);
    point /= point(3);
    LOG(INFO) << "point3d " << point.transpose();
  }
  LOG(FATAL)<<"";


  // compute init frame poses by pnp

  is_initialized_ = true;
  return true;
}

bool VIOInitializer::SolveEpipolarGeometryOf2Frames(
    const Frame& frame0, const Frame& frame1, Pose3d* result_pose) {
  std::vector<cv::DMatch> feature_matches;
  data_manager_->feature_tracker_->FindFeatureMatchesBetweenFrames(
        frame0, frame1, &feature_matches);
  SolveEpipolarGeometryOf2Frames(frame0, frame1, feature_matches, result_pose);
  return true;
}

bool VIOInitializer::SolveEpipolarGeometryOf2Frames(
    const Frame& frame0,
    const Frame& frame1,
    const std::vector<cv::DMatch>& feature_matches,
    Pose3d* result_pose) {
  Eigen::Matrix3d K = data_manager_->parameter_reader_->intrinsic_mat;
  int focal_length = 460;
  cv::Point2d principle_point(K(0,2), K(1,2));
  LOG(INFO) << "principle_point " << K(0,2) << " " << K(1,2);
  // build match points
  std::vector<cv::Point2d> point0, point1;
  for(auto match : feature_matches) {
    point0.push_back(frame0.keypoints[match.queryIdx].pt);
    point1.push_back(frame1.keypoints[match.trainIdx].pt);
  }

  auto essential_mat = cv::findEssentialMat(point0, point1, focal_length,
                                            principle_point, cv::RANSAC);

  cv::Mat R_cv, t_cv;
  cv::recoverPose(essential_mat, point0, point1, R_cv, t_cv,
                  focal_length, principle_point);
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(t_cv, t);
  result_pose->SetRotation(R);
  result_pose->SetTranslation(t);
  return true;
}

bool VIOInitializer::SolveEpipolarGeometryOf2Frames(
    const int& frame0_id,
    const int& frame1_id,
    const std::vector<cv::DMatch>& feature_matches,
    Pose3d* result_pose) {
  auto& frame0 = data_manager_->GetFrameById(frame0_id);
  auto& frame1 = data_manager_->GetFrameById(frame1_id);
  SolveEpipolarGeometryOf2Frames(frame0, frame1, feature_matches, result_pose);
  return true;
}

}

