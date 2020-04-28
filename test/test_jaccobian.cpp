#include "ultimate_msckf_vio/utility/geometric_kit.h"
#include "ultimate_msckf_vio/visual_ekf_update.h"

using Eigen::Quaterniond;
using Eigen::Vector3d;
using ultimate_msckf_vio::VectorToSkewSymmetricMatrix;

Eigen::Vector3d ComputeFeatureProjection(const Eigen::Isometry3d& T_G_C,
                                         const Vector3d& G_p_G_feature,
                                         const Eigen::Matrix3d& intrinsic_mat) {
  Eigen::Vector3d result_uz;
  Vector3d C_p_C_fea = T_G_C.inverse() * G_p_G_feature;
  if (C_p_C_fea(2) <= 0) {
    LOG(ERROR) << "point can't be seen !!!!";
    return result_uz;
  }
  Eigen::Vector3d unnorm_xy = Eigen::Vector3d(C_p_C_fea(0) / C_p_C_fea(2),
                                              C_p_C_fea(1) / C_p_C_fea(2),
                                              1);
  result_uz = intrinsic_mat * unnorm_xy;
  return result_uz;
}



int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "test geometry ";

  LOG(WARNING) << "-------------- delta rotation test --------------";
  // d_R = I + [dth]^
  double dthx = 0.001;
  double dthy = 0.002;
  double dthz = 0.003;
  Quaterniond delta_q(1, dthx/2, dthy/2, dthz/2);
  delta_q.normalize();

//  double dx = 0.04;
//  double dy = 0.05;
//  double dz = 0.06;
  double dx = 0.00;
  double dy = 0.00;
  double dz = 0.00;
  Vector3d delta_p(dx, dy, dz);

  // test of jaccobian evaluation:
  // 1) A = disteurbed_obsv - original_obsv
  // 2) B = dx * jaccob
  // 3) check A == B ?

  // in this test , focal length = 0.01; feature = (0, 0, 2),
  // C : camera     G : global     fea : feature
  //


  // test 0
  double focal_length = 0.01;
  Eigen::Quaterniond q_I_C(1, 0, 0, 0);
  Eigen::Vector3d p_I_C(0, 0, 0);


  Vector3d G_p_G_feature = Vector3d(0, 0, 2);
  Quaterniond q_G_C(1, 0, 0, 0);
  Vector3d G_p_G_C(0, 0, 0);
  Eigen::Isometry3d T_G_C;
  T_G_C.setIdentity();
  T_G_C.matrix().block(0,0,3,3) = q_G_C.normalized().toRotationMatrix();
  T_G_C.translate(G_p_G_C);
  LOG(INFO) << "orgin T_G_C \n" << T_G_C.matrix();

  Eigen::Matrix3d intrinsic_mat;
  intrinsic_mat << focal_length, 0, 0,
                   0, focal_length, 0,
                   0, 0, 1;

  // origin
  Vector3d C_p_C_fea = T_G_C.inverse() * G_p_G_feature;
  Eigen::Vector3d before_uz =
      ComputeFeatureProjection(T_G_C, C_p_C_fea, intrinsic_mat);
  LOG(INFO) << "before uz: " << before_uz.transpose();

  // after
  Quaterniond q_C0_C1 = delta_q;
  Quaterniond q_G_C1 = q_G_C * q_C0_C1;
  Eigen::Isometry3d T_G_C1;
  T_G_C1.setIdentity();
  T_G_C1.matrix().block(0,0,3,3) = q_G_C1.normalized().toRotationMatrix();
  T_G_C1.translate(G_p_G_C);

  Eigen::Vector3d after_uz =
      ComputeFeatureProjection(T_G_C1, G_p_G_feature, intrinsic_mat);

  LOG(INFO) << "after uz: " << after_uz.transpose();

  Eigen::Vector3d delta_uz = after_uz - before_uz;


  ultimate_msckf_vio::VisualEkfUpdate ekf_handle;
  ekf_handle.LoadIntrinsicMatrix(intrinsic_mat);
  //
  int feature_id  = 1;
  int keyframe_id = 2;
  ultimate_msckf_vio::FeatureBundle feature_bundle(feature_id,
                                                   keyframe_id,
                                                   Eigen::Vector2d(0, 0),
                                                   G_p_G_feature,
                                                   nullptr);
  std::vector<ultimate_msckf_vio::FeatureBundle> feature_bundles;
  feature_bundles.push_back(feature_bundle);
  ekf_handle.AddVisualConstraints(&feature_bundles);

  // new a fake ekf state which have one keyframe
  std::deque<ultimate_msckf_vio::KeyFrameState<double>> tmp_keyframes;
  tmp_keyframes.emplace_back(keyframe_id, q_G_C, G_p_G_C);

  Eigen::Vector3d tmp_bg(0, 0, 0);
  Eigen::Vector3d tmp_ba(0, 0, 0);
  Eigen::Vector3d tmp_v(0, 0, 0);
  auto tmp_imu_state = ultimate_msckf_vio::ImuState<double>(q_G_C, tmp_bg,
                                                            tmp_v, tmp_ba,
                                                            G_p_G_C);
  auto tmp_calib_state =
      ultimate_msckf_vio::CalibrationState<double>(q_I_C, p_I_C);

  ultimate_msckf_vio::EkfState<double> ekf_state(tmp_imu_state,
                                                 tmp_calib_state,
                                                 tmp_keyframes);

  Eigen::MatrixXd H;
  Eigen::MatrixXd res;

  ekf_handle.EvaluateJaccobianAndResidualSingleFeature(
        std::make_shared<ultimate_msckf_vio::EkfState<double>>(ekf_state),
        feature_bundle,
        &H, &res);
  LOG(INFO) << "H " << H;
  LOG(INFO) << "res " << res;


  return 0;
}
