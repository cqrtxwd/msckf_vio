#include "ultimate_msckf_vio/visual_ekf_update.h"

namespace ultimate_msckf_vio {

VisualEkfUpdate::VisualEkfUpdate() {}



//bool EkfUpdate::IEKFUpdate(EkfStated *ekf_state) {
//  LOG(INFO) << "IEKFUpdate calls ...";
//  // iekf update procedure follows the steps below
//  // 1) ComputeObservationMatrixAndResidual
//  // 2) compute kalman gain
//  // 3) update state using kalman gain
//  // 4) loop these 3 steps above iterately, untill the break condition satisfied
//  // 5) finally, update covariance

//  // iekf loop
//  while (true) {
//    MatrixXd H, residual;
//    EvaluateJaccobianAndResidual(ekf_state, &H, &residual);
//  }



//}

bool VisualEkfUpdate::EvaluateJaccobianAndResidual(EkfStated* ekf_state,
                                                    Eigen::MatrixXd* H,
                                                    Eigen::MatrixXd* residual) {
  CHECK(H != nullptr);
  CHECK(residual != nullptr);
  for (auto feature_bundle : visual_bundle_constraints_) {
    MatrixXd H_single;
    MatrixXd res_single;
    EvaluateJaccobianAndResidualSingleFeature(ekf_state,
                                              &H_single,
                                              &res_single);
  }
  return true;
}

bool VisualEkfUpdate::EvaluateJaccobianAndResidualSingleFeature(
    EkfStated *ekf_state,
    Eigen::MatrixXd *H,
    Eigen::MatrixXd *residual) {
  // when compute H and residual of a single feature observation, we follow
  // the steps below
  // 1) trianglate the 3d point
  // 2) compute H_state and H_feature, and residual
  // 3) Marginalize out the 3d features position from state, also known as
  //    "project H_state to H_feature's left-null space"
  // 4) now we get the H and residual of this feature bundle


  return true;
}


void VisualEkfUpdate::AddVisualConstraints(
    vector<FeatureBundle>* visual_bundle_constraints) {
  visual_bundle_constraints_.swap(*visual_bundle_constraints);
}











}
