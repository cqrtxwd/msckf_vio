#include "ultimate_msckf_vio/visual_ekf_update.h"
#include "ultimate_msckf_vio/utility/geometric_kit.h"

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

bool VisualEkfUpdate::EvaluateJaccobianAndResidual(
    shared_ptr<EkfStated> ekf_state,
    Eigen::MatrixXd* H,
    Eigen::MatrixXd* residual) {
  CHECK(H != nullptr);
  CHECK(residual != nullptr);
  for (auto feature_bundle : visual_bundle_constraints_) {
    MatrixXd H_single;
    MatrixXd res_single;
    EvaluateJaccobianAndResidualSingleFeature(ekf_state,
                                              feature_bundle,
                                              &H_single,
                                              &res_single);
  }
  return true;
}

/*
 * when compute H and residual of a single feature observation, we follow
 * the steps below
 * 1) trianglate the 3d point
 * 2) compute H_state and H_feature, and residual
 * 3) Marginalize out the 3d features position from state, also known as
 *    "project H_state to H_feature's left-null space"
 * 4) now we get the H and residual of this feature bundle
 *  (G: gloabl frame;  C: camera frame)
 */

bool VisualEkfUpdate::EvaluateJaccobianAndResidualSingleFeature(
    shared_ptr<EkfStated> ekf_state,
    const FeatureBundle& feature_bundle,
    Eigen::MatrixXd *H,
    Eigen::MatrixXd *res) {

  // trianglate (todo)
  Vector3d feat_3d;
  Matrix3d intrinsic_matrix;


  // compute H_state and H_feature, and residual
  MatrixXd H_state;
  H_state.resize(2 * feature_bundle.NumObservedKeyframe(),
                 ekf_state->ErrorStateSize());
  MatrixXd H_feature;
  H_feature.resize(2 * feature_bundle.NumObservedKeyframe(), 3);
  Matrix<double, 2, 2> H_intrinsic = intrinsic_matrix.block(0, 0, 2, 2);

  // compute jaccobian for each keyframe observation
  for (size_t i = 0; i < feature_bundle.observed_keframes_id().size(); i++) {
    int keyframei_id = feature_bundle.observed_keframes_id()[i];
    int keyframei_index = ekf_state->GetKeyframeIndexById(keyframei_id);

    // compute H_feature, hint:
    // H_feature_C (camera frame) = H_intrinsic * H_project;
    // H_feature_G (global frame) = H_intrinsic * H_project * R_C_G
    Matrix<double, 2, 3> H_feature_i;
    Matrix<double, 2, 3> H_project_i;
    double z_square = feat_3d(2) * feat_3d(2);
    H_project_i << feat_3d(0), 0, feat_3d(0) / z_square,
                   0, feat_3d(1), feat_3d(1) / z_square;

    Matrix3d keyframei_R_C_G =
        ekf_state->keyframe_states()[keyframei_index].q_G_I()
        .toRotationMatrix().transpose();
    H_feature_i = H_intrinsic * H_project_i * keyframei_R_C_G;

    // compute H_state
    // H_state_i_p (position of i th keyframe)
    // H_state_i_p = - H_feature_G;
    Matrix<double, 2, 3> H_state_i_p = - H_feature_i;

    // H_state_i_q = H_intrinsic * H_project * [T_C_G * G_p_G_feat]^
    // aka: H_state_i_q = H_intrinsic * H_project * [C_p_C_feat]^
    Matrix<double, 2, 3> H_state_i_q;
    Vector3d C_p_C_feat = keyframei_R_C_G * feat_3d;
    H_state_i_q =
        H_intrinsic * H_project_i * VectorToSkewSymmetricMatrix(C_p_C_feat);
    Matrix<double, 2, 6> H_state_i;
    H_state_i << H_state_i_q, H_state_i_p;

    // insert jaccobian matrix block to H_state and H_feature
    int feat_insert_index = 2 * keyframei_index;
    H_feature.block(feat_insert_index, 0, 2, 3) = H_feature_i;
    int state_insert_row_index = feat_insert_index;
    int state_insert_col_index =
        ImuState<double>::ErrorStateSize()
        + CalibrationState<double>::ErrorStateSize()
        + keyframei_index * KeyFrameState<double>::ErrorStateSize();
    H_state.block(state_insert_row_index,
                  state_insert_col_index, 2, 6) = H_state_i;
  }

  // project H_state to the left-null space of H_feature, to marginalize the
  // feature state, compressing Jaccobian matrix in cols.
  // we use givens rotation to achieve that.









  return true;
}


void VisualEkfUpdate::AddVisualConstraints(
    vector<FeatureBundle>* visual_bundle_constraints) {
  visual_bundle_constraints_.swap(*visual_bundle_constraints);
}











}
