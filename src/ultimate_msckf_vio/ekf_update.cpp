#include "ultimate_msckf_vio/ekf_update.h"

namespace ultimate_msckf_vio {

bool EkfUpdate::IEKFUpdate(EkfStated *ekf_state) {
  LOG(INFO) << "IEKFUpdate calls ...";
  // iekf update procedure follows the steps below
  // 1) ComputeObservationMatrixAndResidual
  // 2) compute kalman gain
  // 3) update state using kalman gain
  // 4) loop these 3 steps above iterately, untill the break condition satisfied
  // 5) finally, update covariance

//  while (true) {
//    MatrixXd H, residual;
//    ComputeObservationMatrixAndResidual();
//  }



}

void EkfUpdate::AddVisualConstraints(
    vector<FeatureBundle>* visual_bundle_constraints) {
  visual_bundle_constraints_.swap(*visual_bundle_constraints);
}











}
