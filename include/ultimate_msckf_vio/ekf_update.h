#ifndef EKF_UPDATE_H
#define EKF_UPDATE_H

#include "glog/logging.h"
#include "eigen3/Eigen/Eigen"

#include "ultimate_msckf_vio/ekf_update_base.h"
#include "ultimate_msckf_vio/feature_bundle.h"
#include "ultimate_msckf_vio/ekf_state.h"


namespace ultimate_msckf_vio {
using std::vector;
using Eigen::Matrix;
using Eigen::MatrixXd;


class EkfUpdate : public EkfUpdateBase {
 public:

  bool ComputeObservationMatrixAndResidual(){}

  bool ComputeKalmanGain(){}

  bool UpdateState(){}

  bool UpdateCovariance(){}


  bool IEKFUpdate(EkfStated* ekf_state);

  void AddVisualConstraints(vector<FeatureBundle>* visual_bundle_constraints);

  private:
  vector<FeatureBundle> visual_bundle_constraints_;


};


}

#endif // EKF_UPDATE_H
