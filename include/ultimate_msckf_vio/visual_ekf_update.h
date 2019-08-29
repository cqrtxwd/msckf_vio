#ifndef VISUAL_EKF_UPDATE_H
#define VISUAL_EKF_UPDATE_H

#include "glog/logging.h"
#include "eigen3/Eigen/Eigen"

#include "ultimate_msckf_vio/ekf_update_base.h"
#include "ultimate_msckf_vio/feature_bundle.h"
#include "ultimate_msckf_vio/ekf_state.h"


namespace ultimate_msckf_vio {
using std::vector;
using Eigen::Matrix;
using Eigen::MatrixXd;


class VisualEkfUpdate : public EkfUpdateBase {
 public:
  VisualEkfUpdate();
  ~VisualEkfUpdate() {}

  bool IEKFUpdate(EkfStated* ekf_state);

  bool EvaluateJaccobianAndResidual(EkfStated *ekf_state,
                                    MatrixXd* H,
                                    MatrixXd* residual);

//  bool ComputeKalmanGain(){
//    // todo
//    return true;
//  }

//  bool UpdateState(){
//    // todo
//    return true;
//  }

//  bool UpdateCovariance(){
//    // todo
//    return true;
//  }

  bool EvaluateJaccobianAndResidualSingleFeature(EkfStated *ekf_state,
                                                        MatrixXd* H,
                                                        MatrixXd* residual);




  void AddVisualConstraints(vector<FeatureBundle>* visual_bundle_constraints);

  private:
  vector<FeatureBundle> visual_bundle_constraints_;


};


}

#endif // EKF_UPDATE_H