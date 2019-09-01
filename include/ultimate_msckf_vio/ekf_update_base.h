#ifndef EKF_UPDATE_BASE_H_
#define EKF_UPDATE_BASE_H_

#include "eigen3/Eigen/Eigen"
#include "ros/ros.h"
#include <deque>

#include "ultimate_msckf_vio/ekf_state.h"

namespace ultimate_msckf_vio {

using Eigen::Vector2d;
using std::deque;
using std::map;
using std::vector;
using Eigen::MatrixXd;

class EkfUpdateBase {
 public:
  EkfUpdateBase() {}
//   ~EkfUpdateBase();

  virtual bool EvaluateJaccobianAndResidual(
      std::shared_ptr<EkfStated> ekf_state,
      MatrixXd* H,
      MatrixXd* residual) = 0;

//  virtual bool ComputeKalmanGain() = 0;

//  virtual bool UpdateState() = 0;

//  virtual bool UpdateCovariance() = 0;


//  bool IEKFUpdate(EkfStated* ekf_state) {}
 private:


};


}






#endif // EKF_UPDATE_BASE_H_
