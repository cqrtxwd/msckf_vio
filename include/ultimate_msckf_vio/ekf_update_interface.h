#ifndef EKF_UPDATE_INTERFACE_H_
#define EKF_UPDATE_INTERFACE_H_

#include "eigen3/Eigen/Eigen"
#include "ros/ros.h"

namespace ultimate_msckf_vio {

using Eigen::Vector2d;
using std::deque;
using std::map;
using std::vector;

class EkfUpdateInterface {
 public:
  virtual bool ComputeObservationMatrixAndResidual();

  virtual bool ComputeKalmanGain();

  virtual bool UpdateState();

  virtual bool UpdateCovariance();

  virtual bool UpdateCovariance();

  bool IEKFUpdate();



};


}






#endif // EKF_UPDATE_INTERFACE_H_
