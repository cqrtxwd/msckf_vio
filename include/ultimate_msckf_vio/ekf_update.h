#ifndef EKF_UPDATE_H
#define EKF_UPDATE_H

#include "ultimate_msckf_vio/ekf_update_base.h"


namespace ultimate_msckf_vio {
class EkfUpdate : public EkfUpdateBase {
 public:

  bool ComputeObservationMatrixAndResidual(){}

  bool ComputeKalmanGain(){}

  bool UpdateState(){}

  bool UpdateCovariance(){}


  bool IEKFUpdate(){}

};


}

#endif // EKF_UPDATE_H
