#ifndef NUMERIC_INTEGRATOR_H
#define NUMERIC_INTEGRATOR_H

#include <eigen3/Eigen/Geometry>
#include "ultimate_msckf_vio/ekf_state.h"

using Eigen::Matrix;
using Eigen::Quaternion;

namespace ultimate_msckf_vio {
template <typename Scalar>
class Integrator {
public:
  Integrator() {}
};


template <typename Scalar>
class RungeKuttaIntegrator {
 public:
  RungeKuttaIntegrator() {}

 private:
  Matrix<double, 3, 1> accel_global;
  Matrix<double, 3, 1> gyro_global;
  EkfState<Scalar> ekf_state_;

};

}











#endif // NUMERIC_INTEGRATOR_H
