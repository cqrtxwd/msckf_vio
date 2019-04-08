#ifndef EKF_STATE_H_
#define EKF_STATE_H_

#include <eigen3/Eigen/Eigen>

namespace ultimate_msckf_vio {
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaternion;
using std::vector;


template <typename Scalar>
class InertialState {
 public:
  InertialState() {}

  Quaternion<Scalar> I_q_G;

  Scalar bg;

  Matrix<Scalar, 3, 1> G_v_I;

  Scalar ba;

  Matrix<Scalar, 3, 1> G_p_I;

};

template <typename Scalar>
class CalibrationState {
 public:
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;
};

template <typename Scalar>
class KeyFrameState {
 public:
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;
};

template <typename Scalar>
class EkfState {
 public:
  EkfState():last_update_time_(0) {}

  InertialState<Scalar> inertial_state;
  CalibrationState<Scalar> calibration_state;
  KeyFrameState<Scalar> key_frame_state;

  vector<Matrix<Scalar, 3, 1>> land_marks;

  void ClearState();



 private:
   double last_update_time_;

};


}


#endif // EKF_STATE_H_
