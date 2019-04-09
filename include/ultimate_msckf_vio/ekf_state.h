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
class ImuState {
 public:
  ImuState() {}

  Quaternion<Scalar> I_q_G() {
    return I_q_G_;
  }

  Matrix<Scalar, 3, 1> bg() {
    return bg_;
  }

  Matrix<Scalar, 3, 1> G_v_I() {
    return G_v_I_;
  }

  Matrix<Scalar, 3, 1> ba() {
    return ba_;
  }

  Matrix<Scalar, 3, 1> G_p_I() {
    return G_p_I_;
  }

  int StateSize() {
    return 16;
  }
  int ErrorStateSize() {
    return 15;
  }

 private:
  Quaternion<Scalar> I_q_G_;

  Matrix<Scalar, 3, 1> bg_;

  Matrix<Scalar, 3, 1> G_v_I_;

  Matrix<Scalar, 3, 1> ba_;

  Matrix<Scalar, 3, 1> G_p_I_;

};

template <typename Scalar>
class CalibrationState {
 public:
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;

  int StateSize(){
    return 7;
  }
  int ErrorStateSize() {
    return 6;
  }
};

template <typename Scalar>
class KeyFrameState {
 public:
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;

  int StateSize() {
    return 7;
  }
  int ErrorStateSize() {
    return 6;
  }
};

template <typename Scalar>
class EkfState {
 public:
  EkfState(): last_update_time_(0) {}

  ImuState<Scalar> imu_state;

  CalibrationState<Scalar> calibration_state;

  vector<KeyFrameState<Scalar>> keyframe_states;

  vector<Matrix<Scalar, 3, 1>> landmarks;

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> covarience;

  void ClearState();

  int StateSize() {
    return imu_state.StateSize()
        + calibration_state.StateSize()
        + keyframe_states.size() * 7
        + landmarks.size() * 3;
  }

  int ErrorStateSize() {
    return imu_state.ErrorStateSize()
        + calibration_state.ErrorStateSize()
        + keyframe_states.size() * 6
        + landmarks.size() * 3;
  }

 private:
   double last_update_time_;

};


}


#endif // EKF_STATE_H_
