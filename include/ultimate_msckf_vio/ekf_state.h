#ifndef EKF_STATE_H_
#define EKF_STATE_H_

#include <eigen3/Eigen/Eigen>

namespace ultimate_msckf_vio {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaternion;
using Eigen::Vector3d;
using std::vector;

constexpr int kQIndex = 0;
constexpr int kBgIndex = 4 + kQIndex;
constexpr int kVIndex = 3 + kBgIndex;
constexpr int kBaIndex = 3 + kVIndex;
constexpr int kPIndex = 3 + kBaIndex;
constexpr int kImuStateSize = 16;
constexpr int kImuErrorStateSize = 15;

template <typename Scalar>
class ImuState {
 public:
  ImuState() {}

  Quaternion<Scalar> I_q_G() { return I_q_G_; }

  Matrix<Scalar, 3, 1> bg() { return bg_; }

  Matrix<Scalar, 3, 1> G_v_I() { return G_v_I_; }

  Matrix<Scalar, 3, 1> ba() { return ba_; }

  Matrix<Scalar, 3, 1> G_p_I() { return G_p_I_; }

  int StateSize() { return 16; }
  int ErrorStateSize() { return 15; }

  Matrix<Scalar, 16, 1> GetImuStateVector() {
    Matrix<Scalar, 16, 1> imu_state_vector;
    imu_state_vector << I_q_G_.coeffs(), bg_, G_v_I_, ba_, G_p_I_;
    return imu_state_vector;
  }

  void SetImuStateVector(const Matrix<Scalar, 16, 1>& state_vector) {
    I_q_G_ = Quaternion<Scalar>(state_vector.head(4).data());
    bg_ = state_vector.block(kBgIndex, 0, 3, 1);
    G_v_I_ = state_vector.block(kVIndex, 0, 3, 1);
    ba_ = state_vector.block(kBaIndex, 0, 3, 1);
    G_p_I_ = state_vector.block(kPIndex, 0, 3, 1);
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

  int StateSize() { return 7; }
  int ErrorStateSize() { return 6; }
};

template <typename Scalar>
class KeyFrameState {
 public:
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;

  int StateSize() { return 7; }
  int ErrorStateSize() { return 6; }
};

template <typename Scalar>
class EkfState {
 public:
  EkfState() : last_update_time_(0) {}

  ImuState<Scalar> imu_state;

  CalibrationState<Scalar> calibration_state;

  vector<KeyFrameState<Scalar>> keyframe_states;

  vector<Matrix<Scalar, 3, 1>> landmarks;

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> covariance;

  void ClearState();

  int StateSize() {
    return imu_state.StateSize() + calibration_state.StateSize() +
           keyframe_states.size() * 7 + landmarks.size() * 3;
  }

  int ErrorStateSize() {
    return imu_state.ErrorStateSize() + calibration_state.ErrorStateSize() +
           keyframe_states.size() * 6 + landmarks.size() * 3;
  }

  Matrix<Scalar, 16, 1> GetImuStateVector() {
    return imu_state.GetImuStateVector();
  }

  void SetImuStateVector(const Matrix<Scalar, 16, 1>& state_vector) {
    imu_state.SetImuStateVector(state_vector);
  }

  Matrix<Scalar, kImuErrorStateSize, kImuErrorStateSize>
  GetImuStateCovariance() {
    return covariance.block(0, 0, kImuErrorStateSize, kImuErrorStateSize);
  }

 private:
  double last_update_time_;
};

}  // namespace ultimate_msckf_vio

#endif  // EKF_STATE_H_
