#ifndef EKF_STATE_H_
#define EKF_STATE_H_

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>

namespace ultimate_msckf_vio {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaternion;
using Eigen::Vector3d;
using std::vector;

constexpr int kQStateIndex = 0;
constexpr int kBgStateIndex = 4 + kQStateIndex;
constexpr int kVStateIndex = 3 + kBgStateIndex;
constexpr int kBaStateIndex = 3 + kVStateIndex;
constexpr int kPStateIndex = 3 + kBaStateIndex;

constexpr int kCalibQStateIndex = kPStateIndex + 3;
constexpr int kCalibPStateIndex = kCalibQStateIndex + 4;

constexpr int kQErrorStateIndex = 0;
constexpr int kBgErrorStateIndex = 3 + kQErrorStateIndex; // 3
constexpr int kVErrorStateIndex = 3 + kBgErrorStateIndex; // 6
constexpr int kBaErrorStateIndex = 3 + kVErrorStateIndex; // 9
constexpr int kPErrorStateIndex = 3 + kBaErrorStateIndex; // 12

constexpr int kCalibQErrorStateIndex = kPErrorStateIndex + 3; // 15
constexpr int kCalibPErrorStateIndex = kCalibQErrorStateIndex + 3; // 18

constexpr int kImuStateSize = 16;
constexpr int kImuErrorStateSize = 15;

constexpr int kCalibStateSize = 7;
constexpr int kCalibErrorStateSize = 6;

template <typename Scalar>
class ImuState {
 public:
  ImuState() {}

  Quaternion<Scalar> I_q_G() const { return I_q_G_; }

  Matrix<Scalar, 3, 1> bg() const{ return bg_; }

  Matrix<Scalar, 3, 1> G_v_I() const { return G_v_I_; }

  Matrix<Scalar, 3, 1> ba() const { return ba_; }

  Matrix<Scalar, 3, 1> G_p_I() const { return G_p_I_; }

  static int StateSize() { return 16; }
  static int ErrorStateSize() { return 15; }

  Matrix<Scalar, 16, 1> GetImuStateVector() {
    Matrix<Scalar, 16, 1> imu_state_vector;
    imu_state_vector << I_q_G_.coeffs(), bg_, G_v_I_, ba_, G_p_I_;
    return imu_state_vector;
  }

  void SetImuStateVector(const Matrix<Scalar, 16, 1>& state_vector) {
    I_q_G_ = Quaternion<Scalar>(state_vector.head(4).data());
    bg_ = state_vector.block(kBgStateIndex, 0, 3, 1);
    G_v_I_ = state_vector.block(kVStateIndex, 0, 3, 1);
    ba_ = state_vector.block(kBaStateIndex, 0, 3, 1);
    G_p_I_ = state_vector.block(kPStateIndex, 0, 3, 1);
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

  static int StateSize() { return 7; }
  static int ErrorStateSize() { return 6; }
};

template <typename Scalar>
class KeyFrameState {
 public:
  KeyFrameState(const ImuState<Scalar>& imu_state):
    I_q_G(imu_state.I_q_G()),
    G_p_I(imu_state.G_p_I()) {}
  Quaternion<Scalar> I_q_G;
  Matrix<Scalar, 3, 1> G_p_I;

  static int StateSize() { return 7; }
  static int ErrorStateSize() { return 6; }
};


template <typename Scalar>
class EkfState {
 public:
  EkfState() : last_update_time_(0) {
    covariance.resize(kImuErrorStateSize + kCalibErrorStateSize,
                      kImuErrorStateSize + kCalibErrorStateSize);
    covariance.setIdentity(kImuErrorStateSize + kCalibErrorStateSize,
                           kImuErrorStateSize + kCalibErrorStateSize);
  }

  ImuState<Scalar> imu_state;

  CalibrationState<Scalar> calibration_state;

  vector<KeyFrameState<Scalar>> keyframe_states;

  vector<Matrix<Scalar, 3, 1>> landmarks;

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> covariance;
//  Eigen::MatrixXd covariance;

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

  void SetCovariance() {
    // for now, just for debug
    Matrix<Scalar, 3, 3> cov_1 = Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_2 = 2 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_3 = 3 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_4 = 4 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_5 = 5 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_6 = 6 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_7 = 7 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_8 = 8 * Matrix<Scalar, 3, 3>::Identity();
    Matrix<Scalar, 3, 3> cov_9 = 9 * Matrix<Scalar, 3, 3>::Identity();
    ROS_INFO_STREAM("cov size: "<< covariance.cols(););
    covariance
     << cov_9, cov_2, cov_3, cov_4, cov_5, cov_6, cov_7,
        cov_2, cov_1, cov_8, cov_1, cov_1, cov_1, cov_1,
        cov_3, cov_8, cov_2, cov_1, cov_1, cov_1, cov_1,
        cov_4, cov_1, cov_1, cov_3, cov_1, cov_1, cov_1,
        cov_5, cov_1, cov_1, cov_1, cov_4, cov_1, cov_1,
        cov_6, cov_1, cov_1, cov_1, cov_1, cov_5, cov_1,
        cov_7, cov_1, cov_1, cov_1, cov_1, cov_1, cov_6;


  }

  Matrix<Scalar, kImuErrorStateSize, kImuErrorStateSize>
  GetImuStateCovariance() {
    return covariance.block(0, 0, kImuErrorStateSize, kImuErrorStateSize);
  }

  void AugmentStateAndCovariance() {
    ROS_INFO_STREAM("AugmentStateAndCovariance");
    // TODO: add info to KF manager
    // ROS_INFO_STREAM("before augment cov: \n"<< covariance;);

//    ROS_INFO_STREAM("new_key_frame: " << new_key_frame.G_p_I ;);
    // matrix block operation
    /*
     * old cov:
     * | [Imu]    a'         b'       |
     * |   a   [Calib]       c'       |
     * |               [KF1        ]  |
     * |   b      c    [    KF2    ]  |
     * |               [        KF3]  |
     *
     * new cov after add a new KF:
     *
     * | [Imu]   a'        b'         I    |
     * |   a  [Calib]      c'         a    |
     * |             [KF1        ]         |
     * |   b     c   [    KF2    ]    b    |
     * |             [        KF3]         |
     * |   I     a'       b'      [new_KF] |
     *
     */
    // copy new KF cov
    Matrix<Scalar, 6, 6> new_KF_cov;
    new_KF_cov.block(0, 0, 3, 3) =
        covariance.block(kQErrorStateIndex, kQErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 3, 3, 3) =
        covariance.block(kPErrorStateIndex, kPErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 0, 3, 3) =
        covariance.block(kPErrorStateIndex, kQErrorStateIndex, 3, 3);
    new_KF_cov.block(0, 3, 3, 3) =
        covariance.block(kQErrorStateIndex, kPErrorStateIndex, 3, 3);

    // copy new_KF to calib and old_KFs cov
    Matrix<Scalar, Eigen::Dynamic, 6> new_KF_to_calib_old_KFs_cov;
    int KFs_size = keyframe_states.size();
    new_KF_to_calib_old_KFs_cov.resize(6 * KFs_size + kCalibErrorStateSize, 6);
    new_KF_to_calib_old_KFs_cov.block(0, 0,
                                      6 * KFs_size + kCalibErrorStateSize, 3) =
        covariance.block(kCalibQErrorStateIndex,
                         kQErrorStateIndex,
                         6 * KFs_size + kCalibErrorStateSize, 3);
    new_KF_to_calib_old_KFs_cov.block(0, 3,
                                      6 * KFs_size + kCalibErrorStateSize, 3) =
        covariance.block(kCalibQErrorStateIndex,
                         kPErrorStateIndex,
                         6 * KFs_size + kCalibErrorStateSize, 3);

    // copy imu to new KF cov
    Matrix<Scalar, kImuErrorStateSize, 6> new_KF_to_imu_cov;
    new_KF_to_imu_cov.setZero(kImuErrorStateSize, 6);
    new_KF_to_imu_cov.block(kQErrorStateIndex, 0, kImuErrorStateSize, 3) =
        covariance.block(kQErrorStateIndex, kQErrorStateIndex,
                         kImuErrorStateSize, 3);
    new_KF_to_imu_cov.block(kQErrorStateIndex, 3, kImuErrorStateSize, 3) =
        covariance.block(kQErrorStateIndex, kPErrorStateIndex,
                         kImuErrorStateSize, 3);

    // add new cov_block to new augmented cov
    int new_KF_insert_index = covariance.cols();
    covariance.conservativeResize(new_KF_insert_index + 6,
                                  new_KF_insert_index + 6);
    covariance.block(new_KF_insert_index, new_KF_insert_index, 6, 6) =
        new_KF_cov;
    covariance.block(kCalibQErrorStateIndex,
                     new_KF_insert_index,
                     6 * KFs_size + kCalibErrorStateSize, 6)
        = new_KF_to_calib_old_KFs_cov;
    covariance.block(new_KF_insert_index,
                     kCalibQErrorStateIndex,
                     6, 6 * KFs_size + kCalibErrorStateSize)
        = new_KF_to_calib_old_KFs_cov.transpose();

    covariance.block(0, new_KF_insert_index,
                     kImuErrorStateSize, 6) = new_KF_to_imu_cov;
    covariance.block(new_KF_insert_index, 0,
                     6, kImuErrorStateSize) = new_KF_to_imu_cov.transpose();

    // after augment cov, add new KF to KF state
    KeyFrameState<Scalar> new_key_frame(imu_state);
    keyframe_states.push_back(new_key_frame);
//    ROS_INFO_STREAM("after augment cov: \n"<< covariance;);
  }


  bool MarginalizeOldestKeyframe() {
    // todo :
    return true;
  }

  bool MarginalizeKeyframeByIndex(int kf_index) {
    // todo:
    return true;
  }

  bool MarginalizeLandmarkByIndex(int landmark_index) {
    // todo
    return true;
  }


 private:
  double last_update_time_;
};

typedef EkfState<double> EkfStated;

}  // namespace ultimate_msckf_vio

#endif  // EKF_STATE_H_
