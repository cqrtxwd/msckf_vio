#ifndef EKF_STATE_H_
#define EKF_STATE_H_

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <glog/logging.h>

namespace ultimate_msckf_vio {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Quaternion;
using Eigen::Vector3d;
using std::vector;

// state size :
constexpr int kVectorStateSize = 3;
constexpr int kQuaterionStateSize = 4;
constexpr int kVectorErrorStateSize = 3;
constexpr int kQuaterionErrorStateSize = 3;

constexpr int kImuStateSize = kQuaterionStateSize + 4 * kVectorStateSize; // 16
constexpr int kImuErrorStateSize = kQuaterionErrorStateSize + 4 * kVectorErrorStateSize; // 15

constexpr int kCalibStateSize = kQuaterionStateSize + kVectorStateSize;
constexpr int kCalibErrorStateSize = kVectorStateSize + kVectorStateSize;

constexpr int kKeyframeStateSize = kQuaterionStateSize + kVectorStateSize;
constexpr int kKeyframeErrorStateSize = kQuaterionErrorStateSize + kVectorStateSize;

// state index :
constexpr int kQStateIndex = 0;
constexpr int kBgStateIndex = kQuaterionStateSize + kQStateIndex;
constexpr int kVStateIndex = kVectorStateSize + kBgStateIndex;
constexpr int kBaStateIndex = kVectorStateSize + kVStateIndex;
constexpr int kPStateIndex = kVectorStateSize + kBaStateIndex;

constexpr int kCalibQStateIndex = kPStateIndex + kVectorStateSize;
constexpr int kCalibPStateIndex = kCalibQStateIndex + kQuaterionStateSize;

constexpr int kQErrorStateIndex = 0;
constexpr int kBgErrorStateIndex = kQuaterionErrorStateSize + kQErrorStateIndex; // 3
constexpr int kVErrorStateIndex = kVectorErrorStateSize + kBgErrorStateIndex; // 6
constexpr int kBaErrorStateIndex = kVectorErrorStateSize + kVErrorStateIndex; // 9
constexpr int kPErrorStateIndex = kVectorErrorStateSize + kBaErrorStateIndex; // 12

constexpr int kCalibQErrorStateIndex = kPErrorStateIndex + kVectorStateSize; // 15
constexpr int kCalibPErrorStateIndex = kCalibQErrorStateIndex + kVectorStateSize; // 18


/*
 * imu state sequence : [q, bg, v, ba, p]
 */
template <typename Scalar>
class ImuState {
 public:
  ImuState() {}

  Quaternion<Scalar> q_G_I() const { return q_G_I_; }

  Matrix<Scalar, 3, 1> bg() const{ return bg_; }

  Matrix<Scalar, 3, 1> v_G_I() const { return v_G_I_; }

  Matrix<Scalar, 3, 1> ba() const { return ba_; }

  Matrix<Scalar, 3, 1> p_G_I() const { return p_G_I_; }

  static int StateSize() { return kImuStateSize; }
  static int ErrorStateSize() { return kImuErrorStateSize; }

  Matrix<Scalar, 16, 1> GetImuStateVector() const {
    Matrix<Scalar, 16, 1> imu_state_vector;
    imu_state_vector << q_G_I_.coeffs(), bg_, v_G_I_, ba_, p_G_I_;
    return imu_state_vector;
  }

  void SetImuStateFromVector(const Matrix<Scalar, 16, 1>& state_vector) {
    q_G_I_ = Quaternion<Scalar>(state_vector.head(4).data());
    bg_ = state_vector.block(kBgStateIndex, 0, 3, 1);
    v_G_I_ = state_vector.block(kVStateIndex, 0, 3, 1);
    ba_ = state_vector.block(kBaStateIndex, 0, 3, 1);
    p_G_I_ = state_vector.block(kPStateIndex, 0, 3, 1);
  }

  void PrintInfo() {
    LOG(INFO) << "Imu state: ";
    LOG(INFO) << "q_G_I: " << q_G_I_.coeffs().transpose();
    LOG(INFO) << "bg: " << bg_.transpose();
    LOG(INFO) << "v_G_I: " << v_G_I_.transpose();
    LOG(INFO) << "ba: " << ba_.transpose();
    LOG(INFO) << "p_G_I: " << p_G_I_.transpose();
  }

 private:
  Quaternion<Scalar> q_G_I_;

  Matrix<Scalar, 3, 1> bg_;

  Matrix<Scalar, 3, 1> v_G_I_;

  Matrix<Scalar, 3, 1> ba_;

  Matrix<Scalar, 3, 1> p_G_I_;
};


template <typename Scalar>
class CalibrationState {
 public:
  CalibrationState() {}
  CalibrationState(const Quaternion<Scalar>& q_G_I,
                   const Matrix<Scalar, 3, 1>& p_G_I)
    :q_G_I_(q_G_I),
     p_G_I_(p_G_I) {}
  Quaternion<Scalar> q_G_I() const { return q_G_I_;}
  Matrix<Scalar, 3, 1> p_G_I() const { return p_G_I_;}

  static int StateSize() { return kCalibStateSize;}
  static int ErrorStateSize() { return kCalibErrorStateSize;}

  void SetState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& p_G_I) {
    q_G_I_ = q_G_I;
    p_G_I_ = p_G_I;
  }

  void SetZeroState() {
    SetState(Quaternion<Scalar>(1, 0, 0, 0),
             Matrix<Scalar, 3, 1>::Zero());
  }

  void PrintInfo () {
    LOG(INFO) << "Calibration state: ";
    LOG(INFO) << "q_G_I: " << q_G_I_.coeffs().transpose();
    LOG(INFO) << "p_G_I: " << p_G_I_.transpose();
  }
 private:
  Quaternion<Scalar> q_G_I_;
  Matrix<Scalar, 3, 1> p_G_I_;
};

template <typename Scalar>
class KeyFrameState {
 public:
  KeyFrameState() {}
  KeyFrameState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& p_G_I):
    q_G_I_(q_G_I),
    p_G_I_(p_G_I) {}

  static int StateSize() { return kKeyframeStateSize;}
  static int ErrorStateSize() { return kKeyframeErrorStateSize;}

  void PrintInfo () {
    LOG(INFO) << "keyframe state: ";
    LOG(INFO) << "q_G_I: " << q_G_I_.coeffs().transpose();
    LOG(INFO) << "p_G_I: " << p_G_I_.transpose();
  }

 private:
  Quaternion<Scalar> q_G_I_;
  Matrix<Scalar, 3, 1> p_G_I_;
};

template <typename Scalar>
class EkfState {
 public:
  EkfState() : last_update_time_(double(0)) {
    covariance_.resize(kImuErrorStateSize + kCalibErrorStateSize,
                      kImuErrorStateSize + kCalibErrorStateSize);
    covariance_.setIdentity(kImuErrorStateSize + kCalibErrorStateSize,
                           kImuErrorStateSize + kCalibErrorStateSize);
  }

  void PrintInfo() {
    imu_state_.PrintInfo();
    calibration_state_.PrintInfo();
    for (int i = 0; i < keyframe_states_.size(); i++) {
      LOG(INFO) << "keyframe "<< i;
      keyframe_states_[i].PrintInfo();
    }
  }

  void ClearState();

  int StateSize() {
    return imu_state_.StateSize() + calibration_state_.StateSize() +
           keyframe_states_.size() * KeyFrameState<Scalar>::StateSize() +
          landmarks_.size() * 3;
  }

  int ErrorStateSize() {
    return imu_state_.ErrorStateSize() + calibration_state_.ErrorStateSize() +
           keyframe_states_.size() * 6 + landmarks_.size() * 3;
  }

  Matrix<Scalar, 16, 1> GetImuStateVector() {
    return imu_state_.GetImuStateVector();
  }

  void SetImuStateVector(const Matrix<Scalar, 16, 1>& state_vector) {
    imu_state_.SetImuStateFromVector(state_vector);
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
    LOG(INFO) << "cov size: "<< covariance_.cols();
    covariance_
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
    return covariance_.block(0, 0, kImuErrorStateSize, kImuErrorStateSize);
  }

  void AugmentStateAndCovariance() {
    LOG(INFO) << "AugmentStateAndCovariance";
    // TODO: add info to KF manager

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
        covariance_.block(kQErrorStateIndex, kQErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 3, 3, 3) =
        covariance_.block(kPErrorStateIndex, kPErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 0, 3, 3) =
        covariance_.block(kPErrorStateIndex, kQErrorStateIndex, 3, 3);
    new_KF_cov.block(0, 3, 3, 3) =
        covariance_.block(kQErrorStateIndex, kPErrorStateIndex, 3, 3);

    // copy new_KF to calib and old_KFs cov
    Matrix<Scalar, Eigen::Dynamic, 6> new_KF_to_calib_old_KFs_cov;
    int KFs_size = keyframe_states_.size();
    new_KF_to_calib_old_KFs_cov.resize(6 * KFs_size + kCalibErrorStateSize, 6);
    new_KF_to_calib_old_KFs_cov.block(0, 0,
                                      6 * KFs_size + kCalibErrorStateSize, 3) =
        covariance_.block(kCalibQErrorStateIndex,
                         kQErrorStateIndex,
                         6 * KFs_size + kCalibErrorStateSize, 3);
    new_KF_to_calib_old_KFs_cov.block(0, 3,
                                      6 * KFs_size + kCalibErrorStateSize, 3) =
        covariance_.block(kCalibQErrorStateIndex,
                         kPErrorStateIndex,
                         6 * KFs_size + kCalibErrorStateSize, 3);

    // copy imu to new KF cov
    Matrix<Scalar, kImuErrorStateSize, 6> new_KF_to_imu_cov;
    new_KF_to_imu_cov.setZero(kImuErrorStateSize, 6);
    new_KF_to_imu_cov.block(kQErrorStateIndex, 0, kImuErrorStateSize, 3) =
        covariance_.block(kQErrorStateIndex, kQErrorStateIndex,
                         kImuErrorStateSize, 3);
    new_KF_to_imu_cov.block(kQErrorStateIndex, 3, kImuErrorStateSize, 3) =
        covariance_.block(kQErrorStateIndex, kPErrorStateIndex,
                         kImuErrorStateSize, 3);

    // add new cov_block to new augmented cov
    int new_KF_insert_index = covariance_.cols();
    covariance_.conservativeResize(new_KF_insert_index + 6,
                                  new_KF_insert_index + 6);
    covariance_.block(new_KF_insert_index, new_KF_insert_index, 6, 6) =
        new_KF_cov;
    covariance_.block(kCalibQErrorStateIndex,
                     new_KF_insert_index,
                     6 * KFs_size + kCalibErrorStateSize, 6)
        = new_KF_to_calib_old_KFs_cov;
    covariance_.block(new_KF_insert_index,
                     kCalibQErrorStateIndex,
                     6, 6 * KFs_size + kCalibErrorStateSize)
        = new_KF_to_calib_old_KFs_cov.transpose();

    covariance_.block(0, new_KF_insert_index,
                     kImuErrorStateSize, 6) = new_KF_to_imu_cov;
    covariance_.block(new_KF_insert_index, 0,
                     6, kImuErrorStateSize) = new_KF_to_imu_cov.transpose();

    // after augment cov, add new KF to KF state
    KeyFrameState<Scalar> new_key_frame(imu_state_.q_G_I(), imu_state_.p_G_I());
    keyframe_states_.push_back(new_key_frame);
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

  ImuState<Scalar> imu_state() const {
    return imu_state_;
  }

  CalibrationState<Scalar> calibration_state() const {
    return calibration_state_;
  }

  vector<KeyFrameState<Scalar>> keyframe_states() const {
    return keyframe_states_;
  }

  vector<Matrix<Scalar, 3, 1>> landmarks() const {
    return landmarks_;
  }

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> covariance() const {
    return covariance_;
  }

 private:
  ImuState<Scalar> imu_state_;

  CalibrationState<Scalar> calibration_state_;

  vector<KeyFrameState<Scalar>> keyframe_states_;

  vector<Matrix<Scalar, 3, 1>> landmarks_;

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> covariance_;


  double last_update_time_;
};

typedef EkfState<double> EkfStated;

}  // namespace ultimate_msckf_vio

#endif  // EKF_STATE_H_
