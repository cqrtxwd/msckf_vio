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

constexpr int kCalibrationStateSize = kQuaterionStateSize + kVectorStateSize;
constexpr int kCalibrationErrorStateSize = kVectorStateSize + kVectorStateSize;

constexpr int kKeyframeStateSize = kQuaterionStateSize + kVectorStateSize;
constexpr int kKeyframeErrorStateSize = kQuaterionErrorStateSize + kVectorStateSize;

// state index :
constexpr int kImuQStateIndex = 0;
constexpr int kImuBgStateIndex = kQuaterionStateSize + kImuQStateIndex;
constexpr int kImuVStateIndex = kVectorStateSize + kImuBgStateIndex;
constexpr int kImuBaStateIndex = kVectorStateSize + kImuVStateIndex;
constexpr int kImuPStateIndex = kVectorStateSize + kImuBaStateIndex;

constexpr int kCalibrationQStateIndex = kImuPStateIndex + kVectorStateSize;
constexpr int kCalibrationPStateIndex = kCalibrationQStateIndex + kQuaterionStateSize;

constexpr int kImuQErrorStateIndex = 0;
constexpr int kImuBgErrorStateIndex = kQuaterionErrorStateSize + kImuQErrorStateIndex; // 3
constexpr int kImuVErrorStateIndex = kVectorErrorStateSize + kImuBgErrorStateIndex; // 6
constexpr int kImuBaErrorStateIndex = kVectorErrorStateSize + kImuVErrorStateIndex; // 9
constexpr int kImuPErrorStateIndex = kVectorErrorStateSize + kImuBaErrorStateIndex; // 12

constexpr int kCalibQErrorStateIndex = kImuPErrorStateIndex + kVectorStateSize; // 15
constexpr int kCalibPErrorStateIndex = kCalibQErrorStateIndex + kVectorStateSize; // 18

constexpr int kImuStateIndex = 0;
constexpr int kCalibrationStateIndex = kImuStateIndex + kImuStateSize; // 16
constexpr int kKeyframeStateIndex = kCalibrationStateIndex + kCalibrationStateSize; //23

constexpr int kImuErrorStateIndex = 0;
constexpr int kCalibrationErrorStateIndex = kImuErrorStateIndex + kImuErrorStateSize; // 16
constexpr int kKeyframeErrorStateIndex = kCalibrationErrorStateIndex + kCalibrationErrorStateSize; //23
/*
 * imu state sequence : [q, bg, v, ba, p]
 */
template <typename Scalar>
class ImuState {
 public:
  ImuState():
    q_G_I_(1, 0, 0 ,0),
    bg_(0, 0, 0),
    v_G_I_(0, 0, 0),
    ba_(0, 0, 0),
    p_G_I_(0, 0, 0) {}

  Quaternion<Scalar> q_G_I() const { return q_G_I_; }

  Matrix<Scalar, 3, 1> bg() const{ return bg_; }

  Matrix<Scalar, 3, 1> v_G_I() const { return v_G_I_; }

  Matrix<Scalar, 3, 1> ba() const { return ba_; }

  Matrix<Scalar, 3, 1> p_G_I() const { return p_G_I_; }

  static int StateSize() { return kImuStateSize; }
  static int ErrorStateSize() { return kImuErrorStateSize; }

  void SetState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& bg,
                const Matrix<Scalar, 3, 1>& v_G_I,
                const Matrix<Scalar, 3, 1>& ba,
                const Matrix<Scalar, 3, 1>& p_G_I) {
    q_G_I_ = q_G_I;
    bg_ = bg,
    v_G_I_ = v_G_I,
    ba_ = ba,
    p_G_I_ = p_G_I;
  }

  void SetZeroState() {
    SetState(Quaternion<Scalar>(1, 0, 0, 0),
             Matrix<Scalar, 3, 1>::Zero(),
             Matrix<Scalar, 3, 1>::Zero(),
             Matrix<Scalar, 3, 1>::Zero(),
             Matrix<Scalar, 3, 1>::Zero());
  }

  Matrix<Scalar, 16, 1> GetImuStateVector() const {
    Matrix<Scalar, 16, 1> imu_state_vector;
    imu_state_vector << q_G_I_.coeffs(), bg_, v_G_I_, ba_, p_G_I_;
    return imu_state_vector;
  }

  // quaternion [x ,y ,z ,w]
  void SetImuStateFromVector(const Matrix<Scalar, Eigen::Dynamic, 1>& state_vector) {
    CHECK(state_vector.rows() == kImuStateSize) << "input imu state invalid !!!";
    CHECK(state_vector.cols() == 1) << "input imu state invalid !!!";
    q_G_I_ = Quaternion<Scalar>(state_vector.head(4).data());
    q_G_I_.normalize();
    bg_ = state_vector.block(kImuBgStateIndex, 0, 3, 1);
    v_G_I_ = state_vector.block(kImuVStateIndex, 0, 3, 1);
    ba_ = state_vector.block(kImuBaStateIndex, 0, 3, 1);
    p_G_I_ = state_vector.block(kImuPStateIndex, 0, 3, 1);
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
  CalibrationState():
    q_G_I_(1, 0, 0, 0),
    p_G_I_(0, 0, 0) {}
  CalibrationState(const Quaternion<Scalar>& q_G_I,
                   const Matrix<Scalar, 3, 1>& p_G_I)
    :q_G_I_(q_G_I),
     p_G_I_(p_G_I) {}
  Quaternion<Scalar> q_G_I() const { return q_G_I_;}
  Matrix<Scalar, 3, 1> p_G_I() const { return p_G_I_;}

  static int StateSize() { return kCalibrationStateSize;}
  static int ErrorStateSize() { return kCalibrationErrorStateSize;}

  void SetState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& p_G_I) {
    q_G_I_ = q_G_I;
    p_G_I_ = p_G_I;
  }

  void SetZeroState() {
    SetState(Quaternion<Scalar>(1, 0, 0, 0),
             Matrix<Scalar, 3, 1>::Zero());
  }

  Matrix<Scalar, kCalibrationStateSize, 1> GetCalibrationStateVector() const {
    Matrix<Scalar, kCalibrationStateSize, 1> calib_state_vector;
    calib_state_vector << q_G_I_.coeffs(), p_G_I_;
    return calib_state_vector;
  }

  // quaternion [x ,y ,z ,w]
  void SetCalibrationStateFromVector(
      const Matrix<Scalar, kCalibrationStateSize, 1>& state_vector) {
    CHECK(state_vector.rows() == kCalibrationStateSize)
        << "input calibration state invalid !!!";
    CHECK(state_vector.cols() == 1) << "input calibration state invalid !!!";
    q_G_I_ = Quaternion<Scalar>(state_vector.head(4).data());
    q_G_I_.normalize();
    p_G_I_ = state_vector.block(kQuaterionStateSize, 0, 3, 1);
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
  KeyFrameState():
    q_G_I_(1, 0, 0, 0),
    p_G_I_(0, 0, 0) {}
  KeyFrameState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& p_G_I):
    q_G_I_(q_G_I),
    p_G_I_(p_G_I) {}

  static int StateSize() { return kKeyframeStateSize;}
  static int ErrorStateSize() { return kKeyframeErrorStateSize;}

  void SetState(const Quaternion<Scalar>& q_G_I,
                const Matrix<Scalar, 3, 1>& p_G_I) {
    q_G_I_ = q_G_I;
    p_G_I_ = p_G_I;
  }

  void SetZeroState() {
    SetState(Quaternion<Scalar>(1, 0, 0, 0),
             Matrix<Scalar, 3, 1>::Zero());
  }

  Matrix<Scalar, kKeyframeStateSize, 1> GetKeyframeStateVector() const {
    Matrix<Scalar, kKeyframeStateSize, 1> keyframe_state_vector;
    keyframe_state_vector << q_G_I_.coeffs(), p_G_I_;
    return keyframe_state_vector;
  }

  // quaternion [x ,y ,z ,w]
  void SetKeyframeStateFromVector(
      const Matrix<Scalar, kKeyframeStateSize, 1>& state_vector) {
    CHECK(state_vector.rows() == kKeyframeStateSize)
        << "input keyframe state invalid !!!";
    CHECK(state_vector.cols() == 1) << "input keyframe state invalid !!!";
    q_G_I_ = Quaternion<Scalar>(state_vector.head(4).data());
    q_G_I_.normalize();
    p_G_I_ = state_vector.block(0, 0, 3, 1);
  }

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
    covariance_.resize(kImuErrorStateSize + kCalibrationErrorStateSize,
                      kImuErrorStateSize + kCalibrationErrorStateSize);
    covariance_.setIdentity(kImuErrorStateSize + kCalibrationErrorStateSize,
                           kImuErrorStateSize + kCalibrationErrorStateSize);
  }

  EkfState(int num_keyframe) : last_update_time_(double(0)) {
    for (int i = 0; i < num_keyframe; i++) {
      keyframe_states_.push_back(KeyFrameState<Scalar>());
    }
    covariance_.resize(
          kImuErrorStateSize + kCalibrationErrorStateSize
          + num_keyframe * KeyFrameState<Scalar>::ErrorStateSize(),
          kImuErrorStateSize + kCalibrationErrorStateSize
          + num_keyframe * KeyFrameState<Scalar>::ErrorStateSize());
    covariance_.setIdentity(
          kImuErrorStateSize + kCalibrationErrorStateSize
          + num_keyframe * KeyFrameState<Scalar>::ErrorStateSize(),
          kImuErrorStateSize + kCalibrationErrorStateSize
          + num_keyframe * KeyFrameState<Scalar>::ErrorStateSize());
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

  int StateSize() const {
    return imu_state_.StateSize() + calibration_state_.StateSize() +
           keyframe_states_.size() * KeyFrameState<Scalar>::StateSize() +
          landmarks_.size() * 3;
  }

  int ErrorStateSize() {
    return imu_state_.ErrorStateSize() + calibration_state_.ErrorStateSize() +
           keyframe_states_.size() * 6 + landmarks_.size() * 3;
  }

  Matrix<Scalar, 16, 1> GetImuStateVector() const {
    return imu_state_.GetImuStateVector();
  }

  void SetImuStateFromVector(
      const Matrix<Scalar, Eigen::Dynamic, 1>& state_vector) {
    CHECK(state_vector.rows() == kImuStateSize)
        << "input Imu state invalid !!!";
    CHECK(state_vector.cols() == 1) << "input Imu state invalid !!!";
    imu_state_.SetImuStateFromVector(state_vector);
  }

  Matrix<Scalar, kCalibrationStateSize, 1> GetCalibrationStateVector() const {
    return calibration_state_.GetCalibrationStateVector();
  }

  void SetCalibrationStateFromVector(
      const Matrix<Scalar, Eigen::Dynamic, 1>& state_vector) {
    CHECK(state_vector.rows() == kCalibrationStateSize)
        << "input calibration state invalid !!!";
    CHECK(state_vector.cols() == 1) << "input calibration state invalid !!!";
    calibration_state_.SetCalibrationStateFromVector(state_vector);
  }

  Matrix<Scalar, Eigen::Dynamic, 1> GetStateVector() const {
    CHECK(landmarks_.size() == 0) << "has landmark !!!"; // for now , we have no landmark
    Matrix<Scalar, Eigen::Dynamic, 1> state_vector;
    state_vector.resize(StateSize(), 1);
    state_vector.block(kImuStateIndex, 0, kImuStateSize, 1) =
        GetImuStateVector();
    state_vector.block(kCalibrationStateIndex, 0, kCalibrationStateSize, 1) =
        GetCalibrationStateVector();
    for (int i = 0; i < keyframe_states_.size(); i++) {
      state_vector.block(kKeyframeStateIndex + i * kKeyframeStateSize, 0,
                         kKeyframeStateSize, 1) =
          keyframe_states_[i].GetKeyframeStateVector();
    }
    return state_vector;
  }

  void SetStateFromVector(
      const Matrix<Scalar, Eigen::Dynamic, 1>& state_vector) {
    CHECK(state_vector.rows() == StateSize())
        << "SetStateFromVector size not match !!!";
    CHECK(state_vector.cols() == 1)
        << "SetStateFromVector size not match !!!";
    Matrix<Scalar, kImuStateSize, 1> imu_state_vector =
        state_vector.block(kImuStateIndex, 0, kImuStateSize, 1);
    imu_state_.SetImuStateFromVector(imu_state_vector);
    Matrix<Scalar, kCalibrationStateSize, 1> calib_state_vector =
        state_vector.block(kCalibrationStateIndex, 0, kCalibrationStateSize, 1);
    calibration_state_.SetCalibrationStateFromVector(calib_state_vector);
    // set keyframe state
    for (int i = 0; i < keyframe_states_.size(); i++) {
      Matrix<Scalar, kKeyframeStateSize, 1> keyframe_state_vector =
          state_vector.block(kKeyframeStateIndex + i * kKeyframeStateSize, 0,
                             kKeyframeStateSize, 1);
      keyframe_states_[i].SetKeyframeStateFromVector(keyframe_state_vector);
    }
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
        covariance_.block(kImuQErrorStateIndex, kImuQErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 3, 3, 3) =
        covariance_.block(kImuPErrorStateIndex, kImuPErrorStateIndex, 3, 3);
    new_KF_cov.block(3, 0, 3, 3) =
        covariance_.block(kImuPErrorStateIndex, kImuQErrorStateIndex, 3, 3);
    new_KF_cov.block(0, 3, 3, 3) =
        covariance_.block(kImuQErrorStateIndex, kImuPErrorStateIndex, 3, 3);

    // copy new_KF to calib and old_KFs cov
    Matrix<Scalar, Eigen::Dynamic, 6> new_KF_to_calib_old_KFs_cov;
    int KFs_size = keyframe_states_.size();
    new_KF_to_calib_old_KFs_cov.resize(6 * KFs_size + kCalibrationErrorStateSize, 6);
    new_KF_to_calib_old_KFs_cov.block(0, 0,
                                      6 * KFs_size + kCalibrationErrorStateSize, 3) =
        covariance_.block(kCalibQErrorStateIndex,
                         kImuQErrorStateIndex,
                         6 * KFs_size + kCalibrationErrorStateSize, 3);
    new_KF_to_calib_old_KFs_cov.block(0, 3,
                                      6 * KFs_size + kCalibrationErrorStateSize, 3) =
        covariance_.block(kCalibQErrorStateIndex,
                         kImuPErrorStateIndex,
                         6 * KFs_size + kCalibrationErrorStateSize, 3);

    // copy imu to new KF cov
    Matrix<Scalar, kImuErrorStateSize, 6> new_KF_to_imu_cov;
    new_KF_to_imu_cov.setZero(kImuErrorStateSize, 6);
    new_KF_to_imu_cov.block(kImuQErrorStateIndex, 0, kImuErrorStateSize, 3) =
        covariance_.block(kImuQErrorStateIndex, kImuQErrorStateIndex,
                         kImuErrorStateSize, 3);
    new_KF_to_imu_cov.block(kImuQErrorStateIndex, 3, kImuErrorStateSize, 3) =
        covariance_.block(kImuQErrorStateIndex, kImuPErrorStateIndex,
                         kImuErrorStateSize, 3);

    // add new cov_block to new augmented cov
    int new_KF_insert_index = covariance_.cols();
    covariance_.conservativeResize(new_KF_insert_index + 6,
                                  new_KF_insert_index + 6);
    covariance_.block(new_KF_insert_index, new_KF_insert_index, 6, 6) =
        new_KF_cov;
    covariance_.block(kCalibQErrorStateIndex,
                     new_KF_insert_index,
                     6 * KFs_size + kCalibrationErrorStateSize, 6)
        = new_KF_to_calib_old_KFs_cov;
    covariance_.block(new_KF_insert_index,
                     kCalibQErrorStateIndex,
                     6, 6 * KFs_size + kCalibrationErrorStateSize)
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
