#ifndef MSCKF_ESTIMATOR_H_
#define MSCKF_ESTIMATOR_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include "ros/ros.h"
#include "glog/logging.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "ultimate_msckf_vio/common_data/common_data.h"
#include "ultimate_msckf_vio/ekf_state.h"
#include "ultimate_msckf_vio/utility/low_pass_filter.h"
#include "ultimate_msckf_vio/utility/geometric_kit.h"
#include "ultimate_msckf_vio/visual_observation_manager.h"
#include "ultimate_msckf_vio/visual_ekf_update.h"
#include "ultimate_msckf_vio/utility/timer.h"

namespace ultimate_msckf_vio {

constexpr double kAccelFilterCutOffFreqency = 6;
constexpr double kGyroFilterCutOffFreqency = 6;


using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Matrix;
using std::pair;
using std::map;

class MsckfEstimator {
 public:
  MsckfEstimator()
      : estimator_status_(kInitializeing),
        frame_count_(0),
        accel_filter_(kAccelFilterCutOffFreqency),
        gyro_filter_(kGyroFilterCutOffFreqency),
        pre_time_(-1),
        cur_time_(-1) {
    visual_observation_manager_.Initialize(&ekf_state_);
    ekf_update_.reset(new VisualEkfUpdate());
  }

  enum EstimatorStatus {
    kInitializeing = 0,
    kNormalStage = 1,
    kLost = 2,
    kUnknown = 3
  };

  EstimatorStatus estimator_status() { return estimator_status_; }

  void ProcessMeasurementsInitialization(const SensorMeasurement&);

  void ProcessMeasurementsNormalStage(const SensorMeasurement&);

  void PropagateImu(const sensor_msgs::ImuConstPtr&);

  bool PropagateEkfStateAndPhiByRungeKutta(
      const Vector3d& accel_measurement,
      const Vector3d& gyro_measurement,
      const double& delta_t, EkfState<double>* ekf_state,
      Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* phi);

  void SetInitialState(const Quaterniond& q,
                       const Vector3d& bg,
                       const Vector3d& v,
                       const Vector3d& ba,
                       const Vector3d& p);

  bool PropagateStateByRungeKuttaSingleStep(
      const Matrix<double, kImuStateSize, 1>& state_vector,
      const Matrix<double, kImuErrorStateSize, kImuErrorStateSize>& state_transition,
      const Matrix<double, 3, 1>& accel_body,
      const Matrix<double, 3, 1>& gyro_body,
      const Matrix<double, 3, 1>& gravity,
      const double& delta_t,
      Matrix<double, kImuStateSize, 1>* derivative_vector,
      Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* state_transition_derivative,
      Matrix<double, kImuErrorStateSize, kImuErrorStateSize>* covariance);

  bool KeyFrameCheck();

  bool ShouldMarginalize();

  bool ShouldVisualUpdate();


 private:
  EstimatorStatus estimator_status_;
  EkfStated ekf_state_;

  int frame_count_;
  int key_frame_count_;
  double pre_time_;
  double cur_time_;

  LowPassFilter<Eigen::Vector3d> accel_filter_;
  LowPassFilter<Eigen::Vector3d> gyro_filter_;

  VisualObservationManager visual_observation_manager_;

  std::unique_ptr<VisualEkfUpdate> ekf_update_;

};

}  // namespace ultimate_msckf_vio

#endif  // MSCKF_ESTIMATOR_H_
