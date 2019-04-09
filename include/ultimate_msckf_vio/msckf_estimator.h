#ifndef MSCKF_ESTIMATOR_H_
#define MSCKF_ESTIMATOR_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "ultimate_msckf_vio/common_data/common_data.h"
#include "ultimate_msckf_vio/ekf_state.h"
#include "ultimate_msckf_vio/utility/low_pass_filter.h"

namespace ultimate_msckf_vio {

constexpr double kAccelFilterCutOffFreqency = 6;
constexpr double kGyroFilterCutOffFreqency = 6;

class MsckfEstimator {
 public:
  MsckfEstimator()
      : estimator_status_(kInitializeing),
        frame_count_(0),
        accel_filter_(kAccelFilterCutOffFreqency),
        gyro_filter_(kGyroFilterCutOffFreqency),
        pre_time_(-1),
        cur_time_(-1) {}

  enum EstimatorStatus {
    kInitializeing = 0,
    kNormalStage = 1,
    kLost = 2,
    kUnknown = 3
  };

  EstimatorStatus estimator_status() { return estimator_status_; }

  void ProcessMeasurementsInitialization(const SensorMeasurement&);

  void ProcessMeasurementsNormalStage(const SensorMeasurement&);

  void ProcessImu(const sensor_msgs::ImuConstPtr&);

  bool PropagateEkfState(const Vector3d& accel_measurement,
                         const Vector3d& gyro_measurement,
                         const double& delta_t, EkfState<double>* ekf_state);

 private:
  EstimatorStatus estimator_status_;
  EkfState<double> ekf_state_;

  int frame_count_;
  double pre_time_;
  double cur_time_;

  LowPassFilter<Eigen::Vector3d> accel_filter_;
  LowPassFilter<Eigen::Vector3d> gyro_filter_;
};

}  // namespace ultimate_msckf_vio

#endif  // MSCKF_ESTIMATOR_H_
