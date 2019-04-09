#include "ultimate_msckf_vio/msckf_estimator.h"

namespace ultimate_msckf_vio {

void MsckfEstimator::ProcessMeasurementsInitialization(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("Initializing");
  frame_count_++;
  if (frame_count_ >= 20) {
    ROS_INFO_STREAM("Initialize done ...");
    estimator_status_ = EstimatorStatus::kNormalStage;
  }
}

void MsckfEstimator::ProcessMeasurementsNormalStage(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("normal stage");
  ROS_ASSERT(measurement.image->header.stamp ==
             measurement.imu_measurements.back()->header.stamp);
  // propagete imu data, push ekf state to next image timestamp
  deque<ImuConstPtr> imu_measurements = measurement.imu_measurements;
  for (auto imu_msg : imu_measurements) {
    ProcessImu(imu_msg);
  }

  // check keyframe

  // when there is enough KF , ekf update
}

void MsckfEstimator::ProcessImu(const sensor_msgs::ImuConstPtr& imu_msg) {
  if (cur_time_ <= 0) {
    cur_time_ = imu_msg->header.stamp.toSec();
    pre_time_ = cur_time_;
    return;
  }
  pre_time_ = cur_time_;
  cur_time_ = imu_msg->header.stamp.toSec();

  // convert ros sensor_msg to Eigen Vector
  Eigen::Vector3d accel_data(imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyro_data(imu_msg->angular_velocity.x,
                            imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z);

  // low pass filter for imu data
  auto filtered_accel_data = accel_filter_.FilterData(accel_data, cur_time_);
  auto fitered_gyro_data = gyro_filter_.FilterData(gyro_data, cur_time_);
  // propagate imu to next timestamp
  double delta_t = cur_time_ - pre_time_;
  if (!PropagateEkfState(filtered_accel_data, fitered_gyro_data, delta_t,
                         &ekf_state_)) {
    ROS_INFO_STREAM("Propagate IMU failed !!!");
  }
}

bool MsckfEstimator::PropagateEkfState(const Vector3d& accel_measurement,
                                       const Vector3d& gyro_measurement,
                                       const double& delta_t,
                                       EkfState<double>* ekf_state) {
  // use RungeKutta4 to propagate IMU state
  auto imu_state_vector = ekf_state->GetImuStateVector();

  Matrix<double, kImuStateSize, kImuStateSize> phi;
  Matrix<double, kImuErrorStateSize, kImuErrorStateSize> imu_cov =
      ekf_state->GetImuStateCovariance();



  return true;
}

}  // namespace ultimate_msckf_vio
