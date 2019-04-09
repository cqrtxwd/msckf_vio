#include "ultimate_msckf_vio/msckf_estimator.h"

void ultimate_msckf_vio::MsckfEstimator::ProcessMeasurementsInitialization(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("Initializing");
  frame_count_++;
  if (frame_count_ >= 20) {
    ROS_INFO_STREAM("Initialize done ...");
    estimator_status_ = EstimatorStatus::kNormalStage;
  }
}

void ultimate_msckf_vio::MsckfEstimator::ProcessMeasurementsNormalStage(
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

void ultimate_msckf_vio::MsckfEstimator::ProcessImu(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  // ros sensor_msg to Eigen_vector
  Eigen::Vector3d accel_data(imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
  Eigen::Vector3d gyro_data(imu_msg->angular_velocity.x,
                            imu_msg->angular_velocity.y,
                            imu_msg->angular_velocity.z);
  double cur_time = imu_msg->header.stamp.toSec();
  // low pass filter imu
  auto filtered_accel_data =
      accel_filter_.FilterData(accel_data, cur_time);
  auto fitered_gyro_data = gyro_filter_.FilterData(gyro_data, cur_time);

  // propagate imu to next timestamp
}
