#include "ultimate_msckf_vio/msckf_estimator.h"

namespace ultimate_msckf_vio {

bool MsckfEstimator::ReceiveImage(
    const sensor_msgs::PointCloudConstPtr& image) {
  sensor_buf_mutex_.lock();
  images_buf_.push_back(image);
  sensor_buf_mutex_.unlock();
}

bool MsckfEstimator::ReceiveImuMeasurement(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  // ROS_INFO_STREAM("recieve imudata " << imu_msg->header.stamp.toSec() -
  // kTime0);
  sensor_buf_mutex_.lock();
  imu_buf_.push_back(imu_msg);
  sensor_buf_mutex_.unlock();

  // when accumlate enough imu data
  sensor_buf_mutex_.lock();
  if (images_buf_.empty() || imu_buf_.empty()) {
    sensor_buf_mutex_.unlock();
    return false;
  }
  // ROS_INFO("ready to get measurement");
  if (imu_buf_.back()->header.stamp.toSec() >
      images_buf_.front()->header.stamp.toSec()) {
    auto measurement = GetSensorMeasurement();
    // check if imu deque is aligned with image time
    ROS_ASSERT(measurement.imu_measurements.back()->header.stamp.toSec() ==
               measurement.image->header.stamp.toSec());
  }
  sensor_buf_mutex_.unlock();
  return true;
}

// cut a piece of imu deque as current measurement
SensorMeasurement MsckfEstimator::GetSensorMeasurement() {
  deque<sensor_msgs::ImuConstPtr> imu_deque;
  ROS_INFO_STREAM("get measurement calls";);
  auto image_time = images_buf_.front()->header.stamp.toSec();

  while (imu_buf_.front()->header.stamp.toSec() <= image_time) {
    imu_deque.push_back(imu_buf_.front());
    imu_buf_.pop_front();
  }
  // append a interpolate fake data to make sure imu_deque time are aligned with
  // image_deque time
  if (imu_deque.back()->header.stamp.toSec() < image_time) {
    auto back_imu_data = imu_buf_.front();
    auto pre_imu_data = imu_deque.back();
    ROS_INFO_STREAM("about to interpolate ";);
    auto interpolate_imu =
        InterpolateImu(image_time, pre_imu_data, back_imu_data);
    ROS_INFO_STREAM(
        "interpolate time: " << interpolate_imu->header.stamp.toSec(););
    imu_buf_.push_front(interpolate_imu);
    imu_deque.push_back(interpolate_imu);
  } else if (imu_deque.back()->header.stamp.toSec() == image_time) {
    imu_buf_.push_front(imu_deque.back());
    // ROS_INFO("imu_msg time equals image time exacly ...");
  } else {
    ROS_INFO("you miss some case !!!!!");
  }
  SensorMeasurement result_sensor_measurement{imu_deque, images_buf_.front()};
  images_buf_.pop_front();
  return result_sensor_measurement;
}

ImuConstPtr MsckfEstimator::InterpolateImu(const double& interpolate_time,
                                           const ImuConstPtr& pre_imu,
                                           const ImuConstPtr& forw_imu) {
  double pre_time = pre_imu->header.stamp.toSec();
  double forw_time = forw_imu->header.stamp.toSec();
  if (forw_time <= pre_time) {
    ROS_INFO_STREAM("please inverse timestamp when interpolate imu");
  }

  Matrix<double, 6, 1> pre_accel_gyro;
  pre_accel_gyro << pre_imu->linear_acceleration.x,
      pre_imu->linear_acceleration.y, pre_imu->linear_acceleration.z,
      pre_imu->angular_velocity.x, pre_imu->angular_velocity.y,
      pre_imu->angular_velocity.z;
  Matrix<double, 6, 1> forw_accel_gyro;
  forw_accel_gyro << forw_imu->linear_acceleration.x,
      forw_imu->linear_acceleration.y, forw_imu->linear_acceleration.z,
      forw_imu->angular_velocity.x, forw_imu->angular_velocity.y,
      forw_imu->angular_velocity.z;
  double alpha = (interpolate_time - pre_time) / (forw_time - pre_time);
  ROS_INFO_STREAM("pre_time is: " << pre_time;);
  ROS_INFO_STREAM("forw_time is: " << forw_time;);
  ROS_INFO_STREAM("interpolate_time is: " << interpolate_time;);
  ROS_INFO_STREAM("alpha is: " << alpha;);
  if (alpha > 1 || alpha < 0) {
    ROS_FATAL_STREAM("InterpolateImu alpha wrong!!! alpha is: " << alpha;);
  }
  Matrix<double, 6, 1> result_accel_gyro;
  result_accel_gyro = (1 - alpha) * pre_accel_gyro + alpha * forw_accel_gyro;
  sensor_msgs::Imu result_imu;

  result_imu.linear_acceleration.x = result_accel_gyro(0, 0);
  result_imu.linear_acceleration.y = result_accel_gyro(1, 0);
  result_imu.linear_acceleration.z = result_accel_gyro(2, 0);

  result_imu.angular_velocity.x = result_accel_gyro(3, 0);
  result_imu.angular_velocity.y = result_accel_gyro(4, 0);
  result_imu.angular_velocity.z = result_accel_gyro(5, 0);

  result_imu.header.stamp = ros::Time(interpolate_time);

  ImuConstPtr result_imu_constptr = ImuConstPtr(&result_imu);
  return result_imu_constptr;
}

bool MsckfEstimator::ProcessMeasurement(
    const SensorMeasurement& sensor_measurement) {
      return true;
    }

}  // namespace ultimate_msckf_vio
