#ifndef MSCKF_ESTIMATOR_H_
#define MSCKF_ESTIMATOR_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <mutex>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include <condition_variable>

namespace ultimate_msckf_vio {
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using sensor_msgs::ImuConstPtr;
using sensor_msgs::PointCloudConstPtr;
using std::deque;
using std::make_pair;
using std::mutex;
using std::pair;
using std::vector;

constexpr double kTime0 = 1.40364e+9 - 3500;

struct SensorMeasurement {
  // SensorMeasurement(deque<ImuConstPtr> imu, PointCloudConstPtr img_msg) {
  //   imu_measurements = imu;
  //   image = img_msg;
  // }

  deque<ImuConstPtr> imu_measurements;
  PointCloudConstPtr image;
};

class MsckfEstimator {
 public:
  MsckfEstimator() : cur_time_(0) {}

  void Process();

  bool ReceiveImage(const sensor_msgs::PointCloudConstPtr& image);

  bool ReceiveImuMeasurement(const sensor_msgs::ImuConstPtr& imu_msg);

  SensorMeasurement GetSensorMeasurement();

  bool ProcessMeasurement(const SensorMeasurement&);


 private:
  ImuConstPtr InterpolateImu(const double&, const ImuConstPtr&,
                             const ImuConstPtr&);

  double cur_time_;
  mutex sensor_buf_mutex_;
  deque<sensor_msgs::ImuConstPtr> imu_buf_;  // front is the oldest data
  deque<sensor_msgs::PointCloudConstPtr> images_buf_;  // front is the oldest
  std::condition_variable process_thread_;
};

}  // namespace ultimate_msckf_vio

#endif
