#ifndef MSCKF_ESTIMATOR_H_
#define MSCKF_ESTIMATOR_H_

#include <deque>
#include <iostream>
#include <mutex>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"

namespace ultimate_msckf_vio {
using std::deque;
using std::mutex;

class MsckfEstimator {
 public:
  MsckfEstimator() : cur_time_(0) {}

  bool ReceiveImage(const sensor_msgs::PointCloudConstPtr& image);

  bool ReceiveImuMeasurement(const sensor_msgs::ImuConstPtr& imu_msg);

  bool ProcessMeasurement();

 private:
  double cur_time_;
  mutex sensor_buf_mutex_;
  deque<sensor_msgs::ImuConstPtr> imu_buf_;
  deque<sensor_msgs::PointCloudConstPtr> images_buf_;
};

}  // namespace ultimate_msckf_vio

#endif