#ifndef COMMON_DATA_H_
#define COMMON_DATA_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <mutex>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"

namespace ultimate_msckf_vio {
using std::deque;
using sensor_msgs::ImuConstPtr;
using sensor_msgs::PointCloudConstPtr;

struct SensorMeasurement {
  deque<ImuConstPtr> imu_measurements;
  PointCloudConstPtr image;
};



}




#endif // COMMON_DATA_H_