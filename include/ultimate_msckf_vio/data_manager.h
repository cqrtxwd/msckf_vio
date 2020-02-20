#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include <mutex>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include <condition_variable>
#include "ultimate_msckf_vio/msckf_estimator.h"
#include "ultimate_msckf_vio/common_data/common_data.h"
#include "ultimate_msckf_vio/utility/timer.h"

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

// for debug
constexpr double kTime0 = 1.40364e+9 - 3500;

//struct SensorMeasurement {
//  // SensorMeasurement(deque<ImuConstPtr> imu, PointCloudConstPtr img_msg) {
//  //   imu_measurements = imu;
//  //   image = img_msg;
//  // }

//  deque<ImuConstPtr> imu_measurements;
//  PointCloudConstPtr image;
//};

class DataManager {
 public:
  DataManager() : cur_time_(-1) {}

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
  MsckfEstimator msckf_estimator_;
};

}  // namespace ultimate_msckf_vio

#endif
