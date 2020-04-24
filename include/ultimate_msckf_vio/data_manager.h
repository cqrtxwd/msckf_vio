#ifndef DATA_MANAGER_H_
#define DATA_MANAGER_H_

#include <mutex>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include <condition_variable>
#include "ultimate_msckf_vio/msckf_estimator.h"
#include "ultimate_msckf_vio/common_data/common_data.h"
#include "ultimate_msckf_vio/utility/timer.h"
#include "ultimate_msckf_vio/feature_tracker.h"
#include "ultimate_msckf_vio/parameter_reader.h"
#include "ultimate_msckf_vio/vio_initializer.h"
#include "ultimate_msckf_vio/frame.h"



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


/*
 * class graph:
 *  DataManager
 *      |-FeatureTracker
 *      |-MsckfEstimator
 *      |-VIOInitializer
 *      |
 *      |
 */



class DataManager {
 public:
  DataManager(ParameterReader* parameter_reader);

  void Process();

  bool ReceiveRawImage(const sensor_msgs::ImageConstPtr& raw_image);

//  bool ReceiveImage(const sensor_msgs::PointCloudConstPtr& image);

  void ReceiveImuMeasurement(const sensor_msgs::ImuConstPtr& imu_msg);

  SensorMeasurement GetSensorMeasurement();

  bool ProcessMeasurement(const SensorMeasurement&);

 private:
  ImuConstPtr InterpolateImu(const double&, const ImuConstPtr&,
                             const ImuConstPtr&);

  int AddFrame(const ros::Time& timestamp,
               std::vector<cv::KeyPoint>& keypoints,
               cv::Mat& descriptors);

  double cur_time_;
  deque<sensor_msgs::ImuConstPtr> imu_buf_;  // front is the oldest data
  deque<sensor_msgs::PointCloudConstPtr> images_buf_;  // front is the oldest

  deque<Frame> frames_; // guarded by frame_mutex_

  // 3d point_cloud
  std::map<int /* feature id */, Eigen::Vector3d> point_cloud_;

  std::condition_variable process_thread_;
  MsckfEstimator msckf_estimator_;
  std::shared_ptr<ParameterReader> parameter_reader_;
  friend class FeatureTracker;
  std::unique_ptr<FeatureTracker> feature_tracker_;
  friend class VIOInitializer;
  std::shared_ptr<VIOInitializer> vio_initializer_;

  std::mutex frame_mutex_;
  std::mutex sensor_buf_mutex_;

};

}  // namespace ultimate_msckf_vio

#endif
