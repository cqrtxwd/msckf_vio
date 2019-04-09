#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/data_manager.h"
#include "eigen3/Eigen/Eigen"

using std::deque;
using std::mutex;

ultimate_msckf_vio::DataManager data_manager;

void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  // ROS_INFO_STREAM("receve imu");
  data_manager.ReceiveImuMeasurement(imu_msg);
}

void FeaturesCallback(const sensor_msgs::PointCloudConstPtr& image) {
  // ROS_INFO_STREAM("receve features");
  data_manager.ReceiveImage(image);
}

void RestartCallback(const std_msgs::BoolConstPtr& restart_msg) {
  ROS_INFO_STREAM("receve call back");
}

void ProcessLoop() {
  while (true) {
    data_manager.Process();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "msckf_estimator");
  ros::NodeHandle n;

  ROS_INFO_STREAM(" msckf start");
  // Eigen::Quaterniond q(1, 2, 3, 4);
  // q.coeffs();
  // ROS_INFO_STREAM("q.coeffs(); " << q.coeffs(););

  ros::Subscriber sub_feature =
      n.subscribe("/feature_tracker/feature", 2000, FeaturesCallback);
  ros::Subscriber sub_restart =
      n.subscribe("/feature_tracker/restart", 2000, RestartCallback);
  ros::Subscriber sub_imu = n.subscribe("imu0", 2000, ImuCallback);

  std::thread process_thread{ProcessLoop};

  ros::spin();

  return 0;
}
