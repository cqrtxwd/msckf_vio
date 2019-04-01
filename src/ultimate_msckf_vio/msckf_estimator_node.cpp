#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/msckf_estimator.h"
#include <mutex>
#include <deque>

using std::mutex;
using std::deque;



mutex sensor_buf_mutex;
deque<sensor_msgs::ImuConstPtr> imu_buf;
deque<sensor_msgs::PointCloudConstPtr> images_buf;

ultimate_msckf_vio::MsckfEstimator msckf_estimator;

void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  // ROS_INFO_STREAM("receve imu");
  msckf_estimator.ReceiveImuMeasurement(imu_msg);
}

void FeaturesCallback(const sensor_msgs::PointCloudConstPtr& image) {
  // ROS_INFO_STREAM("receve features");
  msckf_estimator.ReceiveImage(image);
}

void RestartCallback(const std_msgs::BoolConstPtr& restart_msg) {
  ROS_INFO_STREAM("receve call back");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "msckf_estimator");
  ros::NodeHandle n;

  ROS_INFO_STREAM(" msckf start");

  ros::Subscriber sub_feature =
      n.subscribe("/feature_tracker/feature", 2000, FeaturesCallback);
  ros::Subscriber sub_restart =
      n.subscribe("/feature_tracker/restart", 2000, RestartCallback);
  ros::Subscriber sub_imu = n.subscribe("imu0", 2000, ImuCallback);

  ros::spin();

  return 0;
}