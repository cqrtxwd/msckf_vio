#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/data_manager.h"
#include <mutex>
#include <deque>
#include <thread>

using std::mutex;
using std::deque;


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

void ProcessLoop(){
  while(true) {
    data_manager.Process();
  }
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

  std::thread process_thread{ProcessLoop};

  ros::spin();

  return 0;
}
