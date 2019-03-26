#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/msckf_estimator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "msckf_estimator");
  ros::NodeHandle n;

  ros::Subscriber sub_feature = n.subscribe("cam0/image_raw", 100, );
  ros::Subscriber sub_feature = n.subscribe("cam0/image_raw", 100, );
  ros::Subscriber sub_feature = n.subscribe("cam0/image_raw", 100, );


  ros::spin();

  return 0;
}