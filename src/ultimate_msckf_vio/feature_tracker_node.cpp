#include <cv_bridge/cv_bridge.h>
// #include <glog/logging.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/feature_tracker.h"

double last_time = 0;
double cur_time = 0;
ultimate_msckf_vio::FeatrueTracker feature_tracker;

void ImageCallback(const sensor_msgs::ImageConstPtr& raw_image) {
  last_time = cur_time;
  cur_time = raw_image->header.stamp.toSec();

  ROS_INFO_STREAM("time count " << std::setprecision(10)
                                << cur_time - last_time);

  // prepare the image to process
  ROS_INFO_STREAM("image encoding " << raw_image->encoding);
  auto cv_image_ptr =
      cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::MONO8);
  feature_tracker.ProcessImage(*cv_image_ptr);
}

int main(int argc, char** argv) {
  // google::InitGoogleLogging(argv[0]);
  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle n;

  // TODO: read config file

  ros::Subscriber sub_image = n.subscribe("cam0/image_raw", 100, ImageCallback);

  ros::Publisher pub_feature =
      n.advertise<sensor_msgs::PointCloud>("features", 1000);

  ros::spin();

  return 0;
}
