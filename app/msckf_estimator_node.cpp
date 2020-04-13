#include <deque>
#include <mutex>
#include <thread>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include <cv_bridge/cv_bridge.h>
#include "ultimate_msckf_vio/data_manager.h"
#include "ultimate_msckf_vio/parameter_reader.h"

#include <opencv2/highgui/highgui.hpp>

using std::deque;
using std::mutex;

ultimate_msckf_vio::DataManager data_manager;

void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  // LOG(INFO) << "receve imu";
  data_manager.ReceiveImuMeasurement(imu_msg);
}

void FeaturesCallback(const sensor_msgs::PointCloudConstPtr& image) {
  // LOG(INFO) << "receve features";
  data_manager.ReceiveImage(image);
}

void RestartCallback(const std_msgs::BoolConstPtr& restart_msg) {
  LOG(INFO) << "receve call back";
}

void ProcessLoop() {
  while (true) {
    data_manager.Process();
  }
}

void RawImageCallBack(const sensor_msgs::ImageConstPtr& raw_image) {
  LOG(INFO) << "raw image coming";
//  cv_bridge::CvImageConstPtr cv_image_ptr;
  auto cv_image_ptr = cv_bridge::toCvCopy(raw_image, sensor_msgs::image_encodings::MONO8);

  cv::namedWindow("raw image", cv::WINDOW_AUTOSIZE);
  cv::imshow("raw image", cv_image_ptr->image);
  cv::waitKey(0);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "msckf_estimator");
  ros::NodeHandle n;

  LOG(INFO) << "hello msckf";
  LOG(ERROR) << "hello msckf";

  ParameterReader param_reader;
  param_reader.ReadParametersFromYaml(
        "/home/cqr/catkin_ws/src/VINS-Mono/config/euroc/euroc_config.yaml");

  ros::Subscriber sub_raw_image =
      n.subscribe(param_reader.image0_topic, 2000, RawImageCallBack);

  ros::Subscriber sub_feature =
      n.subscribe("/feature_tracker/feature", 2000, FeaturesCallback);
  ros::Subscriber sub_restart =
      n.subscribe("/feature_tracker/restart", 2000, RestartCallback);
  ros::Subscriber sub_imu = n.subscribe("imu0", 2000, ImuCallback);

  std::thread process_thread{ProcessLoop};

  ros::spin();

  return 0;
}
