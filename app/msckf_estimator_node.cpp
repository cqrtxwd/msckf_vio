#include <deque>
#include <mutex>
#include <thread>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include "ultimate_msckf_vio/data_manager.h"

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

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "msckf_estimator");
  ros::NodeHandle n;

  LOG(INFO) << "hello msckf";
  LOG(ERROR) << "hello msckf";

  ros::Subscriber sub_feature =
      n.subscribe("/feature_tracker/feature", 2000, FeaturesCallback);
  ros::Subscriber sub_restart =
      n.subscribe("/feature_tracker/restart", 2000, RestartCallback);
  ros::Subscriber sub_imu = n.subscribe("imu0", 2000, ImuCallback);

  std::thread process_thread{ProcessLoop};

  ros::spin();

  return 0;
}
