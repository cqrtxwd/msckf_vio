#include <deque>
#include <mutex>
#include <thread>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "std_msgs/Bool.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/function.hpp>

#include "ultimate_msckf_vio/data_manager.h"
#include "ultimate_msckf_vio/parameter_reader.h"


using std::deque;
using std::mutex;

void RestartCallback(const std_msgs::BoolConstPtr& restart_msg) {
  LOG(INFO) << "receve call back";
}

//void ProcessLoop() {
//  while (true) {
//    data_manager.Process();
//  }
//}

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

  ParameterReader parameter_reader;

  parameter_reader.ReadParametersFromYaml(
        "/home/cqr/catkin_ws/src/VINS-Mono/config/euroc/euroc_config.yaml");

  ultimate_msckf_vio::DataManager data_manager(&parameter_reader);

  ros::Subscriber sub_raw_image =
      n.subscribe(parameter_reader.image0_topic, 1000,
                  boost::function<void(const sensor_msgs::ImageConstPtr&)>(
                    [&data_manager]
                    (const sensor_msgs::ImageConstPtr& raw_image) {
    data_manager.ReceiveRawImage(raw_image);
  }));


  ros::Subscriber sub_imu =
      n.subscribe("imu0", 2000,
                  boost::function<void(const sensor_msgs::ImuConstPtr&)>(
                    [&data_manager](const sensor_msgs::ImuConstPtr& imu_msg) {
    data_manager.ReceiveImuMeasurement(imu_msg);
  }));

//  std::thread process_thread{ProcessLoop};

  ros::spin();

  return 0;
}
