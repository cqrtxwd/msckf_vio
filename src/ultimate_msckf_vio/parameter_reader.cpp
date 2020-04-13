#include "ultimate_msckf_vio/parameter_reader.h"

#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

bool ParameterReader::ReadParametersFromYaml(const std::string& config_path) {
  LOG(INFO) << "reading param";
 cv::FileStorage param_file(config_path, cv::FileStorage::READ);
 if (!param_file.isOpened()) {
   LOG(ERROR) << "Fail to read config yaml file at " << config_path;
   return false;
 }

 param_file["image_topic"] >> image0_topic;
 param_file["imu_topic"] >> imu_topic;


 image_width = static_cast<int>(param_file["image_width"]);
 image_height = static_cast<int>(param_file["image_height"]);

 cv::FileNode distort_param = param_file["distortion_parameters"];

 distortion_k1 = static_cast<double>(distort_param["k1"]);
 distortion_k2 = static_cast<double>(distort_param["k2"]);
 distortion_p1 = static_cast<double>(distort_param["p1"]);
 distortion_p2 = static_cast<double>(distort_param["p2"]);

 cv::FileNode intrinsic_param = param_file["projection_parameters"];

 double fx = static_cast<double>(intrinsic_param["fx"]);
 double fy = static_cast<double>(intrinsic_param["fy"]);
 double cx = static_cast<double>(intrinsic_param["cx"]);
 double cy = static_cast<double>(intrinsic_param["cy"]);
 intrinsic_mat << fx, 0, cx,
                  0, fy, cy,
                  0, 0, 1;

  PrintParamInfo();
}

void ParameterReader::PrintParamInfo() {
  LOG(INFO) << "param read : \n"
               "imu_topic: " << imu_topic
            << "\nimage_topic: " << image0_topic
            << "\nimage_width: " << image_width
            << "\nimage_height: " << image_height
            << "\ndistortion: "
            << distortion_k1 << " "
            << distortion_k2 << " "
            << distortion_p1 << " "
            << distortion_p2
            << "\nintrinsic : \n" << intrinsic_mat;
}
