#ifndef PARAMETER_READER_H
#define PARAMETER_READER_H

#include <string>

#include <eigen3/Eigen/Eigen>

class ParameterReader {
 public:
  ParameterReader() {
    Param_loaded = false;
  }

  bool ReadParametersFromYaml(const std::string& config_path);

  void PrintParamInfo();

  std::string imu_topic;

  std::string image0_topic;

  std::string image1_topic;

  std::string model_type;
  std::string camera_name;
  int image_width;
  int image_height;

  Eigen::Matrix3d intrinsic_mat;

  bool Param_loaded;

  double distortion_k1;
  double distortion_k2;
  double distortion_p1;
  double distortion_p2;

};
#endif // PARAMETER_READER_H
