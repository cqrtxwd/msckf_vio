#include <iostream>
#include "eigen3/Eigen/Eigen"
#include "glog/logging.h"
#include "ultimate_msckf_vio/ekf_state.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  LOG(INFO) << "test ekf state ";
  ultimate_msckf_vio::EkfStated ekf_state;
  ekf_state.PrintInfo();
  return 0;
}
