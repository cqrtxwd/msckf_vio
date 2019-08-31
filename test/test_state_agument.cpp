#include <iostream>
#include "eigen3/Eigen/Eigen"
#include "glog/logging.h"
#include "ultimate_msckf_vio/ekf_state.h"

using namespace ultimate_msckf_vio;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "test ekf state ";
  LOG(WARNING) << "-------------- index check -------------- ";
  //state size
  LOG(INFO) << "kImuStateSize: " << kImuStateSize;
  LOG(INFO) << "kImuErrorStateSize: " << kImuErrorStateSize;
  LOG(INFO) << "kCalibStateSize: " << kCalibrationStateSize;
  LOG(INFO) << "kCalibErrorStateSize: " << kCalibrationErrorStateSize;

  // index check
  LOG(INFO) << "\nkImuQStateIndex: " << kImuQStateIndex << "\n"
            << "kImuBgStateIndex: " << kImuBgStateIndex << "\n"
            << "kImuVStateIndex: " << kImuVStateIndex << "\n"
            << "kImuBaStateIndex: " << kImuBaStateIndex << "\n"
            << "kImuPStateIndex: " << kImuPStateIndex << "\n"
            << "kCalibQStateIndex: " << kCalibrationQStateIndex << "\n"
            << "kCalibPStateIndex: " << kCalibrationPStateIndex << "\n";

  LOG(INFO) << "\nkImuQErrorStateIndex: " << kImuQErrorStateIndex << "\n"
            << "kImuBgErrorStateIndex: " << kImuBgErrorStateIndex << "\n"
            << "kImuVErrorStateIndex: " << kImuVErrorStateIndex << "\n"
            << "kImuBaErrorStateIndex: " << kImuBaErrorStateIndex << "\n"
            << "kImuPErrorStateIndex: " << kImuPErrorStateIndex << "\n"
            << "kCalibQErrorStateIndex: " << kCalibQErrorStateIndex << "\n"
            << "kCalibPErrorStateIndex: " << kCalibPErrorStateIndex << "\n";

  LOG(WARNING) << "-------------- default state -------------- ";
  ultimate_msckf_vio::EkfStated default_state;
  LOG(INFO) << "state size: " << default_state.StateSize();
  default_state.PrintInfo();
  LOG(INFO) << "covarance: \n" << default_state.covariance();

  LOG(WARNING) << "-------------- default 3 keyframe state -------------- ";
  ultimate_msckf_vio::EkfStated kf3_state(3);
  LOG(INFO) << "state size: " << kf3_state.StateSize();
  kf3_state.PrintInfo();
  LOG(INFO) << "covarance: \n" << kf3_state.covariance();

  // set state , get state
  LOG(WARNING) << "-------------- set state , get state -------------- ";
  LOG(INFO) << "imu state before: \n" <<kf3_state.GetImuStateVector().transpose();
  LOG(INFO) << "calib state before: \n" <<kf3_state.GetCalibrationStateVector().transpose();
  LOG(INFO) << "keyframe state before: \n" <<kf3_state.GetCalibrationStateVector().transpose();
  LOG(INFO) << "state before: \n" <<kf3_state.GetStateVector().transpose();

  Matrix<double, 16, 1> input_imu_state;
  input_imu_state << 1,2,3, 1,2,3, 1,2,3, 1,2,3, 1,2,3, 1;
  kf3_state.SetImuStateFromVector(input_imu_state);
  LOG(INFO) << "state after set imu: \n" <<kf3_state.GetStateVector().transpose();

  Matrix<double, 7, 1> input_calib_state;
  input_calib_state << 7, 6, 5, 4, 3, 2, 1;
  kf3_state.SetCalibrationStateFromVector(input_calib_state);
  LOG(INFO) << "state after set calib: \n" <<kf3_state.GetStateVector().transpose();

  Matrix<double, 44, 1> input_state;
  LOG(INFO) << "test 0";
  input_state << 1,2,3,4,5,6,7,8,9,0,
                 1,2,3,4,5,6,7,8,9,0,
                 1,2,3,4,5,6,7,8,9,0,
                 1,2,3,4,5,6,7,8,9,0,
                 1,2,3,4;
  LOG(INFO) << "test 1";
  kf3_state.SetStateFromVector(input_state);
  LOG(INFO) << "state after set 2: \n" <<kf3_state.GetStateVector().transpose();



  return 0;
}
