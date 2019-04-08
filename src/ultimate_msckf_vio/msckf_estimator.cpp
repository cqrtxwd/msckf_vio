#include "ultimate_msckf_vio/msckf_estimator.h"

void ultimate_msckf_vio::MsckfEstimator::ProcessMeasurementsInitialization(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("Initializing");
  frame_count_++;
  if(frame_count_ >= 20) {
    ROS_INFO_STREAM("Initialize done ...");
    estimator_status_ = EstimatorStatus::kNormalStage;
  }
}

void ultimate_msckf_vio::MsckfEstimator::ProcessMeasurementsNormalStage(
    const SensorMeasurement& measurement) {
  ROS_INFO_STREAM("normal stage");
}

