#ifndef MSCKF_ESTIMATOR_H_
#define MSCKF_ESTIMATOR_H_

#include <deque>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "ultimate_msckf_vio/common_data/common_data.h"

namespace ultimate_msckf_vio {
class MsckfEstimator {
 public:
  MsckfEstimator():
    estimator_status_(kInitializeing),
    frame_count_(0){}

  enum EstimatorStatus {
    kInitializeing = 0,
    kNormalStage = 1,
    kLost = 2,
    kUnknown = 3
  };

  EstimatorStatus estimator_status() {
    return estimator_status_;
  }

  void ProcessMeasurementsInitialization(const SensorMeasurement&);

  void ProcessMeasurementsNormalStage(const SensorMeasurement&);




 private:
  EstimatorStatus estimator_status_;
  int frame_count_;


};

}

#endif // MSCKF_ESTIMATOR_H_
