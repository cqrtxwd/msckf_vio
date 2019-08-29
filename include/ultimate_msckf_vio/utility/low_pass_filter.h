#ifndef LOW_PASS_FILTER_H_
#define LOW_PASS_FILTER_H_

#include <cmath>
#include "ros/ros.h"
#include <glog/logging.h>

namespace ultimate_msckf_vio {

template <typename Scalar>
class LowPassFilter {
 public:
  LowPassFilter()
      : cut_off_freqency_(10.0),
        flag_initialized_(false),
        filtered_data_timestamp_(0) {
    LOG(INFO) << "filter cut off freqency is: " << cut_off_freqency_;
  }

  LowPassFilter(double cut_off_frqency)
      : cut_off_freqency_(cut_off_frqency),
        flag_initialized_(false),
        filtered_data_timestamp_(0) {
//    ROS_INFO_STREAM("filter cut off freqency is: " << cut_off_freqency_;);
  }

  Scalar FilterData(Scalar unfiltered_data, double timestamp) {
    if (!flag_initialized_) {
      flag_initialized_ = true;
      filtered_data_ = unfiltered_data;
      filtered_data_timestamp_ = timestamp;
    } else {
      double dt = timestamp - filtered_data_timestamp_;
      double time_constant = 1.0 / (2 * M_PI * cut_off_freqency_);
      double alpha = dt / (time_constant + dt);
      filtered_data_ = alpha * unfiltered_data + (1 - alpha) * filtered_data_;
      filtered_data_timestamp_ = timestamp;
    }
    return filtered_data_;
  }

 private:
  Scalar filtered_data_;
  bool flag_initialized_;
  double cut_off_freqency_;
  double filtered_data_timestamp_;
};

}  // namespace ultimate_msckf_vio

#endif
