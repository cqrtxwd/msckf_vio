#ifndef FRAME_H
#define FRAME_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

namespace ultimate_msckf_vio {
class Frame {
 public:
  Frame() = delete;
  Frame(const int tmp_id,
        const ros::Time& tmp_image_timestamp,
        std::vector<cv::KeyPoint>& tmp_keypoints,
        cv::Mat& tmp_descriptors);

  int id;
  ros::Time timestamp;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

 private:

};

}



#endif // FRAME_H
