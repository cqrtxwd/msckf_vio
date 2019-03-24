#include <functional>
#include "ros/ros.h"
#include "std_msgs/String.h"

void ListennerCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("subscriber recieve %s ", msg->data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "listenner");
  ros::NodeHandle listenner_handle;
  auto callback = [](std_msgs::String::ConstPtr& msg) {
    ROS_INFO("subscriber recieve %s ", msg->data.c_str());
  };
  ros::Subscriber msg_subscriber =
      listenner_handle.subscribe("chatter_test", 1000, ListennerCallback);
  ros::spin();
  return 0;
}

// [](std_msgs::String::ConstPtr& msg) {
//   ROS_INFO("subscriber recieve %s ", msg->data.c_str());
// }