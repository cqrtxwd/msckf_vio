#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle talker_handle;
  ros::Publisher msg_publisher =
      talker_handle.advertise<std_msgs::String>("chatter_test", 1000);
  ros::Rate node_freqency(10);

  int loop_count = 0;
  while (ros::ok()) {
    std_msgs::String some_words;
    std::stringstream ss;
    ss << "hello ros " << loop_count;
    some_words.data = ss.str();
    msg_publisher.publish(some_words);
    // ROS_INFO_STREAM << "LOG INFO : " << some_words.data.c_str();
    ROS_INFO("c format info %s", some_words.data.c_str());
    ros::spinOnce();
    node_freqency.sleep();
    ++loop_count;
  }
  return 0;
}
