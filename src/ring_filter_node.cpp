#include <robosense_filter/ring_filter.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "ring_filter_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  RingFilter ring_filter(nh, pnh);

  ros::spin();
  return 0;
}