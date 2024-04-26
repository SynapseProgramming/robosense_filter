#ifndef RING_FILTER_H
#define RING_FILTER_H

#include <ros/ros.h>

class RingFilter {
 public:
  RingFilter(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;
};

#endif  // RING_FILTER_H