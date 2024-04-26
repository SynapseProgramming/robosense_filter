#include <robosense_filter/ring_filter.h>
#include <iostream>

RingFilter::RingFilter(ros::NodeHandle nh, ros::NodeHandle pnh)
    : m_nh(nh), m_pnh(pnh) {
  std::cout << "RingFilter constructor called" << std::endl;
}