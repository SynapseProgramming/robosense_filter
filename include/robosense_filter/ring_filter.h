#ifndef RING_FILTER_H
#define RING_FILTER_H
#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#if PCL_VERSION_COMPARE(>, 1, 9, 0)
#include <pcl-1.10/pcl/filters/impl/filter.hpp>
#endif

#if PCL_VERSION_COMPARE(<, 1, 9, 0)
#include <pcl-1.8/pcl/filters/impl/filter.hpp>
#endif

struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  std::uint16_t intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    Point, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                                   intensity)(
               double, timestamp, timestamp)(std::uint16_t, ring, ring))

class RingFilter {
 public:
  RingFilter(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
  void scanCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  ros::NodeHandle m_nh;
  ros::NodeHandle m_pnh;

  ros::Subscriber m_scan_sub;
  ros::Publisher m_scan_pub;
};

#endif  // RING_FILTER_H