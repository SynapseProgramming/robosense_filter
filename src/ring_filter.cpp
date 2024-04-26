#include <robosense_filter/ring_filter.h>

#include <iostream>

RingFilter::RingFilter(ros::NodeHandle nh, ros::NodeHandle pnh)
    : m_nh(nh), m_pnh(pnh) {
  m_scan_sub =
      m_nh.subscribe("/rslidar_points", 10, &RingFilter::scanCallback, this);

  m_scan_pub =
      m_nh.advertise<sensor_msgs::PointCloud2>("/ring_filtered_points", 10);
}

void RingFilter::scanCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr output_cloud(new pcl::PointCloud<Point>);

  pcl::fromROSMsg(*msg, *cloud);

  for (auto& it : cloud->points) {
    if (it.ring == 31 || it.ring == 30 || it.ring == 29 || it.ring == 28 ||
        it.ring == 27 || it.ring == 26 || it.ring == 24 || it.ring == 23 ||
        it.ring == 21 || it.ring == 20 || it.ring == 18 || it.ring == 17 ||
        it.ring == 16 || it.ring == 15 || it.ring == 14 || it.ring == 13) {
      output_cloud->points.push_back(it);
    }
 
  }

  output_cloud->header = cloud->header;
  output_cloud->height = 1;
  output_cloud->width = output_cloud->points.size();
  output_cloud->is_dense = cloud->is_dense;
  output_cloud->sensor_origin_ = cloud->sensor_origin_;
  output_cloud->sensor_orientation_ = cloud->sensor_orientation_;

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*output_cloud, output_msg);
  output_msg.header = msg->header;
  m_scan_pub.publish(output_msg);
}