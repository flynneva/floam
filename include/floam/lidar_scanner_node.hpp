
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_SCANNER_NODE_HPP_
#define FLOAM__LIDAR_SCANNER_NODE_HPP_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar.hpp"
#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{


class ScanningLidarNode : public nodelet::Nodelet
{
public:
  ScanningLidarNode();
  ~ScanningLidarNode();

  void onInit();

  void handlePoints(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);

  Lidar<floam::lidar::Scanner> m_lidar;

private:
  ros::NodeHandle m_nodeHandle;

  ros::Subscriber m_subPoints;

  ros::Publisher m_pubEdgePoints;
  ros::Publisher m_pubSurfacePoints;
  ros::Publisher m_pubEdgesAndSurfaces;

  tf2_ros::Buffer m_tf2Buffer;
  tf2_ros::TransformListener m_tf2Listener;

  std::queue<sensor_msgs::PointCloud2ConstPtr> m_points;
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_SCANNER_NODE_HPP_