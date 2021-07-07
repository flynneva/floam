
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef FLOAM__LIDAR_MAPPING_NODE_HPP_
#define FLOAM__LIDAR_MAPPING_NODE_HPP_

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar_mapping.hpp"
#include "floam/lidar_imager.hpp"
#include "floam/lidar_scanner.hpp"

namespace floam
{
namespace lidar
{

class LidarMappingNode : public nodelet::Nodelet
{
public:
  LidarMappingNode();
  ~LidarMappingNode();

  void onInit();

  void handleOdom(const nav_msgs::Odometry::ConstPtr &msg);
  void handlePoints(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);
  void mapping();

private:
  ros::NodeHandle m_nodeHandle;
  ros::Publisher m_pubMap;
  ros::Subscriber m_subPoints, m_subOdom;

  LidarMapping m_lidarMapping;
  ImagingLidar m_lidar;
  std::mutex m_mutexLock;
  std::queue<nav_msgs::OdometryConstPtr> m_odometry;
  std::queue<sensor_msgs::PointCloud2ConstPtr> m_points;
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_MAPPING_NODE_HPP_