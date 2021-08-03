
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef FLOAM__LIDAR_MAPPING_NODE_HPP_
#define FLOAM__LIDAR_MAPPING_NODE_HPP_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "floam/lidar_mapping.hpp"
#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ExactSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

class LidarMappingNode : public nodelet::Nodelet
{
public:
  ///
  /// LidarMappingNode constructor
  ///
  LidarMappingNode();

  ///
  /// LidarMappingNode constructor
  ///
  ~LidarMappingNode();

  ///
  /// Initialize Nodelet member variables
  ///
  /// @return void
  ///
  void onInit();

private:
  ros::NodeHandle m_nodeHandle;
  ros::Publisher m_pubMap;

  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSynchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;

  message_filters::Subscriber<sensor_msgs::PointCloud2> m_subPoints;
  message_filters::Subscriber<nav_msgs::Odometry> m_subOdom;

  std::shared_ptr<ApproximateSynchronizer> m_approximateSync;
  std::shared_ptr<ExactSynchronizer> m_exactSync;

  void generateMap(
    const nav_msgs::OdometryConstPtr & odom,
    const sensor_msgs::PointCloud2ConstPtr & cloud);

  LidarMapping m_lidarMapping;

  bool m_useExactSync = false;
  int m_queueSize = 5;
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_MAPPING_NODE_HPP_