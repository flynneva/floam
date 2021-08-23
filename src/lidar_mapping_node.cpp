
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar_mapping_node.hpp"
#include "floam/lidar_mapping.hpp"

namespace floam
{
namespace lidar
{

LidarMappingNode::LidarMappingNode()
{
  // constructor
}

LidarMappingNode::~LidarMappingNode()
{
  // destructor
}

void LidarMappingNode::onInit()
{
  m_nodeHandle = getPrivateNodeHandle();

  double map_resolution = 0.4;

  m_nodeHandle.getParam("use_exact_sync", m_useExactSync);
  m_nodeHandle.getParam("queue_size", m_queueSize);
  m_nodeHandle.getParam("map_resolution", map_resolution);
  
  m_lidarMapping.init(map_resolution);

  // should these topic names be parameters instead of remapped?
  m_subPoints.subscribe(m_nodeHandle, "points_filtered", 1000);
  m_subOdom.subscribe(m_nodeHandle, "odom", 1000);
  
  m_pubMap = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("map", 100);

  // initialize callbacks using sync policy
  if (m_useExactSync) {
    ROS_INFO("Exact Synchronization Policy chosen for Mapping");
    m_exactSync.reset(new ExactSynchronizer(ExactSyncPolicy(m_queueSize), m_subOdom, m_subPoints));
    m_exactSync->registerCallback(
      std::bind(&LidarMappingNode::generateMap, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    ROS_INFO("Approximate Synchronization Policy chosen for Mapping");
    m_approximateSync.reset(new ApproximateSynchronizer(ApproximateSyncPolicy(m_queueSize), m_subOdom, m_subPoints));
    m_approximateSync->registerCallback(
      std::bind(&LidarMappingNode::generateMap, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void LidarMappingNode::generateMap(
  const nav_msgs::OdometryConstPtr & odom,
  const sensor_msgs::PointCloud2ConstPtr & cloud)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZL>());
  pcl::fromROSMsg(*cloud, *pointcloud_in);
  ros::Time pointcloud_time = cloud->header.stamp;

  Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();

  current_pose.rotate(
    Eigen::Quaterniond(
      odom->pose.pose.orientation.w,
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z));

  current_pose.pretranslate(
    Eigen::Vector3d(
      odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z));

  m_lidarMapping.updateCurrentPointsToMap(pointcloud_in, current_pose);
  pcl::PointCloud<pcl::PointXYZL>::Ptr pc_map(new pcl::PointCloud<pcl::PointXYZL>());
  pc_map = m_lidarMapping.getMap();
  sensor_msgs::PointCloud2 PointsMsg;
  pcl::toROSMsg(*pc_map, PointsMsg);
  PointsMsg.header.stamp = pointcloud_time;
  PointsMsg.header.frame_id = "map";
  m_pubMap.publish(PointsMsg);
}

}  // namespace lidar
}  // namespace floam


#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::lidar::LidarMappingNode, nodelet::Nodelet);
