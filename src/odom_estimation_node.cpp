

/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/odom_estimation_node.hpp"
#include "floam/odom_estimation.hpp"

namespace floam
{
namespace odom
{

OdomEstimationNode::OdomEstimationNode()
{
  // constructor
}

OdomEstimationNode::~OdomEstimationNode()
{
  // destructor
}

void OdomEstimationNode::onInit()
{
  m_nodeHandle = getPrivateNodeHandle();

  m_nodeHandle.getParam("use_exact_sync", m_useExactSync);
  m_nodeHandle.getParam("queue_size", m_queueSize);
  m_nodeHandle.getParam("map_resolution", m_mapResolution);
  m_nodeHandle.getParam("frame_id", m_frameId);
  m_nodeHandle.getParam("parent_frame_id", m_parentFrameId);

  m_odomEstimation.init(m_mapResolution);


  ROS_INFO_STREAM(m_nodeHandle.getNamespace() << "/frame_id: " << m_frameId);
  ROS_INFO_STREAM(m_nodeHandle.getNamespace() << "/parent_frame_id: " << m_parentFrameId);
  ROS_INFO_STREAM(m_nodeHandle.getNamespace() << "/use_exact_sync: " << m_useExactSync);
  ROS_INFO_STREAM(m_nodeHandle.getNamespace() << "/queue_size: " << m_queueSize);
  ROS_INFO_STREAM(m_nodeHandle.getNamespace() << "/map_resolution: " << m_mapResolution);

  m_tfGlobal.reset(new geometry_msgs::TransformStamped());

  m_tfGlobal->child_frame_id = m_frameId;

  // should these topic names be parameters instead of remapped?
  m_subEdges.subscribe(m_nodeHandle, "points_edge", 100);
  m_subSurfaces.subscribe(m_nodeHandle, "points_surface", 100);
 
  m_pubLidarOdometry = m_nodeHandle.advertise<nav_msgs::Odometry>("odom", 100);
  
  // initialize callbacks using sync policy
  if (m_useExactSync) {
    ROS_INFO("Exact Synchronization Policy chosen");
    m_exactSync.reset(new ExactSynchronizer(ExactSyncPolicy(m_queueSize), m_subEdges, m_subSurfaces));
    m_exactSync->registerCallback(
      std::bind(&OdomEstimationNode::handleClouds, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    ROS_INFO("Approximate Synchronization Policy chosen");
    m_approximateSync.reset(new ApproximateSynchronizer(ApproximateSyncPolicy(m_queueSize), m_subEdges, m_subSurfaces));
    m_approximateSync->registerCallback(
      std::bind(&OdomEstimationNode::handleClouds, this, std::placeholders::_1, std::placeholders::_2));
  }
}

void OdomEstimationNode::handleClouds(
  const sensor_msgs::PointCloud2ConstPtr & edges_msg,
  const sensor_msgs::PointCloud2ConstPtr & surfaces_msg)
{
  // convert to PCL msgs
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*edges_msg, *pointcloud_edge_in);
  pcl::fromROSMsg(*surfaces_msg, *pointcloud_surf_in);

  // get timestamp from edges pointcloud msg, should (always) be the same as
  ros::Time pointcloud_time = edges_msg->header.stamp;

  // check if odometry is initialized
  if (m_isInitialized == false) {
    m_odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
    m_isInitialized = true;
    ROS_INFO("odometry initialized");  // should only be called once
  } else {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    m_odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
    end = std::chrono::system_clock::now();
    // calculate elapsted time
    std::chrono::duration<float> elapsed_seconds = end - start;
    m_totals.frames++;
    float time_temp = elapsed_seconds.count() * 1000;
    m_totals.time += time_temp;
    ROS_INFO("average odom estimation time %f ms", m_totals.time / m_totals.frames);
  }

  /// get current odometry estimation
  m_tfGlobal->transform.translation.x = m_odomEstimation.m_currentTranslation.x();
  m_tfGlobal->transform.translation.y = m_odomEstimation.m_currentTranslation.y();
  m_tfGlobal->transform.translation.z = m_odomEstimation.m_currentTranslation.z();

  m_tfGlobal->transform.rotation.x = m_odomEstimation.m_currentRotation.x();
  m_tfGlobal->transform.rotation.y = m_odomEstimation.m_currentRotation.y();
  m_tfGlobal->transform.rotation.z = m_odomEstimation.m_currentRotation.z();
  m_tfGlobal->transform.rotation.w = m_odomEstimation.m_currentRotation.w();

  /// broadcast odom transform
  static tf2_ros::TransformBroadcaster br;
  m_tfGlobal->header.stamp = edges_msg->header.stamp;
  m_tfGlobal->header.frame_id = m_parentFrameId;
  br.sendTransform(*m_tfGlobal);

  // publish odometry
  nav_msgs::Odometry lidarOdometry;
  lidarOdometry.header.frame_id = m_parentFrameId;
  lidarOdometry.child_frame_id = m_frameId;
  lidarOdometry.header.stamp = pointcloud_time;
  lidarOdometry.pose.pose.orientation.x = m_tfGlobal->transform.rotation.x;
  lidarOdometry.pose.pose.orientation.y = m_tfGlobal->transform.rotation.y;
  lidarOdometry.pose.pose.orientation.z = m_tfGlobal->transform.rotation.z;
  lidarOdometry.pose.pose.orientation.w = m_tfGlobal->transform.rotation.w;
  lidarOdometry.pose.pose.position.x = m_tfGlobal->transform.translation.x;
  lidarOdometry.pose.pose.position.y = m_tfGlobal->transform.translation.y;
  lidarOdometry.pose.pose.position.z = m_tfGlobal->transform.translation.z;
  m_pubLidarOdometry.publish(lidarOdometry);
}

}  // namespace odom
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::odom::OdomEstimationNode, nodelet::Nodelet)
