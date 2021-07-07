

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

  int map_resolution;

  m_nodeHandle.getParam("use_exact_sync", m_useExactSync);
  m_nodeHandle.getParam("queue_size", m_queueSize);
  m_nodeHandle.getParam("map_resolution", map_resolution);

  m_odomEstimation.init(map_resolution);

  // should these topic names parameters?
  message_filters::Subscriber<sensor_msgs::PointCloud2> subEdges(m_nodeHandle, "points_edge", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subSurfaces(m_nodeHandle, "points_surface", 100);
 
  m_pubLidarOdometry = m_nodeHandle.advertise<nav_msgs::Odometry>("odom", 100);
  
  if (m_useExactSync) {
    ROS_INFO("Exact Synchronization Policy chosen");
    m_exactSync.reset(new ExactSynchronizer(ExactSyncPolicy(m_queueSize), subEdges, subSurfaces));
    m_exactSync->registerCallback(
      std::bind(&OdomEstimationNode::handleClouds, this, std::placeholders::_1, std::placeholders::_2));
  } else {
    ROS_INFO("Approximate Synchronization Policy chosen");
  }
}

void OdomEstimationNode::handleClouds(
  const sensor_msgs::PointCloud2ConstPtr & edges_msg,
  const sensor_msgs::PointCloud2ConstPtr & surfaces_msg)
{
  // convert to PCL msgs
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*edges_msg, *pointcloud_edge_in);
  pcl::fromROSMsg(*surfaces_msg, *pointcloud_surf_in);

  // get timestamp from edges pointcloud msg
  // option to use surfaces_msg stamp instead? average them?
  ros::Time pointcloud_time = edges_msg->header.stamp;

  // check if odometry is initialized
  if (m_isInitialized == false) {
      m_odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
      m_isInitialized = true;
      ROS_INFO("odometry initialized");
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

  Eigen::Quaterniond q_current(m_odomEstimation.m_odom.rotation());
  //q_current.normalize();
  Eigen::Vector3d t_current = m_odomEstimation.m_odom.translation();
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()) );
  tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
  // publish odometry
  nav_msgs::Odometry lidarOdometry;
  lidarOdometry.header.frame_id = "map";
  lidarOdometry.child_frame_id = "base_link";
  lidarOdometry.header.stamp = pointcloud_time;
  lidarOdometry.pose.pose.orientation.x = q_current.x();
  lidarOdometry.pose.pose.orientation.y = q_current.y();
  lidarOdometry.pose.pose.orientation.z = q_current.z();
  lidarOdometry.pose.pose.orientation.w = q_current.w();
  lidarOdometry.pose.pose.position.x = t_current.x();
  lidarOdometry.pose.pose.position.y = t_current.y();
  lidarOdometry.pose.pose.position.z = t_current.z();
  m_pubLidarOdometry.publish(lidarOdometry);
}

}  // namespace odom
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::odom::OdomEstimationNode, nodelet::Nodelet)