
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef FLOAM__ODOM_ESTIMATION_NODE_HPP_
#define FLOAM__ODOM_ESTIMATION_NODE_HPP_

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"


#include "floam/lidar_utils.hpp"
#include "floam/odom_estimation.hpp"

namespace floam
{
namespace odom
{

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactSyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproximateSyncPolicy;

class OdomEstimationNode : public nodelet::Nodelet
{
public:
  ///
  /// OdomEstimationNode constructor
  ///
  OdomEstimationNode();

  ///
  /// OdomEstimationNode constructor
  ///
  ~OdomEstimationNode();

  ///
  /// Initialize Nodelet member variables
  ///
  /// @return void
  ///
  void onInit();

private:
  ros::NodeHandle m_nodeHandle;

  ros::Publisher m_pubLidarOdometry;

  /// ROS Transorm
  geometry_msgs::TransformStampedPtr m_tfGlobal;

  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSynchronizer;
  typedef message_filters::Synchronizer<ExactSyncPolicy> ExactSynchronizer;

  message_filters::Subscriber<sensor_msgs::PointCloud2> m_subEdges;
  message_filters::Subscriber<sensor_msgs::PointCloud2> m_subSurfaces;

  std::shared_ptr<ApproximateSynchronizer> m_approximateSync;
  std::shared_ptr<ExactSynchronizer> m_exactSync;

  void handleClouds(
    const sensor_msgs::PointCloud2ConstPtr & edges,
    const sensor_msgs::PointCloud2ConstPtr & surfaces);

  bool m_isInitialized = false;
  bool m_useExactSync = false;

  int m_queueSize = 5;

  double m_mapResolution = 0.4;

  std::string m_frameId, m_parentFrameId;

  floam::lidar::Total m_totals;

private:
  floam::odom::OdomEstimation m_odomEstimation;
};

}  // namespace odom
}  // namespace floam

#endif  // FLOAM__ODOM_ESTIMATION_NODE_HPP_