#ifndef FLOAM__ODOM_ESTIMATION_NODE_HPP_
#define FLOAM__ODOM_ESTIMATION_NODE_HPP_

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

//local lib
#include "floam/lidar.hpp"
#include "floam/odom_estimation.hpp"

namespace floam
{
namespace odom
{

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
  ros::Publisher pubLidarOdometry;

  void surfaceHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);
  void edgeHandler(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);
  void odomEstimation();

  bool is_odom_inited = false;
  double total_time =0;
  int total_frame=0;

private:
  floam::odom::OdomEstimation m_odomEstimation;
  std::mutex m_mutexLock;
  std::queue<sensor_msgs::PointCloud2ConstPtr> m_pointsEdge;
  std::queue<sensor_msgs::PointCloud2ConstPtr> m_pointsSurface;
  floam::lidar::Lidar m_lidar;
};

}  // namespace odom
}  // namespace floam

#endif  // FLOAM__ODOM_ESTIMATION_NODE_HPP_