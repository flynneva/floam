// Significant re-wrte by Evan Flynn 2021

// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar_imager.hpp"


namespace floam
{
namespace lidar
{


class ImagingLidarNode : public nodelet::Nodelet
{
public:
  ImagingLidarNode();
  ~ImagingLidarNode();

  void onInit();

  void handlePoints(const sensor_msgs::PointCloud2ConstPtr &lidarCloudMsg);

  double total_time = 0;
  int frame_count = 0;

private:
  ros::NodeHandle m_nodeHandle;

  ros::Subscriber m_subPoints;

  ros::Publisher m_pubEdgePoints;
  ros::Publisher m_pubSurfacePoints;
  ros::Publisher m_pubPointsFiltered;

  ImagingLidar m_lidar;

  // std::mutex m_mutexLock;
  std::queue<sensor_msgs::PointCloud2ConstPtr> m_points;
};

}  // namespace lidar
}  // namespace floam