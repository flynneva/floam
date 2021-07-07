
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
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
#include "floam/lidar_imager_node.hpp"
#include "floam/lidar.hpp"


namespace floam
{
namespace lidar
{

ImagingLidarNode::ImagingLidarNode()
{
  // constructor
}

ImagingLidarNode::~ImagingLidarNode()
{
  // destructor
}

void ImagingLidarNode::onInit()
{
    ROS_INFO("INIT");
    m_nodeHandle = getPrivateNodeHandle();

    std::string points_topic = "points";
    double vertical_angle = 2.0;
    double horizontal_angle = 120.0;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double framerate = 30.0;

    m_nodeHandle.getParam("points_topic", points_topic);
    m_nodeHandle.getParam("vertical_angle", vertical_angle);
    m_nodeHandle.getParam("horizontal_angle", horizontal_angle);
    m_nodeHandle.getParam("max_dis", max_dis);
    m_nodeHandle.getParam("min_dis", min_dis);
    m_nodeHandle.getParam("framerate", framerate);

    m_lidar.m_setting.framerate = framerate;
    m_lidar.m_setting.common.fov.vertical = vertical_angle;
    m_lidar.m_setting.common.fov.horizontal = horizontal_angle;
    m_lidar.m_setting.common.limits.distance.max = max_dis;
    m_lidar.m_setting.common.limits.distance.min = min_dis;
    
    ROS_INFO("SUBSCRIBE");
    m_subPoints = m_nodeHandle.subscribe(points_topic, 100, &ImagingLidarNode::handlePoints, this);

    m_pubEdgePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("points_edge", 100);

    m_pubSurfacePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("points_surface", 100); 
}

void ImagingLidarNode::handlePoints(const sensor_msgs::PointCloud2ConstPtr & points)
{
  // convert msg to pcl format, only XYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*points, *cloud);

  // initialize timers to calculate how long the processing takes
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  // initialize edge and surface clouds
  pcl::PointCloud<pcl::PointXYZL>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZL>());          
  pcl::PointCloud<pcl::PointNormal>::Ptr pointcloud_surface(new pcl::PointCloud<pcl::PointNormal>());

  // compute edges and surfaces
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud <pcl::Label>::Ptr edges(new pcl::PointCloud<pcl::Label>);

  m_lidar.detectSurfaces(cloud, normals);
  m_lidar.detectEdges(cloud, edges);

  std::vector <pcl::PointIndices> labelIndices;

  // end processing time
  end = std::chrono::system_clock::now();
  std::chrono::duration<float> elapsed_seconds = end - start;

  // increment total frame counter
  m_lidar.m_total.frames++;
  // add time to total time
  float time_temp = elapsed_seconds.count() * 1000;
  m_lidar.m_total.time += time_temp;
  // ROS_INFO("average lidar processing time %f ms", m_lidar.m_total.time / m_lidar.m_total.frames);

  // combine xyz cloud with surface normals and edges
  pcl::concatenateFields(*cloud, *edges, *pointcloud_edge);
  pcl::concatenateFields(*cloud, *normals, *pointcloud_surface);

  // convert edge pcl to ROS message
  sensor_msgs::PointCloud2 edgePoints;
  pcl::toROSMsg(*pointcloud_edge, edgePoints);

  // convert surface pcl to ROS message
  sensor_msgs::PointCloud2 surfacePoints;
  pcl::toROSMsg(*pointcloud_surface, surfacePoints);

  // set header information
  edgePoints.header = points->header;
  surfacePoints.header = points->header;

  // publish filtered, edge and surface clouds
  m_pubEdgePoints.publish(edgePoints);
  m_pubSurfacePoints.publish(surfacePoints);
}

}  // namespace lidar
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::lidar::ImagingLidarNode, nodelet::Nodelet)