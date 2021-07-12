
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

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
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>

//local lib
#include "floam/lidar_scanner_node.hpp"
#include "floam/lidar.hpp"
#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

ScanningLidarNode::ScanningLidarNode()
{
  // constructor
}

ScanningLidarNode::~ScanningLidarNode()
{
  // destructor
}

void ScanningLidarNode::onInit()
{
    m_nodeHandle = getPrivateNodeHandle();
    // defaults
    std::string points_topic = "points";
    int scan_lines = 64;
    double vertical_angle = 2.0;
    double horizontal_angle = 360.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double edgeThreshold = 0.05;

    m_nodeHandle.getParam("points_topic", points_topic);
    m_nodeHandle.getParam("scan_period", scan_period);
    m_nodeHandle.getParam("vertical_angle", vertical_angle);
    m_nodeHandle.getParam("horizontal_angle", horizontal_angle);
    m_nodeHandle.getParam("max_dis", max_dis);
    m_nodeHandle.getParam("min_dis", min_dis);
    m_nodeHandle.getParam("scan_lines", scan_lines);
    m_nodeHandle.getParam("edgeThreshold", edgeThreshold);

    m_lidar.m_settings.period = scan_period;
    m_lidar.m_settings.lines = scan_lines;
    m_lidar.m_settings.common.limits.edgeThreshold = edgeThreshold;
    m_lidar.m_settings.common.fov.vertical = vertical_angle;
    m_lidar.m_settings.common.fov.horizontal = horizontal_angle;
    m_lidar.m_settings.common.limits.distance.max = max_dis;
    m_lidar.m_settings.common.limits.distance.min = min_dis;

    m_subPoints = m_nodeHandle.subscribe(points_topic, 100, &ScanningLidarNode::handlePoints, this);

    m_pubEdgePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("points_edge", 100);
    m_pubSurfacePoints = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("points_surface", 100); 
    m_pubEdgesAndSurfaces = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("edges_and_surfaces", 100); 

    //std::thread lidar_processing_process{floam::lidar::lidar_processing};
}

void ScanningLidarNode::handlePoints(const sensor_msgs::PointCloud2ConstPtr & points)
{
  // convert msg to pcl format, only XYZ and remove NaN points
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithNaN(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  /// conver to pcl format
  pcl::fromROSMsg(*points, *cloudWithNaN);
  std::vector<int> indices;
  /// remove NaN's from pointcloud
  pcl::removeNaNFromPointCloud(*cloudWithNaN, *cloud, indices);

  // initialize timers to calculate how long the processing takes
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();

  // initialize edge and surface clouds
  pcl::PointCloud<pcl::PointXYZL>::Ptr edgesAndSurfaces(new pcl::PointCloud<pcl::PointXYZL>());          
  pcl::PointCloud<pcl::PointXYZL>::Ptr surfaces(new pcl::PointCloud<pcl::PointXYZL>());
  pcl::PointCloud<pcl::PointXYZL>::Ptr edges(new pcl::PointCloud<pcl::PointXYZL>());

  // for scanners, edge pointcloud should also include surface information
  m_lidar.detectEdges(cloud, edgesAndSurfaces);
  // m_lidar.detectSurfaces(cloud, normals);

  // separate edge cloud into edge and surface clouds
  auto edge_cond = pcl::make_shared<floam::lidar::GenericCondition<pcl::PointXYZL>>(
    [](const pcl::PointXYZL & point) {
      return point.label > 0;
    }
  );

  auto surface_cond = pcl::make_shared<floam::lidar::GenericCondition<pcl::PointXYZL>>(
    [](const pcl::PointXYZL & point) {
      return point.label == 0;
    }
  );

  // remove surfaces to get edges
  pcl::ConditionalRemoval<pcl::PointXYZL> surface_removal;
  pcl::ConditionalRemoval<pcl::PointXYZL> edge_removal;

  surface_removal.setCondition(edge_cond);
  surface_removal.setInputCloud(edgesAndSurfaces);
  surface_removal.filter(*edges);

  edge_removal.setCondition(surface_cond);
  edge_removal.setInputCloud(edgesAndSurfaces);
  edge_removal.filter(*surfaces);


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

  // convert edge pcl to ROS message
  sensor_msgs::PointCloud2 edgePoints;
  pcl::toROSMsg(*edges, edgePoints);

  // convert surface pcl to ROS message
  sensor_msgs::PointCloud2 surfacePoints;
  pcl::toROSMsg(*surfaces, surfacePoints);

  // convert edges and surfaces pcl to ROS message
  sensor_msgs::PointCloud2 edgesAndSurfaceMsg;
  pcl::toROSMsg(*edgesAndSurfaces, edgesAndSurfaceMsg);

  // set header information
  edgePoints.header = points->header;
  surfacePoints.header = points->header;
  edgesAndSurfaceMsg.header = points->header;

  // publish filtered, edge and surface clouds
  m_pubEdgePoints.publish(edgePoints);
  m_pubSurfacePoints.publish(surfacePoints);
  m_pubEdgesAndSurfaces.publish(edgesAndSurfaceMsg);
}

}  // namespace lidar
}  // namespace floam

#include <pluginlib/class_list_macros.h>  // NO LINT
PLUGINLIB_EXPORT_CLASS(floam::lidar::ScanningLidarNode, nodelet::Nodelet)