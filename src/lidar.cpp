#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include "floam/lidar_utils.hpp"
#include "floam/lidar.hpp"

namespace floam
{
namespace lidar
{

/// Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Label>::Ptr & edges)
{
  // fill this in later
}

/// Imager type
template <>
void Lidar<floam::lidar::Imager>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Label>::Ptr & edges)
{
  // pointcloud in has to be organized
  pcl::OrganizedEdgeBase <pcl::PointXYZ, pcl::Label> edgeDetector;
  std::vector<pcl::PointIndices> indicies;
  edgeDetector.setInputCloud(points);
  // TODO(flynneva): make these adjustable
  edgeDetector.setDepthDisconThreshold(0.02f);
  edgeDetector.setMaxSearchNeighbors(50);
  // calculate edges
  edgeDetector.compute(*edges, indicies);
}

/// Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals)
{
  // fill this in later
}

/// overloaded for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalDetector;

  // only for structured pointclouds
  normalDetector.setNormalEstimationMethod(normalDetector.AVERAGE_3D_GRADIENT);
  // TODO(flynneva): make these adjustable
  normalDetector.setMaxDepthChangeFactor(0.02f);
  normalDetector.setNormalSmoothingSize(7.0f);
  normalDetector.setInputCloud(points);

  normalDetector.compute(*normals);
}

}  // namespace lidar
}  // namespace floam
