#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include "floam/lidar_imager.hpp"

namespace floam
{
namespace lidar
{

ImagingLidar::ImagingLidar()
{
  // constructor
};

ImagingLidar::~ImagingLidar()
{
  // destructor
};

pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>
ImagingLidar::createSurfaceNormalsBase(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normals_est;

  // only for structured pointclouds
  normals_est.setNormalEstimationMethod(normals_est.AVERAGE_3D_GRADIENT);
  // TODO(flynneva): make these adjustable
  normals_est.setMaxDepthChangeFactor(0.02f);
  normals_est.setNormalSmoothingSize(7.0f);
  normals_est.setInputCloud(points);
  return normals_est;
}

pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> 
ImagingLidar::createEdgeBase(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pc_in)
{
  // pointcloud in has to be organized
  pcl::OrganizedEdgeBase <pcl::PointXYZ, pcl::Label> edgeDetector;

  edgeDetector.setInputCloud(pc_in);

  // TODO(flynneva): make these adjustable
  edgeDetector.setDepthDisconThreshold(0.02f);
  edgeDetector.setMaxSearchNeighbors(50);

  return edgeDetector;
}

}  // namespace lidar
}  // namespace floam


