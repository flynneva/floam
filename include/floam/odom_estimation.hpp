
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef FLOAM__ODOM_ESTIMATION_HPP_
#define FLOAM__ODOM_ESTIMATION_HPP_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "floam/lidar_optimization.hpp"
#include "floam/lidar_utils.hpp"

namespace floam
{
namespace odom
{

/// Odometry Estimation Class
///
/// Can be used across all lidar types
///
class OdomEstimation
{
public:
  /// constructor / destructor
  OdomEstimation();
  ~OdomEstimation() {};

  void init(const double & mapResolution);	
  void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr & edges, const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfaces);
  void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr & edges, const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfaces);
  void getMap(pcl::PointCloud<pcl::PointXYZ>::Ptr & lidarCloudMap);

  /// Odometry
  Eigen::Isometry3d m_odom, m_lastOdom;

  /// Optimization parameters that ceres solvers update each loop
  double m_parameters[7] = {0, 0, 0, 1, 0, 0, 0};

  Eigen::Map<Eigen::Quaterniond> m_currentRotation = Eigen::Map<Eigen::Quaterniond>(m_parameters);
  Eigen::Map<Eigen::Vector3d> m_currentTranslation = Eigen::Map<Eigen::Vector3d>(m_parameters + 4);

  /// kd-tree
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdTreeEdgeMap;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr m_kdTreeSurfMap;

  /// corner and surface map objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_lidarCloudCornerMap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_lidarCloudSurfMap;

  /// points downsampling before add to map
  pcl::VoxelGrid<pcl::PointXYZ> m_downSizeFilterEdge;
  pcl::VoxelGrid<pcl::PointXYZ> m_downSizeFilterSurf;

  /// local map
  pcl::CropBox<pcl::PointXYZ> m_cropBoxFilter;

  /// optimization count 
  int m_optimizationCount;

  /// totals counters (frames, time)
  floam::lidar::Total m_totals;

  /// function
  void addEdgeCostFactor(
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
	  ceres::Problem & problem,
	  ceres::LossFunction * lossFunction);

  void addSurfCostFactor(
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
	  ceres::Problem & problem,
	  ceres::LossFunction * lossFunction);

  // TODO(flynneva): are these even used? I think these were moved to the mapping class
  void addPointsToMap(
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & downsampledEdgeCloud,
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & downsampledSurfCloud);

  // TODO(flynneva): are these even used? I think these were moved to the mapping class
  void pointAssociateToMap(
	  const std::shared_ptr<pcl::PointXYZ> & pointsIn,
	  std::shared_ptr<pcl::PointXYZ> & pointsOut);

  // TODO(flynneva): are these even used? I think these were moved to the mapping class
  void downSamplingToMap(
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edgesIn,
	  pcl::PointCloud<pcl::PointXYZ>::Ptr & edgesOut,
	  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfacesIn,
	  pcl::PointCloud<pcl::PointXYZ>::Ptr & surfacesOut);
};
}  // namespace odom
}  // namespace floam

#endif  // FLOAM__ODOM_ESTIMATION_HPP_

