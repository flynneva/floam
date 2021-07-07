
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

#include <ros/ros.h>

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
  void init(double mapResolution);	
  void initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr & edges, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surfaces);
  void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr & edges, const pcl::PointCloud<pcl::PointXYZI>::Ptr& surfaces);
  void getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr & lidarCloudMap);

  /// optimization variable
  double m_parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> m_currentQW = Eigen::Map<Eigen::Quaterniond>(m_parameters);
  Eigen::Map<Eigen::Vector3d> m_currentTW = Eigen::Map<Eigen::Vector3d>(m_parameters + 4);

  /// kd-tree
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdTreeEdgeMap;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kdTreeSurfMap;

  /// Odometry
  Eigen::Isometry3d m_odom;
  Eigen::Isometry3d m_lastOdom;

  /// corner and surface map objects
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_lidarCloudCornerMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_lidarCloudSurfMap;

  /// points downsampling before add to map
  pcl::VoxelGrid<pcl::PointXYZI> m_downSizeFilterEdge;
  pcl::VoxelGrid<pcl::PointXYZI> m_downSizeFilterSurf;

  /// local map
  pcl::CropBox<pcl::PointXYZI> m_cropBoxFilter;

  /// optimization count 
  int m_optimizationCount;

  /// function
  void addEdgeCostFactor(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & points,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & map,
	ceres::Problem& problem,
	ceres::LossFunction * lossFunction);

  void addSurfCostFactor(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & points,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & map,
	ceres::Problem & problem,
	ceres::LossFunction * lossFunction);

  void addPointsToMap(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & downsampledEdgeCloud,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & downsampledSurfCloud);

  void pointAssociateToMap(
	pcl::PointXYZI const * const pointsIn,
	pcl::PointXYZI * const pointsOut);

  void downSamplingToMap(
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & edgesIn,
	pcl::PointCloud<pcl::PointXYZI>::Ptr & edgesOut,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr & surfacesIn,
	pcl::PointCloud<pcl::PointXYZI>::Ptr & surfacesOut);
};
}  // namespace odom
}  // namespace floam

#endif  // FLOAM__ODOM_ESTIMATION_HPP_

