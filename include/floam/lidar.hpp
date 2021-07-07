
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_HPP_
#define FLOAM__LIDAR_HPP_
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{


template <class T>
class Lidar
{
public:
  ///  Detects surface normals from input pointcloud
  ///
  /// @param points input pointcloud
  /// @param edges output edges
  ///
  void detectSurfaces(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
    const pcl::PointCloud<pcl::Normal>::Ptr & normals);

  /// Detects edges from input pointcloud
  ///
  /// @param points input pointcloud
  /// @param edges output edges
  ///
  void detectEdges(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
    const pcl::PointCloud<pcl::Label>::Ptr & edges);

  /// Settings for specific Lidar type
  T m_setting;
  /// Total counters (i.e. frames and time)
  floam::lidar::Total m_total;
};

/// overload detectSurfaces for Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals);

/// overload detectSurfaces for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Normal>::Ptr & normals);

/// overload detectEdges for Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Label>::Ptr & edges);

/// overload detectEdges for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::Label>::Ptr & edges);

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_HPP_