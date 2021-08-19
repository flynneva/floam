
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_HPP_
#define FLOAM__LIDAR_HPP_
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/conditional_removal.h>

#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

// TODO(flynneva): template the point type
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
    pcl::PointCloud<pcl::PointNormal>::Ptr & normals);

  /// Detects edges from input pointcloud
  ///
  /// @param points input pointcloud
  /// @param edges output edges
  ///
  void detectEdges(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
    pcl::PointCloud<pcl::PointXYZL>::Ptr & edges);

  /// Settings for specific Lidar type
  T m_settings;
  /// Total counters (i.e. frames and time)
  floam::lidar::Total m_total;

  /// lidarScans
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_lidarScans;
  std::vector<int> m_addedPoints;

  /// kdTree
  pcl::KdTreeFLANN<pcl::PointXYZ> m_kdTree;
  std::vector<int> pointSearch;
  std::vector<float> pointSquaredDistance;

  /// covariance objects
  Eigen::Matrix<std::complex<double>, 3, 3> m_covariance;
  Eigen::Matrix<std::complex<double>, 3, 1> m_eigenValues;

};

/// overload detectSurfaces for Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointNormal>::Ptr & normals);

/// overload detectSurfaces for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointNormal>::Ptr & normals);

/// overload detectEdges for Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointXYZL>::Ptr  & edges);

/// overload detectEdges for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointXYZL>::Ptr & edges);

/// taken from the example here (thanks, Davide!)
/// https://cpp-optimizations.netlify.app/pcl_filter/
template <typename PointT>
class GenericCondition : public pcl::ConditionBase<PointT>
{
public:
  typedef std::shared_ptr<GenericCondition<PointT>> Ptr;
  typedef std::shared_ptr<const GenericCondition<PointT>> ConstPtr;
  typedef std::function<bool(const PointT&)> FunctorT;

  GenericCondition(FunctorT evaluator): 
    pcl::ConditionBase<PointT>(),_evaluator( evaluator ) 
  {}

  virtual bool evaluate (const PointT &point) const {
    // just delegate ALL the work to the injected std::function
    return _evaluator(point);
  }
private:
  FunctorT _evaluator;
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_HPP_