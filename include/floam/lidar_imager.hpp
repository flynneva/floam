#ifndef FLOAM__LIDAR_IMAGER_HPP_
#define FLOAM__LIDAR_IMAGER_HPP_
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

struct Imager {
  Type type{Type::imager};
  double frame_rate{0.0};
  FOV fov;  // degrees
  AngularResolution angular; // degrees
  Limits limits;
};

class ImagingLidar
{
  public:
    ImagingLidar();
    ~ImagingLidar();

    Imager m_settings;

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>
    createSurfaceNormalsBase(const pcl::PointCloud<pcl::PointXYZ>::Ptr & points);

    pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> 
    createEdgeBase(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pc_in);
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_IMAGER_HPP_

