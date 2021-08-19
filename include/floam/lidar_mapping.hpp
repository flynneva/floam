
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_MAPPING_HPP_
#define FLOAM__LIDAR_MAPPING_HPP_

//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//c++ lib
#include <string>
#include <math.h>
#include <vector>


#define CELL_WIDTH 50.0  // m
#define CELL_HEIGHT 50.0  // m
#define CELL_DEPTH 50.0  //  m

#define CELL_RANGE_HORIZONTAL 4
#define CELL_RANGE_VERTICAL 4
#define HALF_CELL_RANGE_HORIZONTAL CELL_RANGE_HORIZONTAL / 2
#define HALF_CELL_RANGE_VERTICAL CELL_RANGE_VERTICAL / 2


namespace floam
{
namespace lidar
{

class LidarMapping 
{
public:
  LidarMapping();
  void init(const double & map_resolution);
  void updateCurrentPointsToMap(
    const pcl::PointCloud<pcl::PointXYZL>::Ptr & pc_in,
    const Eigen::Isometry3d & pose_current);
  pcl::PointCloud<pcl::PointXYZL>::Ptr getMap(void);

private:
  int m_originInMapX, m_originInMapY, m_originInMapZ;
  int m_mapWidth, m_mapHeight, m_mapDepth;
  std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr>>> m_map;
  pcl::VoxelGrid<pcl::PointXYZL> m_downSizeFilter;
  
  void addWidthCellNegative(void);
  void addWidthCellPositive(void);
  void addHeightCellNegative(void);
  void addHeightCellPositive(void);
  void addDepthCellNegative(void);
  void addDepthCellPositive(void);
  void checkPoints(int & x, int & y, int & z);
};

}  // namespace lidar
}  // namespace floam

#endif // FLOAM__LIDAR_MAPPING_HPP_

