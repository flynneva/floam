
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


#define LIDAR_CELL_WIDTH 50.0
#define LIDAR_CELL_HEIGHT 50.0
#define LIDAR_CELL_DEPTH 50.0

//separate map as many sub point clouds

#define LIDAR_CELL_RANGE_HORIZONTAL 2
#define LIDAR_CELL_RANGE_VERTICAL 2


namespace floam
{
namespace lidar
{

class LidarMapping 
{

    public:
    	LidarMapping();
		void init(double map_resolution);
		void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZL>::Ptr& pc_in, const Eigen::Isometry3d& pose_current);
		pcl::PointCloud<pcl::PointXYZL>::Ptr getMap(void);

	private:
		int origin_in_map_x;
		int origin_in_map_y;
		int origin_in_map_z;
		int map_width;
		int map_height;
		int map_depth;
		std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr>>> map;
		pcl::VoxelGrid<pcl::PointXYZL> downSizeFilter;
		
		void addWidthCellNegative(void);
		void addWidthCellPositive(void);
		void addHeightCellNegative(void);
		void addHeightCellPositive(void);
		void addDepthCellNegative(void);
		void addDepthCellPositive(void);
		void checkPoints(int& x, int& y, int& z);

};
}  // namespace lidar
}  // namespace floam

#endif // FLOAM__LIDAR_MAPPING_HPP_

