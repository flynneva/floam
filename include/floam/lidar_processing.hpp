// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_PROCESSING_HPP_
#define FLOAM__LIDAR_PROCESSING_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "floam/lidar.hpp"

namespace floam
{
namespace lidar
{

//points covariance class
class Double2d{
public:
	int id;
	double value;
	Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};


class LidarProcessing 
{
    public:
    	LidarProcessing();
		void init(lidar::Lidar lidar_param_in);
		void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);	
	private:
     	lidar::Lidar lidar_param;
};
}  // namespace lidar
}  // namesapce floam

#endif  // FLOAM__LIDAR_PROCESSING_HPP_

