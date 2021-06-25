// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "floam/lidar_mapping.hpp"

namespace floam
{
namespace lidar
{

LidarMapping::LidarMapping()
{
  // constructor
}

void LidarMapping::init(double map_resolution)
{
	//init map
	//init can have real object, but future added block does not need
	for(int i=0;i<LIDAR_CELL_RANGE_HORIZONTAL*2+1;i++){
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
		for(int j=0;j<LIDAR_CELL_RANGE_HORIZONTAL*2+1;j++){
			std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
			for(int k=0;k<LIDAR_CELL_RANGE_VERTICAL*2+1;k++){
				pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
				map_depth_temp.push_back(point_cloud_temp);	
			}
			map_height_temp.push_back(map_depth_temp);
		}
		map.push_back(map_height_temp);
	}

	origin_in_map_x = LIDAR_CELL_RANGE_HORIZONTAL;
	origin_in_map_y = LIDAR_CELL_RANGE_HORIZONTAL;
	origin_in_map_z = LIDAR_CELL_RANGE_VERTICAL;
	map_width = LIDAR_CELL_RANGE_HORIZONTAL*2+1;
	map_height = LIDAR_CELL_RANGE_HORIZONTAL*2+1;
	map_depth = LIDAR_CELL_RANGE_HORIZONTAL*2+1;

	//downsampling size
	downSizeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);
}

void LidarMapping::addWidthCellNegative(void)
{
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.insert(map.begin(), map_height_temp);

	origin_in_map_x++;
	map_width++;
}

void LidarMapping::addWidthCellPositive(void)
{
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.push_back(map_height_temp);
	map_width++;
}
void LidarMapping::addHeightCellNegative(void)
{
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].insert(map[i].begin(), map_depth_temp);
	}
	origin_in_map_y++;
	map_height++;
}
void LidarMapping::addHeightCellPositive(void)
{
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].push_back(map_depth_temp);
	}
	map_height++;
}
void LidarMapping::addDepthCellNegative(void)
{
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].insert(map[i][j].begin(), point_cloud_temp);
		}
	}
	origin_in_map_z++;
	map_depth++;
}
void LidarMapping::addDepthCellPositive(void)
{
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
			map[i][j].push_back(point_cloud_temp);
		}
	}
	map_depth++;
}

//extend map is points exceed size
void LidarMapping::checkPoints(int& x, int& y, int& z)
{

	while(x + LIDAR_CELL_RANGE_HORIZONTAL> map_width-1){
		addWidthCellPositive();
	}
	while(x-LIDAR_CELL_RANGE_HORIZONTAL<0){
		addWidthCellNegative();
		x++;
	}
	while(y + LIDAR_CELL_RANGE_HORIZONTAL> map_height-1){
		addHeightCellPositive();
	}
	while(y-LIDAR_CELL_RANGE_HORIZONTAL<0){
		addHeightCellNegative();
		y++;
	}
	while(z + LIDAR_CELL_RANGE_VERTICAL> map_depth-1){
		addDepthCellPositive();
	}
	while(z-LIDAR_CELL_RANGE_VERTICAL<0){
		addDepthCellNegative();
		z++;
	}

	//initialize 
	//create object if area is null
	for(int i=x-LIDAR_CELL_RANGE_HORIZONTAL;i<x+LIDAR_CELL_RANGE_HORIZONTAL+1;i++){
		for(int j=y-LIDAR_CELL_RANGE_HORIZONTAL;j<y+LIDAR_CELL_RANGE_HORIZONTAL+1;j++){
			for(int k=z-LIDAR_CELL_RANGE_VERTICAL;k<z+LIDAR_CELL_RANGE_VERTICAL+1;k++){
				if(map[i][j][k]==NULL){
					pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
					map[i][j][k] = point_cloud_temp;
				}

			}
				
		}

	}
}

//update points to map 
void LidarMapping::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current)
{
	int currentPosIdX = int(std::floor(pose_current.translation().x() / LIDAR_CELL_WIDTH + 0.5)) + origin_in_map_x;
	int currentPosIdY = int(std::floor(pose_current.translation().y() / LIDAR_CELL_HEIGHT + 0.5)) + origin_in_map_y;
	int currentPosIdZ = int(std::floor(pose_current.translation().z() / LIDAR_CELL_DEPTH + 0.5)) + origin_in_map_z;

	//check is submap is null
	checkPoints(currentPosIdX,currentPosIdY,currentPosIdZ);

	pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());
	
	//save points
	for (int i = 0; i < (int)transformed_pc->points.size(); i++) {
		pcl::PointXYZI point_temp = transformed_pc->points[i];
		//for visualization only
		point_temp.intensity = std::min(1.0 , std::max(pc_in->points[i].z+2.0, 0.0) / 5);
		int currentPointIdX = int(std::floor(point_temp.x / LIDAR_CELL_WIDTH + 0.5)) + origin_in_map_x;
		int currentPointIdY = int(std::floor(point_temp.y / LIDAR_CELL_HEIGHT + 0.5)) + origin_in_map_y;
		int currentPointIdZ = int(std::floor(point_temp.z / LIDAR_CELL_DEPTH + 0.5)) + origin_in_map_z;

		map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
	}
	
	//filtering points 
	for(int i = currentPosIdX - LIDAR_CELL_RANGE_HORIZONTAL; i < currentPosIdX + LIDAR_CELL_RANGE_HORIZONTAL + 1; i++) {
		for(int j = currentPosIdY - LIDAR_CELL_RANGE_HORIZONTAL; j < currentPosIdY + LIDAR_CELL_RANGE_HORIZONTAL + 1; j++) {
			for(int k = currentPosIdZ - LIDAR_CELL_RANGE_VERTICAL; k < currentPosIdZ + LIDAR_CELL_RANGE_VERTICAL + 1; k++) {
				downSizeFilter.setInputCloud(map[i][j][k]);
				downSizeFilter.filter(*(map[i][j][k]));
			}
				
		}

	}

}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarMapping::getMap(void)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr LidarCloudMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new  pcl::PointCloud<pcl::PointXYZI>());
	for (int i = 0; i < map_width; i++) {
		for (int j = 0; j < map_height; j++) {
			for (int k = 0; k < map_depth; k++) {
				if(map[i][j][k]!=NULL){
					*LidarCloudMap += *(map[i][j][k]);
				}
			}
		}
	}
	return LidarCloudMap;
}

}  // namespace lidar
}  // namespace floam

