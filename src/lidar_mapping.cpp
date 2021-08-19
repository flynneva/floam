
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "floam/lidar_mapping.hpp"

#include <iostream>

namespace floam
{
namespace lidar
{

LidarMapping::LidarMapping()
{
  // constructor
}

void LidarMapping::init(const double & map_resolution)
{
	//init map
	//init can have real object, but future added block does not need
	for(int i=0; i < CELL_RANGE_HORIZONTAL; i++) {
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr>> m_mapHeight_temp;
		for(int j=0; j < CELL_RANGE_HORIZONTAL; j++) {
			std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> m_mapDepth_temp;
			for(int k=0; k < CELL_RANGE_VERTICAL; k++) {
				pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>);
				m_mapDepth_temp.push_back(point_cloud_temp);
			}
			m_mapHeight_temp.push_back(m_mapDepth_temp);
		}
		m_map.push_back(m_mapHeight_temp);
	}

	m_originInMapX = HALF_CELL_RANGE_HORIZONTAL;
	m_originInMapY = HALF_CELL_RANGE_HORIZONTAL;
	m_originInMapZ = HALF_CELL_RANGE_VERTICAL;
	m_mapWidth = CELL_RANGE_HORIZONTAL;
	m_mapHeight = CELL_RANGE_HORIZONTAL;
	m_mapDepth = CELL_RANGE_HORIZONTAL;

	//downsampling size
	m_downSizeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);
}

void LidarMapping::addWidthCellNegative(void)
{
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr>> m_mapHeight_temp;
	for(int j=0; j < m_mapHeight; j++) {
		std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> m_mapDepth_temp;
		for(int k=0; k < m_mapDepth; k++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_mapDepth_temp.push_back(point_cloud_temp);
		}
		m_mapHeight_temp.push_back(m_mapDepth_temp);
	}
	m_map.insert(m_map.begin(), m_mapHeight_temp);

	m_originInMapX++;
	m_mapWidth++;
}

void LidarMapping::addWidthCellPositive(void)
{
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr>> m_mapHeight_temp;

	for(int j=0; j < m_mapHeight; j++) {
		std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> m_mapDepth_temp;

		for(int k=0; k < m_mapDepth; k++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_mapDepth_temp.push_back(point_cloud_temp);
		}
		m_mapHeight_temp.push_back(m_mapDepth_temp);
	}
	m_map.push_back(m_mapHeight_temp);
	m_mapWidth++;
}
void LidarMapping::addHeightCellNegative(void)
{
	for(int i=0; i < m_mapWidth; i++) {
		std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> m_mapDepth_temp;

		for(int k=0; k < m_mapDepth; k++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_mapDepth_temp.push_back(point_cloud_temp);
		}
		m_map[i].insert(m_map[i].begin(), m_mapDepth_temp);
	}
	m_originInMapY++;
	m_mapHeight++;
}
void LidarMapping::addHeightCellPositive(void)
{
	for(int i=0; i < m_mapWidth; i++) {
		std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> m_mapDepth_temp;

		for(int k=0; k < m_mapDepth; k++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_mapDepth_temp.push_back(point_cloud_temp);
		}
		m_map[i].push_back(m_mapDepth_temp);
	}
	m_mapHeight++;
}
void LidarMapping::addDepthCellNegative(void)
{
	for(int i = 0; i < m_mapWidth; i++) {
		for(int j = 0; j < m_mapHeight; j++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_map[i][j].insert(m_map[i][j].begin(), point_cloud_temp);
		}
	}
	m_originInMapZ++;
	m_mapDepth++;
}
void LidarMapping::addDepthCellPositive(void)
{
	for(int i=0; i < m_mapWidth; i++) {
		for(int j=0; j < m_mapHeight; j++) {
			pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
			m_map[i][j].push_back(point_cloud_temp);
		}
	}
	m_mapDepth++;
}

//extend map is points exceed size
void LidarMapping::checkPoints(int& x, int& y, int& z)
{
	while(x + CELL_RANGE_HORIZONTAL > m_mapWidth) {
		addWidthCellPositive();
	}
	while(x - CELL_RANGE_HORIZONTAL<0) {
		addWidthCellNegative();
		x++;
	}
	while(y + CELL_RANGE_HORIZONTAL > m_mapHeight) {
		addHeightCellPositive();
	}
	while(y - CELL_RANGE_HORIZONTAL < 0) {
		addHeightCellNegative();
		y++;
	}
	while(z + CELL_RANGE_VERTICAL > m_mapDepth) {
		addDepthCellPositive();
	}
	while(z - CELL_RANGE_VERTICAL < 0) {
		addDepthCellNegative();
		z++;
	}

	//initialize 
	//create object if area is null
	for(int i = (x - CELL_RANGE_HORIZONTAL); i < (x + CELL_RANGE_HORIZONTAL); i++) {
		for(int j = (y - CELL_RANGE_HORIZONTAL); j < (y + CELL_RANGE_HORIZONTAL); j++) {
			for(int k = (z - CELL_RANGE_VERTICAL); k < (z + CELL_RANGE_VERTICAL); k++) {
				if(m_map[i][j][k] == NULL) {
					pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZL>());
					m_map[i][j][k] = point_cloud_temp;
				}
			}
		}
	}
}

//update points to map 
void LidarMapping::updateCurrentPointsToMap(
  const pcl::PointCloud<pcl::PointXYZL>::Ptr & pc_in,
  const Eigen::Isometry3d & pose_current)
{
	int currentPosIdX = int(std::floor(pose_current.translation().x() / CELL_WIDTH + 0.5)) + m_originInMapX;
	int currentPosIdY = int(std::floor(pose_current.translation().y() / CELL_HEIGHT + 0.5)) + m_originInMapY;
	int currentPosIdZ = int(std::floor(pose_current.translation().z() / CELL_DEPTH + 0.5)) + m_originInMapZ;

	//check is submap is null
	checkPoints(currentPosIdX, currentPosIdY, currentPosIdZ);

	pcl::PointCloud<pcl::PointXYZL>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZL>());
	pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());

	//save points
	for (int i = 0; i < (int)transformed_pc->points.size(); i++) {
		pcl::PointXYZL point_temp = transformed_pc->points[i];

		//for visualization only
		int currentPointIdX = int(std::floor(point_temp.x / CELL_WIDTH + 0.5)) + m_originInMapX;
		int currentPointIdY = int(std::floor(point_temp.y / CELL_HEIGHT + 0.5)) + m_originInMapY;
		int currentPointIdZ = int(std::floor(point_temp.z / CELL_DEPTH + 0.5)) + m_originInMapZ;

		m_map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
	}

	//filtering points 
	for(int i = currentPosIdX - CELL_RANGE_HORIZONTAL; i < currentPosIdX + CELL_RANGE_HORIZONTAL; i++) {
		for(int j = currentPosIdY - CELL_RANGE_HORIZONTAL; j < currentPosIdY + CELL_RANGE_HORIZONTAL; j++) {
			for(int k = currentPosIdZ - CELL_RANGE_VERTICAL; k < currentPosIdZ + CELL_RANGE_VERTICAL; k++) {
				m_downSizeFilter.setInputCloud(m_map[i][j][k]);
				m_downSizeFilter.filter(*(m_map[i][j][k]));
			}
		}
	}
}

pcl::PointCloud<pcl::PointXYZL>::Ptr LidarMapping::getMap(void)
{
	pcl::PointCloud<pcl::PointXYZL>::Ptr LidarCloudMap(new  pcl::PointCloud<pcl::PointXYZL>());
	for (int i = 0; i < m_mapWidth; i++) {
		for (int j = 0; j < m_mapHeight; j++) {
			for (int k = 0; k < m_mapDepth; k++) {
				if(m_map[i][j][k]!=NULL){
					*LidarCloudMap += *(m_map[i][j][k]);
				}
			}
		}
	}
	return LidarCloudMap;
}

}  // namespace lidar
}  // namespace floam

