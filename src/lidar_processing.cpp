// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "floam/lidar_processing.hpp"

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/organized_edge_detection.h>

namespace floam
{
namespace lidar
{


LidarProcessing::LidarProcessing()
{
  // constructor
}

void LidarProcessing::init(lidar::Lidar lidar_in)
{
  m_lidar = lidar_in;
}

pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>
LidarProcessing::createSurfaceNormalsBase(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normals_est;

  // only for structured pointclouds
  normals_est.setNormalEstimationMethod(normals_est.AVERAGE_3D_GRADIENT);
  // TODO(flynneva): make these adjustable
  normals_est.setMaxDepthChangeFactor(0.02f);
  normals_est.setNormalSmoothingSize(10.0f);
  normals_est.setInputCloud(points);
  return normals_est;
}

pcl::OrganizedEdgeBase<pcl::PointXYZ, pcl::Label> 
LidarProcessing::createEdgeBase(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pc_in)
{
  // pointcloud in has to be organized
  pcl::OrganizedEdgeBase <pcl::PointXYZ, pcl::Label> edgeDetector;

  edgeDetector.setInputCloud(pc_in);

  // TODO(flynneva): make these adjustable
  edgeDetector.setDepthDisconThreshold(0.02f);
  edgeDetector.setMaxSearchNeighbors(50);

  return edgeDetector;
}

void LidarProcessing::featureExtraction(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf)
{
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    // only applicable for scanning lidars like velodynes
    if (m_lidar.is_scanner) {
      int rows = m_lidar.num_lines;
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_points;
      for(int i=0; i<rows; i++) {
        m_points.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
      }

      for (int i = 0; i < (int) pc_in->points.size(); i++) {
          int scanID=0;
          double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
          if(distance<m_lidar.min_distance || distance>m_lidar.max_distance)
              continue;
          double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;
  
          if (rows == 16) {
              scanID = int((angle + 15) / 2 + 0.5);
              if (scanID > (rows - 1) || scanID < 0) {
                  continue;
              }
          }
          else if (rows == 32) {
              scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
              if (scanID > (rows - 1) || scanID < 0)
              {
                  continue;
              }
          }
          else if (rows == 64)
          {   
              if (angle >= -8.83)
                  scanID = int((2 - angle) * 3.0 + 0.5);
              else
                  scanID = rows / 2 + int((-8.83 - angle) * 2.0 + 0.5);
  
              if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
              {
                  continue;
              }
          }
          else
          {
              printf("wrong scan number\n");
          }
          m_points[scanID]->push_back(pc_in->points[i]);
      }

      for(int i = 0; i < rows; i++){
          if(m_points[i]->points.size()<131){
              continue;
          }

          std::vector<Double2d> cloudCurvature; 
          int total_points = m_points[i]->points.size()-10;
          for(int j = 5; j < (int)m_points[i]->points.size() - 5; j++){
              double diffX = m_points[i]->points[j - 5].x + m_points[i]->points[j - 4].x + m_points[i]->points[j - 3].x + m_points[i]->points[j - 2].x + m_points[i]->points[j - 1].x - 10 * m_points[i]->points[j].x + m_points[i]->points[j + 1].x + m_points[i]->points[j + 2].x + m_points[i]->points[j + 3].x + m_points[i]->points[j + 4].x + m_points[i]->points[j + 5].x;
              double diffY = m_points[i]->points[j - 5].y + m_points[i]->points[j - 4].y + m_points[i]->points[j - 3].y + m_points[i]->points[j - 2].y + m_points[i]->points[j - 1].y - 10 * m_points[i]->points[j].y + m_points[i]->points[j + 1].y + m_points[i]->points[j + 2].y + m_points[i]->points[j + 3].y + m_points[i]->points[j + 4].y + m_points[i]->points[j + 5].y;
              double diffZ = m_points[i]->points[j - 5].z + m_points[i]->points[j - 4].z + m_points[i]->points[j - 3].z + m_points[i]->points[j - 2].z + m_points[i]->points[j - 1].z - 10 * m_points[i]->points[j].z + m_points[i]->points[j + 1].z + m_points[i]->points[j + 2].z + m_points[i]->points[j + 3].z + m_points[i]->points[j + 4].z + m_points[i]->points[j + 5].z;
              Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
              cloudCurvature.push_back(distance);
  
          }
          for(int j=0;j<6;j++){
              int sector_length = (int)(total_points/6);
              int sector_start = sector_length *j;
              int sector_end = sector_length *(j+1)-1;
              if (j==5){
                  sector_end = total_points - 1; 
              }
              std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end); 
              
              featureExtractionFromSector(m_points[i],subCloudCurvature, pc_out_edge, pc_out_surf);
              
          }
      }
    } else {
      // not a scanner, so treat entire pointcloud as one "scan"
      std::vector<Double2d> cloudCurvature; 
      int total_points = pc_in->points.size();
      // TODO(flynneva): make this a parameter?
      uint16_t grid_size = 3; // sets grid size - so 4 would be a 4x4, 6 would be 6x6, etc.
      if (grid_size % 2 != 0) {
        std::cout << "[ Lidar::featureExtraction ] ERROR: grid_size has to be even" << std::endl;
        std::cout << "[ Lidar::featureExtraction ] WARN: setting grid_size to 4..." << std::endl;
        grid_size = 4;
      }

      if (pc_in->height % grid_size != 0) {
        std::cout << "[ Lidar::featureExtraction ] ERROR: pointcloud height not divisible by grid_size" << std::endl;
        std::cout << "[ Lidar::featureExtraction ] WARN: make sure pointcloud height " << pc_in->height << " can be divisible by grid_size" << std::endl;
      }

      if (pc_in->width % grid_size != 0) {
        std::cout << "[ Lidar::featureExtraction ] ERROR: pointcloud width not divisible by grid_size" << std::endl;
        std::cout << "[ Lidar::featureExtraction ] WARN: make sure pointcloud width " << pc_in->width << " can be divisible by grid_size" << std::endl;
      }

      int half_grid = int(grid_size / 2);
      double diffX = 0;
      double diffY = 0;
      double diffZ = 0;
      int grid_count = 0;
      for(int i = half_grid; i < (int)pc_in->points.size() - half_grid; i+=grid_size){
        // group returns together in groups of 9 (3 x 3)
        for (int j = -half_grid; j <= half_grid; j++) {
          diffX += pc_in->points[j + i].x;
          diffY += pc_in->points[j + i].y;
          diffZ += pc_in->points[j + i].z;
        }

        Double2d distance(i, diffX * diffX + diffY * diffY + diffZ * diffZ);
        cloudCurvature.push_back(distance);
        grid_count++;
      }

      std::cout << "[ Lidar::featureExtraction ] INFO: divided pointcloud into " << grid_count << " sections" << std::endl;
      for(int i = 0; i < grid_count; i++) {
        // int sector_length = (int)(total_points / grid_count);  
        int sector_start = grid_size * i;
        int sector_end = grid_size * (i + 1) - 1;
        if (i == grid_count - 1) {
            sector_end = total_points - 1; 
        }
        std::vector<Double2d> subCloudCurvature(cloudCurvature.begin() + sector_start, cloudCurvature.begin() + sector_end); 

        featureExtractionFromSector(pc_in, subCloudCurvature, pc_out_edge, pc_out_surf);
      }
    }
}

void LidarProcessing::featureExtractionFromSector(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
  std::vector<Double2d>& cloudCurvature,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf)
{
  std::cout << "FEATURE EXTRACTION FROM SECTOR" << std::endl;
  std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
  { 
    return a.value < b.value; 
  });
  std::cout << "cloudCurvature size: "<< cloudCurvature.size() << std::endl;
  int largestPickedNum = 0;
  std::vector<int> picked_points;
  int point_info_count =0;
  for (int i = cloudCurvature.size()-1; i >= 0; i--) {
    int ind = cloudCurvature[i].id; 
    if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()) {
      if(cloudCurvature[i].value <= 0.1) {
        break;
      }

      largestPickedNum++;
      picked_points.push_back(ind);
      std::cout << "FEATURE EXTRACTION FROM SECTOR 2" << std::endl;
      if (largestPickedNum <= 20) {
        pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      } else {
        break;
      }
      for(int k=1;k<=5;k++) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
          break;
        }
        picked_points.push_back(ind+k);
      }
      for(int k=-1;k>=-5;k--) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
          break;
        }
        picked_points.push_back(ind+k);
      }
    }
  }

    //find flat points
    // point_info_count =0;
    // int smallestPickedNum = 0;
    
    // for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    // {
    //     int ind = cloudCurvature[i].id; 

    //     if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
    //         if(cloudCurvature[i].value > 0.1){
    //             //ROS_WARN("extracted feature not qualified, please check lidar");
    //             break;
    //         }
    //         smallestPickedNum++;
    //         picked_points.push_back(ind);
            
    //         if(smallestPickedNum <= 4){
    //             //find all points
    //             pc_surf_flat->push_back(pc_in->points[ind]);
    //             pc_surf_lessFlat->push_back(pc_in->points[ind]);
    //             point_info_count++;
    //         }
    //         else{
    //             break;
    //         }

    //         for(int k=1;k<=5;k++){
    //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
    //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
    //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
    //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
    //                 break;
    //             }
    //             picked_points.push_back(ind+k);
    //         }
    //         for(int k=-1;k>=-5;k--){
    //             double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
    //             double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
    //             double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
    //             if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
    //                 break;
    //             }
    //             picked_points.push_back(ind+k);
    //         }

    //     }
    // }
  for (int i = 0; i <= (int)cloudCurvature.size()-1; i++) {
    int ind = cloudCurvature[i].id; 
    if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()) {
      pc_out_surf->push_back(pc_in->points[ind]);
    }
  }
}

Double2d::Double2d(int id_in, double value_in) {
  id = id_in;
  value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in) {
  layer = layer_in;
  time = time_in;
};

}  // namespace lidar
}  // namespace floam
