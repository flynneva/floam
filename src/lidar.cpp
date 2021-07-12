
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>

#include "floam/lidar_utils.hpp"
#include "floam/lidar.hpp"

#include <iostream>

namespace floam
{
namespace lidar
{

/// Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointXYZL>::Ptr & edges)
{
  int N_SCANS = m_settings.lines;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> lidarScans;

  for(int i = 0; i < N_SCANS; i++){
      lidarScans.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()));
  }

  // separate out pointcloud into different scan lines
  // essentially trying to make it an "ordered pointcloud"
  for (int i = 0; i < (int) points->points.size(); i++)
  {
    int scanID=0;

    double distance =
      sqrt(
        points->points[i].x * points->points[i].x +
        points->points[i].y * points->points[i].y);

    if (distance < m_settings.common.limits.distance.min ||
        distance > m_settings.common.limits.distance.max)
    {
      // distance out of min/max limits, go to next point
      continue;
    }
    double angle = atan(points->points[i].z / distance) * 180 / M_PI;
    
    if (N_SCANS == 16) {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        continue;
      }
    } else if (N_SCANS == 32) {
        scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
        if (scanID > (N_SCANS - 1) || scanID < 0) {
          continue;
        }
    } else if (N_SCANS == 64) {   
      if (angle >= -8.83) {
        scanID = int((2 - angle) * 3.0 + 0.5);
      } else {
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
      }

      if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0) {
        continue;
      }
    } else {
      printf("wrong scan number\n");
    }
    lidarScans[scanID]->push_back(points->points[i]); 
  }

  for(int i = 0; i < N_SCANS; i++) {
    /// why 131?
    if(lidarScans[i]->points.size() < 131) {
      continue;
    }

    /// TODO(flynneva): separate out this bit to function?
    /// 
    /// input: points, sectorSize
    /// output: cloudCurvature
    std::vector<floam::lidar::Double2d> cloudCurvature;
    // size of grid of points
    // TODO(flynneva): make sectorSize adjustable
    int sectorSize = 10;
    int halfSector = sectorSize / 2;
    // (TODO: flynneva): do we really need to subtract the sectorSize from the total points?
    int total_points = lidarScans[i]->points.size() - sectorSize;
    // calculate number of sectors
    m_settings.common.limits.sectors = (int)(total_points / sectorSize);
  
    double diffX, diffY, diffZ = 0;
    for(int j = halfSector; j < (int)(lidarScans[i]->points.size() - halfSector); j += 1) {
      // reset diff's at new point
      diffX = 0;
      diffY = 0;
      diffZ = 0;
      for (int k = -halfSector; k <= halfSector; k++) {
        // the middle point )of each sector is the "baseline"
        if (k == 0) {
          // subtract middle point * size (because we add the rest of the points)
          diffX -= sectorSize * lidarScans[i]->points[j + k].x;
          diffY -= sectorSize * lidarScans[i]->points[j + k].y;
          diffZ -= sectorSize * lidarScans[i]->points[j + k].z;
        } else {
          // add points left and right of middle of sector
          diffX += lidarScans[i]->points[j + k].x;
          diffY += lidarScans[i]->points[j + k].y;
          diffZ += lidarScans[i]->points[j + k].z;
        }
      }
      // if diff total is large, sector is very curved or could be an edge
      // j - 1 to store actual location in points index
      floam::lidar::Double2d distance(j - 1, diffX * diffX + diffY * diffY + diffZ * diffZ);
      cloudCurvature.push_back(distance);
    }
    /// end cloudCurvature func


    int edgesCount = 1;
    int index = 0;
    /// loop over sectors
    for(int j = 0; j < m_settings.common.limits.sectors; j++) {
      int sector_start = sectorSize * j;
      int sector_end = sectorSize * (j + 1);
      if (j == (m_settings.common.limits.sectors - 1)) {
        sector_end = total_points - 1; 
      }

      std::vector<floam::lidar::Double2d> subCloudCurvature(
        cloudCurvature.begin() + sector_start,
        cloudCurvature.begin() + sector_end); 

      // sort diff's within sector
      std::sort(subCloudCurvature.begin(), subCloudCurvature.end(),
        [](const floam::lidar::Double2d & a, const floam::lidar::Double2d & b)
        { 
          return a.value < b.value;
        });

      for (int k = 0; k < (int)subCloudCurvature.size(); k++) {
        // get index of point
        index = subCloudCurvature[k].id;
        pcl::PointXYZL tempPointL;
        tempPointL.x = lidarScans[i]->points[index].x;
        tempPointL.y = lidarScans[i]->points[index].y;
        tempPointL.z = lidarScans[i]->points[index].z;
        if (subCloudCurvature[k].value <= m_settings.common.limits.edgeThreshold) {
          // value is small, is not an edge and assume its a surface
          tempPointL.label = 0;
        } else {
          // value is large so this sector is very curved or could be an edge
          tempPointL.label = 1;  // edgesCount;
        }
        edges->push_back(tempPointL);
      }
      // increment edge count on different sector
      edgesCount++;
    }
  }

}

/// Imager type
template <>
void Lidar<floam::lidar::Imager>::detectEdges(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointXYZL>::Ptr & edges)
{
  // pointcloud in has to be organized (i.e. height and width)
  pcl::OrganizedEdgeBase <pcl::PointXYZ, pcl::Label> edgeDetector;
  pcl::PointCloud<pcl::Label>::Ptr labels;
  std::vector<pcl::PointIndices> indicies;
  edgeDetector.setInputCloud(points);
  // TODO(flynneva): make these adjustable
  edgeDetector.setDepthDisconThreshold(0.02f);
  edgeDetector.setMaxSearchNeighbors(50);

  // calculate edges
  edgeDetector.compute(*labels, indicies);

  // combine xyz cloud with labels
  pcl::concatenateFields(*points, *labels, *edges);
}

/// Scanner type
template <>
void Lidar<floam::lidar::Scanner>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointNormal>::Ptr & normals)
{
  // Create the normal estimation class, and pass the input pointcloud to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalDetector;
  pcl::PointCloud<pcl::Normal> normalCloud;
  normalDetector.setInputCloud(points);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  normalDetector.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 3cm
  normalDetector.setRadiusSearch (0.03);

  // Compute the features
  normalDetector.compute(normalCloud);
  // combine xyz cloud with normals
  pcl::concatenateFields(*points, normalCloud, *normals);
}

/// overloaded for Imager type
template <>
void Lidar<floam::lidar::Imager>::detectSurfaces(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  pcl::PointCloud<pcl::PointNormal>::Ptr & normals)
{
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalDetector;
  pcl::PointCloud<pcl::Normal> normalCloud;

  // only for structured pointclouds
  normalDetector.setNormalEstimationMethod(normalDetector.AVERAGE_3D_GRADIENT);
  // TODO(flynneva): make these adjustable
  normalDetector.setMaxDepthChangeFactor(0.02f);
  normalDetector.setNormalSmoothingSize(7.0f);
  normalDetector.setInputCloud(points);

  normalDetector.compute(normalCloud);

  // combine xyz cloud with normals
  pcl::concatenateFields(*points, normalCloud, *normals);
}

}  // namespace lidar
}  // namespace floam
