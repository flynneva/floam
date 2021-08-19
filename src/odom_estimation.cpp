
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "floam/lidar_optimization.hpp"
#include "floam/odom_estimation.hpp"

#include <iostream>

namespace floam
{
namespace odom
{

OdomEstimation::OdomEstimation()
{
  // constructor
}

void OdomEstimation::init(const double & mapResolution) {
  //init local map
  m_lidarCloudCornerMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  m_lidarCloudSurfMap = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  //downsampling size
  m_downSizeFilterEdge.setLeafSize(mapResolution, mapResolution, mapResolution);
  m_downSizeFilterSurf.setLeafSize(mapResolution * 2, mapResolution * 2, mapResolution * 2);
  //kd-tree
  m_kdTreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  m_kdTreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());

  m_odom = Eigen::Isometry3d::Identity();
  m_lastOdom = Eigen::Isometry3d::Identity();

  m_optimizationCount = 2;
}

void OdomEstimation::initMapWithPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edges,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfaces)
{
  *m_lidarCloudCornerMap += *edges;
  *m_lidarCloudSurfMap += *surfaces;
  /// TODO(flynneva): make this a parameter
  m_optimizationCount = 6;
}


void OdomEstimation::updatePointsToMap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edges,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfaces)
{
  /// TODO(flynneva): make this a parameter
  if (m_optimizationCount > 2) {
    m_optimizationCount--;
  }
  
  // init temp odom prediction object
  Eigen::Isometry3d odomPrediction = m_odom * (m_lastOdom.inverse() * m_odom);

  // update odom objects
  m_lastOdom = m_odom;
  m_odom = odomPrediction;

  m_currentTranslation = m_odom.translation();
  m_currentRotation = Eigen::Quaterniond(m_odom.rotation());

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZ>());

  /// generate map from edges and surfaces
  downSamplingToMap(edges, downsampledEdgeCloud, surfaces, downsampledSurfCloud);

  // TODO(flynneva): make these limits parameters?
  if (m_lidarCloudCornerMap->points.size() > 10 &&
      m_lidarCloudSurfMap->points.size() > 50)
  {
    m_kdTreeEdgeMap->setInputCloud(m_lidarCloudCornerMap);
    m_kdTreeSurfMap->setInputCloud(m_lidarCloudSurfMap);
    
    for (int optCount = 0; optCount < m_optimizationCount; optCount++) {
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(m_parameters, 7, new floam::lidar::PoseSE3Parameterization());
      
      addEdgeCostFactor(downsampledEdgeCloud, m_lidarCloudCornerMap, problem, loss_function);
      addSurfCostFactor(downsampledSurfCloud, m_lidarCloudSurfMap, problem, loss_function);
      ceres::Solver::Options options;
      /// TODO(flynneva): make these parameters
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
    }
  } else {
    printf("[ updatePointsToMap ] not enough points in map to associate\n");
    printf("[ updatePointsToMap ] corners: %li  [ minimum required: > 10]\n", m_lidarCloudCornerMap->points.size());
    printf("[ updatePointsToMap ] surfaces: %li [ minimum required: > 50]\n", m_lidarCloudSurfMap->points.size());
  }

  /// Update odom
  m_odom = Eigen::Isometry3d::Identity();
  m_odom.linear() = m_currentRotation.toRotationMatrix();
  m_odom.translation() = m_currentTranslation;

  addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
}

void OdomEstimation::pointAssociateToMap(
  const std::shared_ptr<pcl::PointXYZ> & pointIn,
  std::shared_ptr<pcl::PointXYZ> & pointOut)
{
  Eigen::Vector3d pointCurrent(pointIn->x, pointIn->y, pointIn->z);
  Eigen::Vector3d point_w = m_currentRotation * pointCurrent + m_currentTranslation;

  pointOut->x = point_w.x();
  pointOut->y = point_w.y();
  pointOut->z = point_w.z();
}

void OdomEstimation::downSamplingToMap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & edgesIn,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & edgesOut,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & surfacesIn,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & surfacesOut)
{
  m_downSizeFilterEdge.setInputCloud(edgesIn);
  m_downSizeFilterEdge.filter(*edgesOut);
  m_downSizeFilterSurf.setInputCloud(surfacesIn);
  m_downSizeFilterSurf.filter(*surfacesOut);    
}

void OdomEstimation::addEdgeCostFactor(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
  ceres::Problem & problem,
  ceres::LossFunction *loss_function)
{
  int corner_num=0;
  auto pointTemp = std::make_shared<pcl::PointXYZ>();
  auto pointToAdd = std::make_shared<pcl::PointXYZ>();
  for (int i = 0; i < (int)points->points.size(); i++)
  {
    pointTemp.reset(new pcl::PointXYZ());
    pointToAdd.reset(new pcl::PointXYZ(points->points[i]));
    pointAssociateToMap(pointToAdd, pointTemp);

    // nearest k neighbors search
    const int k = 5;
    std::vector<int> pointSearchInd(k);
    std::vector<float> pointSearchSqDis(k);

    if (m_kdTreeEdgeMap->nearestKSearch(*pointTemp, k, pointSearchInd, pointSearchSqDis) > 0) {
      // check if last nearest point is within 1.0m sq away
      if (pointSearchSqDis[k - 1] < 1.0)
      {
        std::vector<Eigen::Vector3d> nearCorners;
        Eigen::Vector3d center(0, 0, 0);
        
        for (int j = 0; j < k; j++)
        {
          Eigen::Vector3d tmp(
            map->points[pointSearchInd[j]].x,
            map->points[pointSearchInd[j]].y,
            map->points[pointSearchInd[j]].z);
          center = center + tmp;
          nearCorners.push_back(tmp);
        }
        // average
        center = center / (float)k;
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        
        for (int j = 0; j < k; j++)
        {
          Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d curr_point(points->points[i].x, points->points[i].y, points->points[i].z);
        
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
        { 
          Eigen::Vector3d point_on_line = center;
          Eigen::Vector3d point_a, point_b;
          point_a = 0.1 * unit_direction + point_on_line;
          point_b = -0.1 * unit_direction + point_on_line;
          ceres::CostFunction* cost_function =
            new floam::lidar::LidarEdgeFunctor(curr_point, point_a, point_b);
          problem.AddResidualBlock(cost_function, loss_function, m_parameters);
          corner_num++;
        }                           
      }
    }
  }

  if(corner_num < 20){
    printf("not enough correct corner points: %i\n", corner_num);
  }

}

void OdomEstimation::addSurfCostFactor(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & map,
  ceres::Problem & problem,
  ceres::LossFunction * lossFunction)
{
  int surf_num=0;
  const int k = 5;
  auto pointTemp = std::make_shared<pcl::PointXYZ>();
  auto pointToAdd = std::make_shared<pcl::PointXYZ>();
  for (int i = 0; i < (int)points->points.size(); i++)
  {
    pointTemp.reset(new pcl::PointXYZ());
    pointToAdd.reset(new pcl::PointXYZ(points->points[i]));
    pointAssociateToMap(pointToAdd, pointTemp);

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    // check if last nearest point is within 1.0m sq away
    if (m_kdTreeSurfMap->nearestKSearch(*pointTemp, k, pointSearchInd, pointSearchSqDis) > 0) {
      if (pointSearchSqDis[k - 1] < 1.0)
      {
        Eigen::Matrix<double, k, 3> matA0;
        Eigen::Matrix<double, k, 1> matB0 = -1 * Eigen::Matrix<double, k, 1>::Ones();

        for (int j = 0; j < k; j++)
        {
          matA0(j, 0) = map->points[pointSearchInd[j]].x;
          matA0(j, 1) = map->points[pointSearchInd[j]].y;
          matA0(j, 2) = map->points[pointSearchInd[j]].z;
        }
        // find the norm of plane
        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        double negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();
  
        bool planeValid = true;
        for (int j = 0; j < k; j++)
        {
          // if OX * n > 0.2, then plane is not fit well
          if (
            fabs(
              norm(0) * map->points[pointSearchInd[j]].x +
              norm(1) * map->points[pointSearchInd[j]].y +
              norm(2) * map->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
          {
              planeValid = false;
              break;
          }
        }
        Eigen::Vector3d curr_point(points->points[i].x, points->points[i].y, points->points[i].z);
        if (planeValid)
        {
          ceres::CostFunction *cost_function =
            new floam::lidar::LidarSurfaceFunctor(curr_point, norm, negative_OA_dot_norm);   
          problem.AddResidualBlock(cost_function, lossFunction, m_parameters);
          surf_num++;
        }
      }
    }
  }
  if (surf_num < 20) {
    printf("not enough correct surface points: %i\n", surf_num);
  }
}

void OdomEstimation::addPointsToMap(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & downsampledEdgeCloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & downsampledSurfCloud)
{
  int i = 0;
  auto pointTemp = std::make_shared<pcl::PointXYZ>();
  auto pointToAdd = std::make_shared<pcl::PointXYZ>();
  for (i = 0; i < (int)downsampledEdgeCloud->points.size(); i++) {
    pointTemp.reset(new pcl::PointXYZ());
    pointToAdd.reset(new pcl::PointXYZ(downsampledEdgeCloud->points[i]));
    pointAssociateToMap(pointToAdd, pointTemp);
    m_lidarCloudCornerMap->push_back(*pointTemp.get()); 
  }
  
  for (i = 0; i < (int)downsampledSurfCloud->points.size(); i++) {
    pointTemp.reset(new pcl::PointXYZ());
    pointToAdd.reset(new pcl::PointXYZ(downsampledSurfCloud->points[i]));
    pointAssociateToMap(pointToAdd, pointTemp);
    m_lidarCloudSurfMap->push_back(*pointTemp.get());
  }
  
  double x_min = +m_currentTranslation.x() - 100;
  double y_min = +m_currentTranslation.y() - 100;
  double z_min = +m_currentTranslation.z() - 100;
  double x_max = +m_currentTranslation.x() + 100;
  double y_max = +m_currentTranslation.y() + 100;
  double z_max = +m_currentTranslation.z() + 100;

  //ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
  m_cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  m_cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  m_cropBoxFilter.setNegative(false);    

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZ>());

  m_cropBoxFilter.setInputCloud(m_lidarCloudSurfMap);
  m_cropBoxFilter.filter(*tmpSurf);

  m_cropBoxFilter.setInputCloud(m_lidarCloudCornerMap);
  m_cropBoxFilter.filter(*tmpCorner);

  m_downSizeFilterSurf.setInputCloud(tmpSurf);
  m_downSizeFilterSurf.filter(*m_lidarCloudSurfMap);

  m_downSizeFilterEdge.setInputCloud(tmpCorner);
  m_downSizeFilterEdge.filter(*m_lidarCloudCornerMap);
}

void OdomEstimation::getMap(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & lidarCloudMap)
{
  *lidarCloudMap += *m_lidarCloudSurfMap;
  *lidarCloudMap += *m_lidarCloudCornerMap;
}

}  // namespace odom
}  // namespace floam
