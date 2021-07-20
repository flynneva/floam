
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_OPTIMIZATION_HPP_
#define FLOAM__LIDAR_OPTIMIZATION_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace floam
{

namespace lidar
{

void getTransformFromSe3(const Eigen::Matrix<double, 6, 1> &se3, Eigen::Quaterniond &q, Eigen::Vector3d &t);
Eigen::Matrix3d skew(Eigen::Vector3d &mat_in);


class LidarEdgeFunctor : public ceres::SizedCostFunction<1, 7>
{
public:
  LidarEdgeFunctor(
    Eigen::Vector3d currentPoint, Eigen::Vector3d lastPointA, Eigen::Vector3d lastPointB)
  : m_currentPoint(currentPoint), m_lastPointA(lastPointA), m_lastPointB(lastPointB)
  {}

  virtual bool Evaluate(double const *const *parameters, double* residuals, double** jacobians) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_last_curr(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[0] + 4);
    Eigen::Vector3d lp;
    lp = q_last_curr * m_currentPoint + t_last_curr;
    Eigen::Vector3d nu = (lp - m_lastPointA).cross(lp - m_lastPointB);
    Eigen::Vector3d de = m_lastPointA - m_lastPointB;
    double de_norm = de.norm();
    residuals[0] = nu.norm() / de_norm;
    if (jacobians != NULL)
    {
      if (jacobians[0] != NULL)
      {
        Eigen::Matrix3d skew_lp = skew(lp);
        Eigen::Matrix<double, 3, 6> dp_by_se3;
        dp_by_se3.block<3, 3>(0, 0) = -skew_lp;
        (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
        Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
        J_se3.setZero();
        Eigen::Matrix3d skew_de = skew(de);
        J_se3.block<1, 6>(0, 0) = -nu.transpose() / nu.norm() * skew_de * dp_by_se3 / de_norm;
      }
    }
    return true;
  }

  Eigen::Vector3d m_currentPoint, m_lastPointA, m_lastPointB;
};


class LidarSurfaceFunctor : public ceres::SizedCostFunction<1, 7>
{
public:
  LidarSurfaceFunctor(Eigen::Vector3d currentPoint, Eigen::Vector3d planeUnitNormal, double negativeOADotNormal)
  : m_currentPoint(currentPoint),
    m_planeUnitNormal(planeUnitNormal),
    m_negativeOADotNormal(negativeOADotNormal) {}

  virtual bool Evaluate(double const *const *parameters, double* residuals, double** jacobians) const
  {
      Eigen::Map<const Eigen::Quaterniond> q_w_curr(parameters[0]);
      Eigen::Map<const Eigen::Vector3d> t_w_curr(parameters[0] + 4);
      Eigen::Vector3d point_w = q_w_curr * m_currentPoint + t_w_curr;
      residuals[0] = m_planeUnitNormal.dot(point_w) + m_negativeOADotNormal;
      if (jacobians != NULL)
      {
          if (jacobians[0] != NULL)
          {
              Eigen::Matrix3d skew_point_w = skew(point_w);
              Eigen::Matrix<double, 3, 6> dp_by_se3;
              dp_by_se3.block<3, 3>(0, 0) = -skew_point_w;
              (dp_by_se3.block<3, 3>(0, 3)).setIdentity();
              Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
              J_se3.setZero();
              J_se3.block<1, 6>(0, 0) = m_planeUnitNormal.transpose() * dp_by_se3;
          }
      }
      return true;
  }

  Eigen::Vector3d m_currentPoint, m_planeUnitNormal;
  double m_negativeOADotNormal;
};


class PoseSE3Parameterization : public ceres::LocalParameterization
{
public:
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};
} // namespace lidar
} // namespace floam

#endif // FLOAM__LIDAR_OPTIMIZATION_HPP_
