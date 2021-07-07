
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


void getTransformFromSe3(const Eigen::Matrix<double,6,1>& se3, Eigen::Quaterniond& q, Eigen::Vector3d& t);

Eigen::Matrix3d skew(Eigen::Vector3d& mat_in);

class EdgeAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:

		EdgeAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_a_, Eigen::Vector3d last_point_b_);
		virtual ~EdgeAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d last_point_a;
		Eigen::Vector3d last_point_b;
};

class SurfNormAnalyticCostFunction : public ceres::SizedCostFunction<1, 7> {
	public:
		SurfNormAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d plane_unit_norm_, double negative_OA_dot_norm_);
		virtual ~SurfNormAnalyticCostFunction() {}
		virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

		Eigen::Vector3d curr_point;
		Eigen::Vector3d plane_unit_norm;
		double negative_OA_dot_norm;
};

class PoseSE3Parameterization : public ceres::LocalParameterization {
public:
	
    PoseSE3Parameterization() {}
    virtual ~PoseSE3Parameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};
}  // namespace lidar
}  // namespace floam

#endif // FLOAM__LIDAR_OPTIMIZATION_HPP_

