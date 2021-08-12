
/// Major rewrite Author: Evan Flynn

// Original Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef FLOAM__LIDAR_UTILS_HPP_
#define FLOAM__LIDAR_UTILS_HPP_
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/point_cloud.h>

namespace floam
{
namespace lidar
{

enum Type {
  SCANNER_ROTATING = 0,
  SCANNER_MEMS,
  IMAGER,
};

struct Distance {
  double max{100.0};
  double min{0.0};
};

struct Limits {
  Distance distance;
  int sectors;
  double edgeThreshold;
};

struct AngularResolution {
  uint16_t vertical{1};
  uint16_t horizontal{1};
};

struct FOV {
  double vertical{30.0};
  double horizontal{120.0};
};

struct Settings {
  std::string frameId;
  Type type;
  FOV fov;  // degrees
  AngularResolution angular; // degrees
  Limits limits;
};

class Scanner {
public:
  Scanner();
  ~Scanner();
  int lines{0};
  int skipPoints{50};
  int searchK{10};
  double searchRadius{0.25};
  double period{0.0};
  double scan_rate{0.0};
  Settings common;
};

class Imager {
public:
  Imager();
  ~Imager();
  double framerate{0.0};
  Settings common;
};

struct Total {
  double time{0.0};
  int frames{0};
};

struct Surface {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  pcl::PointCloud<pcl::Normal>::Ptr normals;
};

struct Edge {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  pcl::PointCloud<pcl::Label>::Ptr labels;
};

//points covariance class
class Double2d{
public:
	int id;
	double diffTotal;
  double diffLeft;
  double diffRight;
	Double2d(
    const int & id,
    const double & diffTotal,
    const double & diffLeft,
    const double & diffRight);
};

//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_UTILS_HPP_
