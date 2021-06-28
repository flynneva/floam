
// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef FLOAM__LIDAR_SCANNER_HPP_
#define FLOAM__LIDAR_SCANNER_HPP_
#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

struct Scanner {
  double period{100.0};  // milliseconds
  uint16_t lines{16};
  FOV fov;  // degress
  AngularResolution angular;  // degrees
  Limits limits;
};

class ScanningLidar
{
  public:
    ScanningLidar();
    ~ScanningLidar();

    // Set in the constructor
    int m_pointsPerLine;

    Scanner m_settings;
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_SCANNER_HPP_

