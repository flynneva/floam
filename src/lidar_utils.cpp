
/// Major rewrite Author: Evan Flynn

#include "floam/lidar_utils.hpp"

namespace floam
{
namespace lidar
{

Scanner::Scanner()
{
  // constructor
}

Scanner::~Scanner()
{
  // destructor
}

Imager::Imager()
{
  // constructor
}

Imager::~Imager()
{
  // destructor
}

Double2d::Double2d(int id_in, double value_in) {
  id = id_in;
  value = value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in) {
  layer = layer_in;
  time = time_in;
};

}  // namespace lidar
}  // namespace floam