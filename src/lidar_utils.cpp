
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

Double2d::Double2d(
  const int & id,
  const double & diffTotal,
  const double & diffLeft,
  const double & diffRight)
{
  this->id = id;
  this->diffTotal = diffTotal;
  this->diffLeft = diffLeft;
  this->diffRight = diffRight;
};

PointsInfo::PointsInfo(int layer_in, double time_in) {
  this->layer = layer_in;
  this->time = time_in;
};

}  // namespace lidar
}  // namespace floam