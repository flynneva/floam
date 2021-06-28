#ifndef FLOAM__LIDAR_UTILS_HPP_
#define FLOAM__LIDAR_UTILS_HPP_

namespace floam
{
namespace lidar
{

enum Type {
  rotatingScanner = 0,
  imager  // TODO(flynneva): add more types here?
};

struct Distance {
  double max{100.0};
  double min{0.0};
};

struct Limits {
  Distance distance;
};

struct AngularResolution {
  uint16_t vertical{1};
  uint16_t horizontal{1};
};

struct FOV {
  double vertical{30.0};
  double horizontal{120.0};
};

}  // namespace lidar
}  // namespace floam

#endif  // FLOAM__LIDAR_UTILS_HPP_