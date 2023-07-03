#include "quaternion.h"
#include "fatal-error.h"
#include "log.h"
#include <cmath>
#include <sstream>
#include <tuple>

/**
 * \file
 * \ingroup attribute_Quaternion
 * ns3::Quaternion attribute value implementations.
 */

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("Quaternion");

//ATTRIBUTE_HELPER_CPP (Quaternion);


Quaternion::Quaternion (double _w, double _x, double _y, double _z)
  : w (_w),
    x (_x),
    y (_y),
    z (_z)
{
    Normalize();
  NS_LOG_FUNCTION (this << _w << _x << _y << _z);
}

Quaternion::Quaternion (const Vector& euler) {
  NS_LOG_FUNCTION (this << euler);

  // FROM https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  double cr = std::cos(euler.z * 0.5);
  double sr = std::sin(euler.z * 0.5);
  double cp = std::cos(euler.y * 0.5);
  double sp = std::sin(euler.y * 0.5);
  double cy = std::cos(euler.x * 0.5);
  double sy = std::sin(euler.x * 0.5);

  w = cr * cp * cy + sr * sp * sy;
  x = sr * cp * cy - cr * sp * sy;
  y = cr * sp * cy + sr * cp * sy;
  z = cr * cp * sy - sr * sp * cy;

  Normalize(); //Should already be satisfied

}

Quaternion::Quaternion ()
  : w (1.0),
    x (0.0),
    y (0.0),
    z (0.0)
{
  NS_LOG_FUNCTION (this);
}

std::ostream &operator << (std::ostream &os, const Quaternion &quat)
{
  os << quat.w << ":" << quat.x << ":" << quat.y << ":" << quat.z;
  return os;
}
std::istream &operator >> (std::istream &is, Quaternion &quat)
{
  char c1, c2, c3;
  is >> quat.w >> c1 >> quat.x >> c2 >> quat.y >> c3 >> quat.z;
  if (c1 != ':'
      || c2 != ':'
      || c3 != ':')
    {
      is.setstate (std::ios_base::failbit);
    }
  return is;
}

Vector Quaternion::ToEuler() const {

  //FROM https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  Vector euler;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (w * x + y * z);
  double cosr_cosp = 1 - 2 * (x * x + y * y);
  euler.z = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
      euler.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      euler.y = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  euler.x = std::atan2(siny_cosp, cosy_cosp);

  return euler;
}

void Quaternion::Normalize() {
  double len = std::sqrt (w * w + x * x + y * y + z * z);
  w /= len;
  x /= len;
  y /= len;
  z /= len;
}

} // namespace ns3
