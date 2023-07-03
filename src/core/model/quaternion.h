#ifndef NS3_QUATERNION_H
#define NS3_QUATERNION_H

#include "attribute.h"
#include "attribute-helper.h"
#include "vector.h"

/**
 * \file
 * \ingroup geometry
 * ns3::Quaternion declaration.
 */

namespace ns3 {

/**
 * \ingroup core
 * \defgroup geometry Geometry primitives
 * \brief Quaternions for geometry,
 */

/**
 * \ingroup geometry
 * \brief a 3d Quaternion
 * \see attribute_Quaternion
 */
class Quaternion {
public:
    /**
   * \param [in] _w W element of quaternion
   * \param [in] _x X element of quaternion
   * \param [in] _y Y element of quaternion
   * \param [in] _z Z element of quaternion
   *
   * Create quaternion (_w, _x, _y, _z)
   */
    Quaternion(double _w, double _x, double _y, double _z);
    /**
     * \param [in] euler Euler Angle representation
     *
     * Create quaternion from Euler Angle (yaw, pitch, roll)
     */
    Quaternion(const Vector& euler);

    /** Create Quaternion (1.0, 0.0, 0.0, 0.0) */
    Quaternion();

    double w;//!< w element of quaternion
    double x;//!< x element of quaternion
    double y;//!< y element of quaternion
    double z;//!< z element of quaternion

    /**
   * Output streamer.
   * Quaternions are written as "w:x:y:z".
   *
   * \param [in,out] os The stream.
   * \param [in] quat The quaternion to stream
   * \return The stream.
   */
    friend std::ostream &operator<<(std::ostream &os, const Quaternion &quat);

    /**
   * Input streamer.
   *
   * Quaternions are expected to be in the form "w:x:y:z".
   *
   * \param [in,out] is The stream.
   * \param [in] quat The Quaternion.
   * \returns The stream.
   */
    friend std::istream &operator>>(std::istream &is, Quaternion &quat);

    Vector ToEuler() const;
private:
    void Normalize();
};

} // namespace ns3

#endif /* NS3_QUATERNION_H */
