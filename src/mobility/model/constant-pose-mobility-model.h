#ifndef CONSTANT_POSE_MOBILITY_MODEL_H
#define CONSTANT_POSE_MOBILITY_MODEL_H

#include "ns3/quaternion.h"
#include "pose-mobility-model.h"

namespace ns3 {

/**
 * \ingroup mobility
 *
 * \brief Mobility model for which the current pose does not change once it has been set and until it is set again explicitly to a new value.
 */
class ConstantPoseMobilityModel : public PoseMobilityModel
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  /**
   * Create a pose located at coordinates (0,0,0)
   */
  ConstantPoseMobilityModel ();
  virtual ~ConstantPoseMobilityModel ();

private:
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetOrientation (void) const;
  virtual void DoSetOrientation (const Vector &orientation);
  virtual Vector DoGetVelocity (void) const;

  Vector m_position; //!< the constant position
  Quaternion m_orientation;

};

} // namespace ns3

#endif /* CONSTANT_POSE_MOBILITY_MODEL_H */
