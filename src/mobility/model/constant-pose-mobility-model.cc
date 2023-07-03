
#include "constant-pose-mobility-model.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (ConstantPoseMobilityModel);

TypeId
ConstantPoseMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ConstantPoseMobilityModel")
    .SetParent<PoseMobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<ConstantPoseMobilityModel> ()
  ;
  return tid;
}

ConstantPoseMobilityModel::ConstantPoseMobilityModel ()
{
}
ConstantPoseMobilityModel::~ConstantPoseMobilityModel ()
{
}

Vector
ConstantPoseMobilityModel::DoGetPosition (void) const
{
  return m_position;
}
void
ConstantPoseMobilityModel::DoSetPosition (const Vector &position)
{
  m_position = position;
  NotifyCourseChange ();
}
Vector
ConstantPoseMobilityModel::DoGetOrientation (void) const
{
  return m_orientation.ToEuler();
}
void
ConstantPoseMobilityModel::DoSetOrientation (const Vector &orientation)
{
  m_orientation = Quaternion(orientation);
  NotifyCourseChange ();
}
Vector
ConstantPoseMobilityModel::DoGetVelocity (void) const
{
  return Vector (0.0, 0.0, 0.0);
}

} // namespace ns3
