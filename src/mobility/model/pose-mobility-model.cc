#include "pose-mobility-model.h"
#include "ns3/trace-source-accessor.h"

namespace ns3 {

    NS_OBJECT_ENSURE_REGISTERED(PoseMobilityModel);

    TypeId
    PoseMobilityModel::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::PoseMobilityModel")
                                .SetParent<MobilityModel>()
                                .SetGroupName("Mobility")
                                .AddAttribute("Orientation", "The current orientation of the mobility model.",
                                              TypeId::ATTR_SET | TypeId::ATTR_GET,
                                              VectorValue(Vector(0.0, 0.0, 0.0)),
                                              MakeVectorAccessor(&PoseMobilityModel::SetOrientation,
                                                                 &PoseMobilityModel::GetOrientation),
                                              MakeVectorChecker());
        return tid;
    }

    PoseMobilityModel::PoseMobilityModel() {
    }

    PoseMobilityModel::~PoseMobilityModel() {
    }

    Vector
    PoseMobilityModel::GetOrientation(void) const {
        return DoGetOrientation();
    }

    void
    PoseMobilityModel::SetOrientation(const Vector &orientation) {
        DoSetOrientation(orientation);
    }
}