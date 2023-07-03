//
// Created by Jakob Struye on 05/12/22.
//

#ifndef POSE_MOBILITY_MODEL_H
#define POSE_MOBILITY_MODEL_H

#include "ns3/vector.h"
#include "ns3/object.h"
#include "ns3/traced-callback.h"

#include "mobility-model.h"


namespace ns3 {
    class PoseMobilityModel : public MobilityModel {
    public:
        static TypeId GetTypeId (void);
        PoseMobilityModel ();
        virtual ~PoseMobilityModel () = 0;

        /**
         * \return the current orientation
         */
        Vector GetOrientation (void) const;
        /**
         * \param orientation the orientation to set.
         */
        void SetOrientation (const Vector &orientation);
    private:
        /**
         * \return the current orientation.
         *
         * Concrete subclasses of this base class must
         * implement this method.
         */
        virtual Vector DoGetOrientation (void) const = 0;
        /**
         * \param orientation the orientation to set.
         *
         * Concrete subclasses of this base class must
         * implement this method.
         */
        virtual void DoSetOrientation (const Vector &orientation) = 0;
    };

}
#endif//POSE_MOBILITY_MODEL_H
