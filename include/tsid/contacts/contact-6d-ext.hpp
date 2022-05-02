
#ifndef __contact_6d_ext_hpp__
#define __contact_6d_ext_hpp__

#include "tsid/contacts/contact-6d.hpp"

namespace tsid {
    namespace contacts {
        class Contact6dExt : public Contact6d {
        public:
            Contact6dExt(const std::string& name, RobotWrapper& robot, const std::string& frameName,
                ConstRefMatrix contactPoints, ConstRefVector contactNormal, const double frictionCoefficient, const double minNormalForce,
                const double maxNormalForce) : Contact6d(name, robot, frameName, contactPoints, contactNormal,
                                                   frictionCoefficient, minNormalForce, maxNormalForce){};

            ~Contact6dExt(){};

            void setReference(trajectories::TrajectorySample sample_ref)
            {
                m_motionTask.setReference(sample_ref);
            }

            void setMask(math::ConstRefVector mask)
            {
                m_motionTask.setMask(mask);
            }

            Matrix3x getContactPoints() { return m_contactPoints; }

            Vector6 getForceReference() { return m_fRef; }
        };
    } // namespace contacts
} // namespace tsid
#endif