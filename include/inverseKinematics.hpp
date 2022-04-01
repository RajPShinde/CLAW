#ifndef INCLUDE_INVERSEKINEMATICS_HPP_
#define INCLUDE_INVERSEKINEMATICS_HPP_

#include <claw.hpp>

namespace kinematics{
class InverseKinematics {
    public:
        InverseKinematics();

        ~InverseKinematics();

        computeJointAngles(double x, double y);
};
}

#endif  //  INCLUDE_INVERSEKINEMATICS_HPP_