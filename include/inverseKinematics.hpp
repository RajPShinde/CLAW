#ifndef INCLUDE_INVERSEKINEMATICS_HPP_
#define INCLUDE_INVERSEKINEMATICS_HPP_

#include <claw.hpp>

class InverseKinematics {
    public:
        InverseKinematics();

        ~InverseKinematics();

        std::vector<double> computeJointAngles(double x, double y);
};

#endif  //  INCLUDE_INVERSEKINEMATICS_HPP_