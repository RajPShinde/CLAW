#ifndef INCLUDE_VELOCITYKINEMATICS_HPP_
#define INCLUDE_VELOCITYKINEMATICS_HPP_

#include <math>
#include <claw.hpp>
#include <forwardKinematics.hpp>

class VelocityKinematics {
    public:
        VelocityKinematics();

        ~VelocityKinematics();

        std::vector<double> jacobian(std::vector<double> JointAngles, int n);

    private:
        Claw claw_;
};

#endif  //  INCLUDE_VELOCITYKINEMATICS_HPP_