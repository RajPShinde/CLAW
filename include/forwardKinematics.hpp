#ifndef INCLUDE_FORWARDKINEMATICS_HPP_
#define INCLUDE_FORWARDKINEMATICS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <claw.hpp>

class ForwardKinematics {
    public:
        ForwardKinematics();
        
        ~ForwardKinematics();

        getLegPose(double alpha, double beta, double gamma);
};

#endif  //  INCLUDE_FORWARDKINEMATICS_HPP_