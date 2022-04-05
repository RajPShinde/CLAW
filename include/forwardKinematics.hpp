#ifndef INCLUDE_FORWARDKINEMATICS_HPP_
#define INCLUDE_FORWARDKINEMATICS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <claw.hpp>

class ForwardKinematics {
    public:
        ForwardKinematics();
        
        ~ForwardKinematics();

        /**
         * @brief Get the Leg Pose
         * 
         * @param alpha HA angle
         * @param beta HF angle
         * @param gamma KF angle
         * @return Eigen::VectorXd 
         */
        Eigen::VectorXd getLegPose(double alpha, double beta, double gamma);
};

#endif  //  INCLUDE_FORWARDKINEMATICS_HPP_