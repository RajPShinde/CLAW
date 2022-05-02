#ifndef INCLUDE_INVERSEKINEMATICS_HPP_
#define INCLUDE_INVERSEKINEMATICS_HPP_

/**
 * @file inverseKinematics.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <math.h>
#include <claw.hpp>

class InverseKinematics {
    public:
        InverseKinematics();

        ~InverseKinematics();
        /**
         * @brief Compute Leg Joint angles using inverse kinematics equations
         *        REF- Spong 103
         * @param x x in leg frame
         * @param y y in leg frame
         * @param z z in leg frame
         * @return std::vector<double> 
         */
        std::vector<double> computeJointAngles(double x, double y, double z, int n);

    private:
        Claw claw_;
};

#endif  //  INCLUDE_INVERSEKINEMATICS_HPP_