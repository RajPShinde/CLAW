#ifndef INCLUDE_INVERSEKINEMATICS_HPP_
#define INCLUDE_INVERSEKINEMATICS_HPP_

#include <math>
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
        std::vector<double> computeJointAngles(double x, double y, double z);

        private:
            Claw claw_;
};

#endif  //  INCLUDE_INVERSEKINEMATICS_HPP_