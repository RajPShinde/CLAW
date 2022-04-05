#ifndef INCLUDE_TRAJECTORY_HPP_
#define INCLUDE_TRAJECTORY_HPP_

#include <polynomial.hpp>

class Trajectory {
    public:
        Trajectory();
    
        ~Trajectory();

        /**
         * @brief 
         * 
         * @param t time
         * @return std::pair<double, double> 
         */
        std::pair<double, double> baseTrajectory(double t);

        /**
         * @brief 
         * 
         * @param t time
         * @return std::pair<double, double> 
         */
        std::pair<double, double> stancePhaseTrajectory(double t);

        /**
         * @brief 
         * 
         * @param t time
         * @return std::pair<double, double>
         */
        std::pair<double, double> supportPhaseTrajectory(double t);

    private:
};

#endif  //  INCLUDE_TRAJECTORY_HPP_