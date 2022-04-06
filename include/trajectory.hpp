#ifndef INCLUDE_TRAJECTORY_HPP_
#define INCLUDE_TRAJECTORY_HPP_

#include <polynomial.hpp>
#include <gait.hpp>

class Trajectory {
    public:
        Trajectory();
    
        ~Trajectory();

        /**
         * @brief Generates a jerk minimized trajectory between two points
         * 
         * @param t time
         * @return std::pair<double, double> 
         */
        std::pair<double, double> jerkMinimizedTrajectory(double t);

        /**
         * @brief Generates leg's swing phase trajectory based on cycloidal motion
         * 
         * @param t time
         * @return std::pair<double, double> 
         */
        std::pair<double, double> stancePhaseTrajectory(double t);

        /**
         * @brief Generated leg's support phase trajectory based on constant velocity model 
         * 
         * @param t time
         * @return std::pair<double, double>
         */
        std::pair<double, double> supportPhaseTrajectory(double t);

    private:
        Gait gait_;
        const double PI_ = 3.1415926;
};

#endif  //  INCLUDE_TRAJECTORY_HPP_