#ifndef INCLUDE_TRAJECTORY_HPP_
#define INCLUDE_TRAJECTORY_HPP_

#include <math.h>
#include <polynomial.hpp>
#include <gait.hpp>
#include <data.hpp>

class Trajectory {
    public:
        Trajectory(Gait &obj);
    
        ~Trajectory();


        /**
         * @brief Generates a jerk minimized trajectory between two points
         * 
         * @param x0 x initial
         * @param y0 y initial
         * @param z0 z initial
         * @param xt x final
         * @param yT y final
         * @param zT z final
         * @param t time
         * @return Point
         */
        Point jerkMinimizedTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double t);

        /**
         * @brief Generates leg's swing phase trajectory based on compound cycloid
         * 
         * @param t time
         * @return Point
         */
        Point stancePhaseTrajectory(double t);

        /**
         * @brief Generated leg's support phase trajectory based on constant velocity model 
         * 
         * @param t time
         * @return Point
         */
        Point supportPhaseTrajectory(double t);

    private:
        Gait gait_;
};

#endif  //  INCLUDE_TRAJECTORY_HPP_