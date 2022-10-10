/*
BSD 3-Clause License

Copyright (c) 2022, Raj Shinde
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file trajectory.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_TRAJECTORY_HPP_
#define INCLUDE_TRAJECTORY_HPP_

#include <math.h>
#include <polynomial.hpp>
#include <data.hpp>
#include <ros/ros.h>

class Trajectory {
    public:
        Trajectory(Gait &obj, std::string legTrajectoryType_);
    
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
        Point jerkMinimizedTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double T, double t);

        /**
         * @brief Generates leg's swing phase trajectory based on compound cycloid
         * 
         * @param t time
         * @return Point
         */
        Eigen::Vector3d swingPhaseTrajectory(double t, int cycle);

        /**
         * @brief Generated leg's support phase trajectory based on constant velocity model 
         * 
         * @param t time
         * @return Point
         */
        Eigen::Vector3d stancePhaseTrajectory(double t, int cycle);

        long int factorial(int n);

        void scaleControlPointsX(double newStride);

    private:
        Gait gait_;
        std::string legTrajectoryType_;
        std::vector<Point> controlPoints_;
};

#endif  //  INCLUDE_TRAJECTORY_HPP_