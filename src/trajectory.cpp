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
 * @file trajectory.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <trajectory.hpp>

Trajectory::Trajectory(Gait &obj, std::string s): gait_(obj), legTrajectoryType_(s){
    controlPoints_ = {{-0.06, 0, 0},
                      {-0.08, 0, 0},
                      {-0.11, 0.05, 0},
                      {-0.11, 0.05, 0},
                      {-0.11, 0.05, 0},
                      {0, 0.05, 0},
                      {0, 0.05, 0},
                      {0, 0.08, 0},
                      {0.11, 0.08, 0},
                      {0.11, 0.08, 0},
                      {0.08, 0, 0},
                      {0.06, 0, 0}};
}

Trajectory::~Trajectory(){
}

Point Trajectory::jerkMinimizedTrajectory(double x0, double y0, double z0, double xT, double yT, double zT, double T, double t){
    Polynomial pX(x0, 0, 0, xT, 0, 0, T);
    Polynomial pY(y0, 0, 0, yT, 0, 0, T);
    Polynomial pZ(z0, 0, 0, zT, 0, 0, T);

    return {pX.position(t), pY.position(t), pZ.position(t)};
}

Eigen::Vector3d Trajectory::swingPhaseTrajectory(double t, int cycle) {
    Eigen::Vector3d p;
    double x, y;

    if(legTrajectoryType_ == "compoundCycloid"){
        double c2, c3;
        c2 = -gait_.stride/2 - (((-1*gait_.stride*gait_.te*std::sin(M_PI*gait_.te/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*gait_.te)/(2*gait_.tm)));
        c3 = gait_.stride/2;
        if(t >= 0 && t <= gait_.te)
            x = ((-1*gait_.stride*gait_.te*std::sin(M_PI*t/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*t)/(2*gait_.tm)) +c2;
        else if(t >= gait_.te && t <= (gait_.te + gait_.tn))
            x = gait_.stride * (((t-gait_.te)/gait_.tn) - (std::sin(2*M_PI*(t-gait_.te)/gait_.tn)/(2*M_PI))) - gait_.stride/2;
        else if(t >= (gait_.te + gait_.tn) && t <= gait_.tm)
            x = ((gait_.stride*gait_.te*std::sin(M_PI*(t-gait_.te-gait_.tn)/gait_.te))/(2*gait_.tm*M_PI)) - ((gait_.stride*(t-gait_.te-gait_.tn))/(2*gait_.tm)) + c3;

        if(t>=0 && t<gait_.tm/2)
            y = 2*gait_.height * ((t/gait_.tm) - (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));
        else if(t>=gait_.tm/2 && t<gait_.tm)
            y = 2*gait_.height * (1 - (t/gait_.tm) + (std::sin(4*M_PI*t/gait_.tm)/(4*M_PI)));
    }
    else if(legTrajectoryType_ == "bezier"){
        double offset = 0;

        if(cycle == 1){
            scaleControlPointsX(gait_.stride/2);
            offset = gait_.stride/4;
        }
        else if(cycle == -1){
            scaleControlPointsX(gait_.stride/2);
            offset = -gait_.stride/4;
        }

        t = (1/gait_.tSwing)*t;
        int n = controlPoints_.size() - 1;
        for(int i = 0; i < controlPoints_.size(); i++)
            {
                float coeff = factorial(n) / (factorial(i) * factorial(n - i));
                x += coeff * std::pow(t, i) * std::pow((1 - t), (n - i)) * (controlPoints_[i].x + offset) * (gait_.stride == 0 ? 0 : 1);
                y += coeff * std::pow(t, i) * std::pow((1 - t), (n - i)) * controlPoints_[i].y;
            }
    }

    p << x, y, 0;
    return p;
}

Eigen::Vector3d Trajectory::stancePhaseTrajectory(double t, int cycle) {
    Eigen::Vector3d p;
    double x, y;

    if(legTrajectoryType_ == "compoundCycloid"){
        t = gait_.tm - t;
        double c2 = ((-1*gait_.stride*gait_.te*std::sin(M_PI*gait_.te/gait_.te))/(2*gait_.tm*M_PI)) + ((gait_.stride*gait_.te)/(2*gait_.tm));
        double xd =(gait_.stride -  2*c2) / gait_.tm;
        x = xd*t - (gait_.stride-2*c2)/2;
        y = 0;
    }
    else if(legTrajectoryType_ == "bezier"){
        double stride = gait_.stride;
        double offset = 0;
        if(cycle == 1){
            stride = gait_.stride/2;
            offset = gait_.stride/4;
        }
        else if(cycle == -1){
            stride = gait_.stride/2;
            offset = -gait_.stride/4;            
        }

        t = gait_.tStance- t;
        double xd =stride / gait_.tStance;
        x = xd*t - (stride/2) - offset;
        y = 0;
    }

    p << x, y, 0;
    return p;
}

long int Trajectory::factorial(int n){
    return std::tgamma(n+1);
}

void Trajectory::scaleControlPointsX(double newStride){
    double ratio = newStride/gait_.stride;

    for(int i =0; i<controlPoints_.size(); i++){
        if(i == 0)
            controlPoints_[i].x = -newStride/2;
        else if(i == 11)
            controlPoints_[i].x = newStride/2;
        else
            controlPoints_[i].x = controlPoints_[i].x * ratio;
    }
}
