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
 * @file forwardKinematics.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_FORWARDKINEMATICS_HPP_
#define INCLUDE_FORWARDKINEMATICS_HPP_

#include <math.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <claw.hpp>

class ForwardKinematics {

    public:
        ForwardKinematics();
        
        ~ForwardKinematics();

        Eigen::Matrix4d translationH(double xTrans, double yTrans, double zTrans);

        Eigen::Matrix4d rotationH(double xRot, double yRot, double zRot);

        Eigen::Matrix4d inverseH(Eigen::Matrix4d H);

        Eigen::Matrix4d ai(double zRot, double d, double a, double xRot);

        Eigen::Matrix4d worldToBaseH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot);

        Eigen::Matrix4d baseToLegH(int n);

        Eigen::Matrix4d worldToLegH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, int n);

        Eigen::Matrix4d legToFootH(std::vector<double> JointAngles, int n);

        Eigen::Vector3d footInLegFrame(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, Eigen::Vector3d p, int n);

        Eigen::Vector3d trajectoryToLegH(Eigen::Vector3d p);
};

#endif  //  INCLUDE_FORWARDKINEMATICS_HPP_