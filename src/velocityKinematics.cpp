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
 * @file velocityKinematics.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <velocityKinematics.hpp>

VelocityKinematics::VelocityKinematics(){
}

VelocityKinematics::~VelocityKinematics(){
}

Eigen::MatrixXd VelocityKinematics::jacobian(std::vector<double> jointAngles, int n){
    //  https://slideplayer.com/slide/4889400/
    double dir = 1;
    if(n == (2 || 3))
        dir = -1;

    std::vector<std::vector<double>> DH;
    Eigen::Matrix4d h12 = ai(jointAngles[0], DH[0][1], DH[0][2], DH[0][3]);
    Eigen::Matrix4d h13 = h12*ai(jointAngles[1], dir*DH[1][1], DH[1][2], DH[1][3]);
    Eigen::Matrix4d h14 = h13*ai(jointAngles[2], DH[2][1], DH[2][2], DH[2][3]);

    Eigen::Vector3d o1 = {0, 0, 0};
    Eigen::Vector3d o2 = h12.block<3,1>(0,3);
    Eigen::Vector3d o3 = h13.block<3,1>(0,3);
    Eigen::Vector3d o4 = h14.block<3,1>(0,3);

    Eigen::Vector3d z1 = {0, 0, 1};
    Eigen::Vector3d z2 = h12.block<3,1>(0,2);
    Eigen::Vector3d z3 = h13.block<3,1>(0,2);

    Eigen::MatrixXd jacobian(6,3);

    // Linear
    jacobian.block<3, 1>(0,0) = z1.cross(o4-o1);
    jacobian.block<3, 1>(0,1) = z2.cross(o4-o2);
    jacobian.block<3, 1>(0,2) = z3.cross(o4-o3);
    // Angular
    jacobian.block<3, 1>(3,0) = z1;
    jacobian.block<3, 1>(3,1) = z2;
    jacobian.block<3, 1>(3,2) = z3;

    return jacobian;
}

Eigen::VectorXd VelocityKinematics::footVelocities(Eigen::Vector3d jointVelocities, std::vector<double> jointAngles, int n){
    return jacobian(jointAngles, n) * jointVelocities;
}

Eigen::VectorXd VelocityKinematics::reactionForces(Eigen::Vector3d jointTorques, std::vector<double> jointAngles, int n){
}