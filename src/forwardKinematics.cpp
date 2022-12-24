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
 * @file forwardKinematics.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <forwardKinematics.hpp>

ForwardKinematics::ForwardKinematics(){
}

ForwardKinematics::~ForwardKinematics(){
}

Eigen::Matrix4d ForwardKinematics::translationH(double x, double y, double z){
    Eigen::Transform<double, 3, Eigen::Affine> transH;
    transH = Eigen::Translation<double, 3> (Eigen::Vector3d(x ,y, z));
    return transH.matrix();
}

Eigen::Matrix4d ForwardKinematics::rotationH(double roll, double pitch, double yaw){
    Eigen::Transform<double, 3, Eigen::Affine> rotH = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    rotH.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    rotH.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    rotH.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return rotH.matrix();
}

Eigen::Matrix4d ForwardKinematics::inverseH(Eigen::Matrix4d H){
    // invH = |invR -d|
    //        |  0   1|
    Eigen::Matrix3d inverseR = H.block<3,3>(0,0);
    inverseR.transposeInPlace();

    Eigen::Vector3d negatived = H.block<3,1>(0,3);
    negatived = negatived * -1;

    Eigen::Matrix4d a = Eigen::Matrix4d::Identity();
    a.block<3,3>(0,0) = inverseR;
    Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
    b.block<3,1>(0,3) = negatived;
    return a * b;
}

Eigen::Matrix4d ForwardKinematics::ai(double theta, double d, double a, double alpha){
    Eigen::Matrix4d aiH;
    aiH << std::cos(theta), -1*std::sin(theta)*std::cos(alpha), std::sin(theta)*std::sin(alpha),    a*std::cos(theta),
          std::sin(theta), std::cos(theta)*std::cos(alpha),    -1*std::cos(theta)*std::sin(alpha), a*std::sin(theta),
          0              , std::sin(alpha)                ,    std::cos(alpha)                  ,  d,
          0              , 0                              ,    0                                ,  1;
    return aiH;
}

Eigen::Matrix4d ForwardKinematics::worldToBaseH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot){
    return translationH(xTrans, yTrans, zTrans) * rotationH(xRot, yRot, zRot);
}

Eigen::Matrix4d ForwardKinematics::baseToLegH(int n){
    return translationH(Claw::bodyTF[n-1][0], Claw::bodyTF[n-1][1], Claw::bodyTF[n-1][2]) * rotationH(Claw::bodyTF[n-1][3], Claw::bodyTF[n-1][4], Claw::bodyTF[n-1][5]);
}

Eigen::Matrix4d ForwardKinematics::worldToLegH(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, int n){
    return worldToBaseH(xTrans, yTrans, zTrans, xRot, yRot, zRot) * baseToLegH(n);
}

Eigen::Vector3d ForwardKinematics::legToFootH(std::vector<double> jointAngles, int n){
    double dir = 1;
    if(n == (2 || 3))
        dir = -1;
    return (ai(jointAngles[0], Claw::DH[0][0], Claw::DH[0][1], Claw::DH[0][2]) * ai(jointAngles[1], dir*Claw::DH[1][0], Claw::DH[1][1], Claw::DH[1][2]) * ai(jointAngles[2], Claw::DH[2][0], Claw::DH[2][1], Claw::DH[2][2])).block<3,1>(0,3);
}

Eigen::Vector3d ForwardKinematics::footInLegFrame(double xTrans, double yTrans, double zTrans, double xRot, double yRot, double zRot, Eigen::Vector3d p, int n){
    Eigen::Vector4d point(p(0), p(1), p(2), 1);
    return (inverseH(worldToLegH(xTrans, yTrans, zTrans, xRot, yRot, zRot, n)) * point).block<3,1>(0,0);
}

Eigen::Vector3d ForwardKinematics::trajectoryToLegH(Eigen::Vector3d p){
    Eigen::Vector4d point(p(0), p(1), p(2), 1);
    return (translationH(Claw::idleLegHeight, 0, 0) * rotationH(-M_PI/2, 0, M_PI/2) * point).block<3,1>(0,0);
}