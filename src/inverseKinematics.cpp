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
 * @file inverseKinematics.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <inverseKinematics.hpp>

InverseKinematics::InverseKinematics(){
}

InverseKinematics::~InverseKinematics(){
}

std::vector<double> InverseKinematics::computeJointAngles(double x, double y, double z, int n){
    
    double alpha, beta, gamma, r;
    r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

    if(n == (1 || 4))
        alpha = std::atan2(y, x) - std::atan2(Claw::d, std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)));
    else
        alpha = std::atan2(y, x) + std::atan2( -1 * Claw::d, std::sqrt(std::pow(r, 2) - std::pow(Claw::d, 2)));

    double cosGamma = (std::pow(r - Claw::a, 2) + std::pow(z, 2) - std::pow(Claw::link1, 2) - std::pow(Claw::link2, 2))/(2 * Claw::link1 * Claw::link2);
    gamma = std::atan2(-std::sqrt(1 - std::pow(cosGamma, 2)), cosGamma);

    beta = std::atan2(z, r - Claw::a) - std::atan2(Claw::link2 * std::sin(gamma), Claw::link1 + Claw::link2 * std::cos(gamma));

    return {alpha, beta, gamma};
}