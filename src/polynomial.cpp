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
 * @file polynomial.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#include <polynomial.hpp>

Polynomial::Polynomial(double x0, double v0, double a0, double xT, double vT, double aT, double T):
					   x0_(x0), v0_(v0), a0_(a0), xT_(xT), vT_(vT), aT_(aT), T_(T), a1_(x0), a2_(v0), a3_(a0/2) {
	Eigen::Matrix3d A;
	Eigen::Vector3d B;
	A << std::pow(T_,3), std::pow(T_,4), std::pow(T_,5),
		 3 * std::pow(T_,2), 4 * std::pow(T_,3), 5 * std::pow(T_,4),
		 6 * T_, 12 * std::pow(T_,2), 20 * std::pow(T_,3);

	B << xT - a1_ - a2_ * T_ - a3_ * std::pow(T_,2),
		 vT - a2_ - 2 * a3_ * T_,
		 aT - 2 * a3_;

	//  Solve for x in Ax=B
	Eigen::Vector3d coefficients = A.colPivHouseholderQr().solve(B);
	a4_ = coefficients(0); 
	a5_ = coefficients(1);
	a6_ = coefficients(2);
}

Polynomial::~Polynomial(){
}

double Polynomial::position(double t){
    return a1_ + a2_ * t + a3_ * std::pow(t,2) + a4_ * std::pow(t,3) + a5_ * std::pow(t,4) + a6_ * std::pow(t,5);
}

double Polynomial::velocity(double t){
    return a2_ + 2 * a3_ * t + 3 * a4_ * std::pow(t,2) + 4 * a5_ * std::pow(t,3) + 5 * a6_ * std::pow(t,4);
}

double Polynomial::acceleration(double t){
    return 2 * a3_ + 6 * a4_ * t + 12 * a5_ * std::pow(t,2) + 20 * a6_ * std::pow(t,3);
}

double Polynomial::jerk(double t){
    return 6 * a4_ + 24 * a5_ * t + 60 * a6_ * std::pow(t,2);
}