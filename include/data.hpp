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
 * @file data.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */
 
#ifndef INCLUDE_DATA_HPP_
#define INCLUDE_DATA_HPP_

struct Point{
    double x;
    double y;
    double z;

    Point(){};
    Point(double a, double b, double c): x(a), y(b), z(c) {};

    bool operator == (const Point &p) const {
        return (x == p.x && y == p.y && z == p.z);
    }
};

struct Pose{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

struct LegData{
    double n;
    double alpha;
    double beta;
    double gamma;
    Point p;
};

struct OdrivePosition{
    double alphaPosition;
    double betaPosition;
    double gammaPosition;
};

struct Gait{
    double stride;
    double height;

    double tSwing;
    double tStance;
    double tDelay;
    
    double te = 0.1;
    double tm = 0.15;
    double tn = 0.13;
    
    std::string type;

    Gait(){};
    Gait(double a, double b, double c, double d, std::string e): stride(a), height(b), tSwing(c), tStance(d), tDelay(tStance - tSwing), type(e) {};
};

struct ImuData {
  double orientation[4];
  double oorientationCovariance[9];
  double angularVelocity[3]; 
  double angularVelocityCovarinace[9]; 
  double linearAcceleration[3]; 
  double linearAccelerationCovariance[9];
};

struct ActuatorData {
  double position, velocity, torque;  
  double positionDesired, velocityDesired, kp, kd, ff;
};

#endif  //  INCLUDE_DATA_HPP_