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
 * @file claw.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2022-10-09
 * 
 * @copyright BSD 3-Clause License, Copyright (c) 2022
 * 
 */

#ifndef INCLUDE_CLAW_HPP_
#define INCLUDE_CLAW_HPP_

#include <vector>
#include <map>
#include <math.h>

class Claw {
    public:
         // Mechanical Parameters
         static constexpr double femur = 0.2245;
         static constexpr double tibia = 0.238;
         static constexpr double link1 = 0.2245; // femur
         static constexpr double link2 = 0.238; // tibia
         static constexpr double a = 0.03617;
         static constexpr double d = 0.00;
         static constexpr double idleLegHeight = 0.42;
         static constexpr double legMaxHeight = 0.50;
         static constexpr double legMinHeight = 0.20;
         static constexpr int noOfActuators = 12;
         static constexpr double reductionHA = 3;
         static constexpr double reductionHF = 10.8;
         static constexpr double reductionKF = 10.8;
         static constexpr double bodyTF[4][6] = {{ 0.066 + 0.173,  0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Front Left Shoulder
                                                 { 0.066 + 0.173, -0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Front Right Shoulder
                                                 {-0.066 - 0.173, -0.141, -0.031 - 0.036, 0, M_PI/2, 0},   // To Rear Left Shoulder
                                                 {-0.066 - 0.173,  0.141, -0.031 - 0.036, 0, M_PI/2, 0}};  // To Rear Right Shoulder

                                         //  d  a  alpha
         static constexpr double DH[3][3] = {{0, a,     -90},  // HA->HF
                                             {d, link1,   0},  // HF->KF
                                             {0, link2,   0}}; // KF->Foot

         // Electrical Parameters
         static constexpr double kv = 90;
         static constexpr double kt = kv*2*3.142/60;
         static constexpr double maxCurrent = 30;
         static constexpr double minBatteryVoltage = 22.2;
         static constexpr double countsPerRevolution = 2000;
         static constexpr double abductionCPRAngleRelation = (2*M_PI) / (countsPerRevolution *reductionHA);
         static constexpr double flexionCPRAngleRelation = (2*M_PI) / (countsPerRevolution *reductionHF);
         static constexpr double encoderOffset[4][3] = {{-157, -149, -7431},
                                                        {57, 923, 8329},
                                                        {-109, 261, 8875},
                                                        {160, -1125, -8737}};

         static constexpr int encoderDirection[4][3] = {{-1, 1, -1},
                                                        {-1, -1, 1},
                                                        {1, -1, 1},
                                                        {1, 1, -1}};


        // Network
        static constexpr char canDevice[] = "can0";

        // General Parameters
        static constexpr char legConfiguration[] = ">>";
};

#endif  //  INCLUDE_CLAW_HPP_