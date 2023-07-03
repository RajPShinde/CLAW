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
         static constexpr double tibia = 0.2245;
         static constexpr double link1 = 0.2245; // femur
         static constexpr double link2 = 0.2245; // tibia
         static constexpr double a = 0.03315;
         static constexpr double d = 0.00;
         static constexpr double idleLegHeight = 0.43;
         static constexpr double legMaxHeight = 0.50;
         static constexpr double legMinHeight = 0.20;
         static constexpr int noOfActuators = 12;
         static constexpr double reductionHAA = 5.25;
         static constexpr double reductionHFE = 10.8;
         static constexpr double reductionKFE = 10.8;
         static constexpr double bodyTF[4][6] = {{ 0.222,  0.139, -0.031 - 0.03315, 0, M_PI/2, 0},   // To Front Left Shoulder
                                                 { 0.222, -0.139, -0.031 - 0.03315, 0, M_PI/2, 0},   // To Front Right Shoulder
                                                 {-0.222, -0.139, -0.031 - 0.03315, 0, M_PI/2, 0},   // To Rear Left Shoulder
                                                 {-0.222,  0.139, -0.031 - 0.03315, 0, M_PI/2, 0}};  // To Rear Right Shoulder

                                         //  d  a  alpha
         static constexpr double DH[3][3] = {{0, a,     -90},  // HA->HF
                                             {d, link1,   0},  // HF->KF
                                             {0, link2,   0}}; // KF->Foot
         static constexpr int RF = 0;
         static constexpr int LF = 1;
         static constexpr int RH = 2;
         static constexpr int LH = 3;

         // Electrical Parameters
         static constexpr double kv = 90;
         static constexpr double kt = 8.27/kv;
         static constexpr double maxCurrent = 30;
         static constexpr double minBatteryVoltage = 22.2;
         static constexpr double batteryLimit = 23;
         static constexpr double countsPerRevolution = 2000;
         static constexpr int ledCount = 6;
         static constexpr double abductionCPRAngleRelation = (2 * M_PI) / (countsPerRevolution * reductionHAA);
         static constexpr double flexionCPRAngleRelation = (2 * M_PI) / (countsPerRevolution * reductionHFE);
         static constexpr double encoderOffset[4][3] = {{-175, -1934, -7934},
                                                        {114, 1511, 7926},
                                                        {-41, 813, 7876},
                                                        {30, -1370, -7711}};
         static constexpr int encoderDirection[4][3] = {{-1, 1, -1},
                                                        {-1, -1, 1},
                                                        {1, -1, 1},
                                                        {1, 1, -1}};
         static constexpr char* contactSensors[4] = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};
         static constexpr double contactThreshold = 2.5;


        // Communications
        static constexpr char ODRIVE_PORT[] = "can0";
        static constexpr char ADS111_PORT[] = "/dev/i2c-3";
        static constexpr char MPU9255_PORT[] = "/dev/i2c-3";
        static constexpr char WS2812B_PORT[] = "/dev/spidev0.0";
        static constexpr uint8_t ADS111_ADDRESS = 0x48;
        static constexpr uint8_t MPU9255_ADDRESS = 0x68;

};

#endif  //  INCLUDE_CLAW_HPP_