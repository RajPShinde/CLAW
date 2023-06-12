<img src="assets/logo.JPG" width="400"/>

[![License BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/RajPShinde/CLAW/blob/master/LICENSE)
[![Build Status](https://travis-ci.org/RajPShinde/CLAW.svg?branch=master)](https://travis-ci.org/RajPShinde/CLAW)
[![Release](https://img.shields.io/badge/release-0.0.0-green)](https://github.com/RajPShinde/CLAW/releases)
[![Documentation](https://img.shields.io/badge/docs-unknown-lightgrey)](https://github.com/RajPShinde/CLAW/docs)
---

## Overview
<p align="center">
<img src="assets/claw.GIF"/>
</p>
An Open Source Quadruped Robot.
<img src="assets/sponsors.JPG"/>

## Dependencies
1. [ROS Noetic](http://wiki.ros.org/noetic)
2. [OCS2](https://github.com/leggedrobotics/ocs2)
3. [Eigen 3.4.0](https://eigen.tuxfamily.org/index.php?title=Main_Page)
4. [odrive_can](https://github.com/swankun/odrive_can)
5. [U8g2](https://github.com/olikraus/u8g2)

## Hardware
1. UP Xtreme i12- Core i5-1250PE 16GB RAM
2. Odrive 3.6 Motor Controller
3. Intel Realsense D435i
4. 8308 90Kv Outrunner BLDC
5. AS5047P Encoder
6. MCP2515 CAN Controller
7. MPU9265 9-DOF IMU
8. ADS1115 4-Channel ADC
9. WS2812B Addressable LED
10. SHI1106 128X64 OLED
11. MD30-60 Pressure Sensor

## Install & Build
```
mkdir -p ~/claw_ws/src
cd ~/claw_ws/src
git clone https://github.com/RajPShinde/CLAW
```

## TO-DO's
### CAD
- [x] Parts
- [x] Assembly
- [x] URDF
### Hardware Interface
- [x] PCB Design
- [x] SPI Test
- [x] I2C Test (MPU9255 & SH1106)
- [x] Socket CAN Test (MCP2515)
### Kinematics
- [x] Inverse Kinematics
- [x] Forward Kinematics
- [x] Velocity Kinematics
### Controls
- [ ] Hybrid Control (Impedance + Position)
### Trajectory Planning
- [x] Compound Cycloid
- [x] Bezier Curve
- [x] Jerk Minimized
### Visualization
- [x] Joint State Publishers
### Manager
- [ ] State Machine

## LICENSE
```
BSD 3-Clause License

Copyright (c) 2023, Raj Shinde
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
```