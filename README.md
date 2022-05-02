<img src="assets/claw.png" width="510"/>

[![License BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/RajPShinde/CLAW/blob/master/LICENSE)
[![Build Status](https://travis-ci.org/RajPShinde/CLAW.svg?branch=master)](https://travis-ci.org/RajPShinde/CLAW)
[![Release](https://img.shields.io/badge/release-0.0.0-green)](https://github.com/RajPShinde/CLAW/releases)
[![Documentation](https://img.shields.io/badge/docs-unknown-lightgrey)](https://github.com/RajPShinde/CLAW/docs)
---

## Overview
An Open Source Quadruped Robot.

## Dependencies
1. C++ 14
2. Python 3
3. ROS Melodic
4. Eigen 3.4.0
5. [libi2c](https://github.com/amaork/libi2c)
6. [odrive_can](https://github.com/swankun/odrive_can)

## TO-DO's
### Kinematics
- [x] Inverse Kinematics
- [x] Forward Kinematics
- [ ] Velocity Kinematics
- [ ] Force/Torque Rel
### Trajectory Planning
- [x] Compound Cycloid
- [x] Jerk Minimized
### Hardware Interface
- [ ] odrive CAN
- [ ] OLED I2C
### Visualization
- [ ] URDF
- [ ] Joint State Publishers
### Manager
- [ ] State Machine

## LICENSE
```
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
```