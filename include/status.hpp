/*
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
*/

/**
 * @file status.hpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#ifndef INCLUDE_STATUS_HPP_
#define INCLUDE_STATUS_HPP_

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <claw.hpp>

class Status {

   public:
      Status();
      
      ~Status();

      void initialize();

      void spiTransfer(int spiDevice, uint8_t* data, int length);

      void sendLedData(int spiDevice, uint8_t* ledData, int length);

      void setColour(uint8_t (&ledData)[Claw::ledCount * 3], int num, int color[3]);

      void displayContactState(bool contactState[4]);

      void setContactStatusColor(int red, int blue, int green);

      void setPowerStatusColor(int red, int blue, int green);

      int contactStatusColor[3] = {255, 255, 255};

      int powerStatusColor[3] = {255, 255, 255};


   private:
      int spiDevice_;
      uint8_t ledData_[Claw::ledCount * 3];

};

#endif  //  INCLUDE_STATUS_HPP_