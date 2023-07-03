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
 * @file status.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#include <status.hpp>

Status::Status(){
   initialize();
   setColour(ledData_, 5, powerStatusColor);
}

Status::~Status(){
   memset(ledData_, 0, sizeof(ledData_));
   sendLedData(spiDevice_, ledData_, sizeof(ledData_));
   close(spiDevice_);
}

void Status::initialize(){
   spiDevice_ = open(Claw::WS2812B_PORT, O_RDWR);
   if (spiDevice_ < 0) {
      std::cerr << "Failed to open SPI device" << std::endl;
      int error_value = errno;
      printf("Failed to open. Returned fd = %d and errno = %d\n", spiDevice_, error_value);
      return;
   }

   // Configure SPI mode and bits per word
   uint8_t mode = SPI_MODE_0;
   uint8_t bitsPerWord = 8;
   if (ioctl(spiDevice_, SPI_IOC_WR_MODE, &mode) < 0 ||
      ioctl(spiDevice_, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) < 0) {
      std::cerr << "Failed to configure SPI" << std::endl;
      close(spiDevice_);
      return;
   }
}

void Status::spiTransfer(int spiDevice, uint8_t* data, int length) {
    struct spi_ioc_transfer spi;
    memset(&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long)data;
    spi.rx_buf = 0;
    spi.len = length;
    spi.delay_usecs = 0;
    spi.speed_hz = 8000000;  // SPI clock speed (8MHz)
    spi.bits_per_word = 8;
    spi.cs_change = 0;
    ioctl(spiDevice, SPI_IOC_MESSAGE(1), &spi);
}

void Status::sendLedData(int spiDevice, uint8_t* ledData, int length) {
    uint8_t spiData[length * 8];
    for (int i = 0; i < length; ++i) {
        uint8_t value = ledData[i];
        for (int j = 7; j >= 0; --j) {
            spiData[i * 8 + j] = (value & 0x80) ? 0xF8 : 0xC0;
            value <<= 1;
        }
    }
    spiTransfer(spiDevice, spiData, sizeof(spiData));
}

void Status::setColour(uint8_t (&ledData)[Claw::ledCount * 3], int num, int color[3]){
    ledData[num * 3] = color[0] ;
    ledData[num * 3 + 1] =  color[1]; 
    ledData[num * 3 + 2] = color[2];
}

void Status::displayContactState(bool contactState[4]){
   for(int i = 0; i < 4; i++){
      if(contactState[i])
         setColour(ledData_, i, contactStatusColor);
      else{
         int color[3] = {0, 0, 0};
         setColour(ledData_, i, color);
      }
   }
   sendLedData(spiDevice_, ledData_, sizeof(ledData_));
}

void Status::setContactStatusColor(int red, int blue, int green){
   contactStatusColor[0] = red;
   contactStatusColor[1] = blue;
   contactStatusColor[2] = green;
}

void Status::setPowerStatusColor(int red, int blue, int green){
   powerStatusColor[0] = red;
   powerStatusColor[1] = blue;
   powerStatusColor[2] = green;
}

