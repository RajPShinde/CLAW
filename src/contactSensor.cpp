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
 * @file contactSensor.cpp
 * @author Raj Shinde
 * @brief 
 * @version 0.1
 * @date 2023-06-25
 * @copyright BSD 3-Clause License, Copyright (c) 2023
 * 
 */

#include <contactSensor.hpp>

ContactSensor::ContactSensor() : sensorThreadStatus_(true) {
   initialize();
    sensorThread_ = std::thread([&]() {
        while (sensorThreadStatus_) {
            adc_->read(ADS1115::Multiplex::AIN0, a0_);
            adc_->read(ADS1115::Multiplex::AIN1, a1_);
            adc_->read(ADS1115::Multiplex::AIN2, a2_);
            adc_->read(ADS1115::Multiplex::AIN3, a3_);
        }
    });
}

ContactSensor::~ContactSensor(){
   sensorThreadStatus_ = false;
    if (sensorThread_.joinable()) {
        sensorThread_.join();
    }
}

void ContactSensor::initialize(){

   adc_ = std::make_shared<ADS1115::ADC<unix_i2c::i2c>>(Claw::ADS111_PORT, Claw::ADS111_ADDRESS);

   auto config_fsr = ADS1115::FullScaleRange::FSR_6_144V;
   auto config_dr  = ADS1115::DataRate::SPS_860;
   auto config_cm  = ADS1115::ConversionMode::SingleShot;
   auto config_mux = ADS1115::Multiplex::AIN0;

   adc_->set_fsr(config_fsr);
   adc_->set_data_rate(config_dr);
   adc_->set_conversion_mode(config_cm);
   adc_->set_multiplexing(config_mux);

   printConfig();
}

void ContactSensor::read(bool (&contactState)[4]){
   contactState[0] = a0_ > Claw::contactThreshold;
   contactState[1] = a1_ > Claw::contactThreshold;
   contactState[2] = a2_ > Claw::contactThreshold;
   contactState[3] = a3_ > Claw::contactThreshold;
}

void ContactSensor::printConfig(){
   std::cout << "ADC Configuration" << std::endl;
   std::cout << "\tfsr             : " << adc_->get_fsr() << std::endl;
   std::cout << "\tmultiplexing    : " << adc_->get_multiplexing() << std::endl;
   std::cout << "\tdata rate       : " << adc_->get_data_rate() << std::endl;
   std::cout << "\tconversion mode : " << adc_->get_conversion_mode() << std::endl; 
}
