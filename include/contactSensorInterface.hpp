#ifndef INCLUDE_CONTACTSENSORINTERFACE_HPP_
#define INCLUDE_CONTACTSENSORINTERFACE_HPP_

#include <contactSensorHandle.hpp>

class ContactSensorInterface : public hardware_interface::HardwareResourceManager<ContactSensorHandle, hardware_interface::DontClaimResources> {};

#endif  //  INCLUDE_CONTACTSENSORINTERFACE_HPP_