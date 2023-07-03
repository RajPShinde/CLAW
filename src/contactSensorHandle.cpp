#include <contactSensorHandle.hpp>

ContactSensorHandle::ContactSensorHandle(){

}

ContactSensorHandle::ContactSensorHandle(const std::string& name, const bool* isContact) : name_(name), isContact_(isContact) {
   if (isContact == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. isContact pointer is null.");
   }
}

ContactSensorHandle::~ContactSensorHandle(){

}

std::string ContactSensorHandle::getName() const {
   return name_; 
}

bool ContactSensorHandle::isContact() const {
   assert(isContact_);
   return *isContact_;
}