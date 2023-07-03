#ifndef INCLUDE_HYBRIDJOINTINTERFACE_HPP_
#define INCLUDE_HYBRIDJOINTINTERFACE_HPP_

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <hybridJointHandle.hpp>

class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

#endif  //  INCLUDE_HYBRIDJOINTINTERFACE_HPP_