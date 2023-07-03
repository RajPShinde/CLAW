#ifndef INCLUDE_CONTACTSENSORHANDLE_HPP_
#define INCLUDE_CONTACTSENSORHANDLE_HPP_

#include <hardware_interface/joint_state_interface.h>

class ContactSensorHandle {
    public:
        ContactSensorHandle();

        ContactSensorHandle(const std::string& name, const bool* isContact);

        ~ContactSensorHandle();

        std::string getName() const;

        bool isContact() const;

    private:
        std::string name_;

        const bool* isContact_ = nullptr;
};

#endif  //  INCLUDE_CONTACTSENSORHANDLE_HPP_