#ifndef INTERFACE_H
#define INTERFACE_H

#include <USBHID.h>

// HID report descriptor for a generic knob device
static const uint8_t HIDReportDescriptor[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x0E,       // Usage (Dial)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x37,       //   Usage (Dial)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8 bits)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)
    0xC0              // End Collection
};

class CustomHIDDevice {
public:
    CustomHIDDevice();

    void begin();
    void sendDialPosition(uint8_t position);

private:
    USBHID HID;
};

#endif // INTERFACE_H
