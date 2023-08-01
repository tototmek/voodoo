#include "HIDController.h"

namespace hid {

constexpr uint8_t reportId = 0x03;
constexpr size_t reportDescriptorSize = 36;
constexpr PROGMEM uint8_t reportDescriptor[] {
    0x05, 0x01,           // USAGE_PAGE (Generic Desktop)    
    0x85, reportId,  // REPORT_ID
    0x09, 0x04,           // USAGE (Joystick)
    0x75, 0x10,		        // REPORT_SIZE (16)
    0x95, axisCount,		  // REPORT_COUNT (axisCount)
    0x15, 0x00,           // LOGICAL_MIN(0)
    0x26, 0xff, 0x03,     // LOGICAL_MAX(1023)
    0xA1, 0x00,           // COLLECTION (Physical)
    0x09, 0x30,           // USAGE
    0x09, 0x31,           // USAGE
    0x09, 0x32,           // USAGE
    0x09, 0x33,           // USAGE
    0x09, 0x34,           // USAGE
    0x09, 0x35,           // USAGE
    0x09, 0x36,           // USAGE
    0x09, 0x37,           // USAGE
    0x81, 0x02,           // INPUT (Data, Var, Abs)
    0xc0                  // END_COLLECTION
  };

HIDController::HIDController()
{
  using D = DynamicHIDSubDescriptor;
  D* node = new D(reportDescriptor, reportDescriptorSize);
	DynamicHID().AppendDescriptor(node);
}

void HIDController::setAxis(int index, int32_t value) {
  writeAsBytesAt(value, &(reportData[2 * index]));
}

int HIDController::writeAsBytesAt(int32_t value, uint8_t dataLocation[]) 
{
	dataLocation[0] = (uint8_t)(value & 0x00FF);
	dataLocation[1] = (uint8_t)(value >> 8);
}

void HIDController::sendReport()
{
	DynamicHID().SendReport(reportId, reportData, reportSize);
}

} // namespace hid