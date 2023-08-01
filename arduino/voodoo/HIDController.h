#include "DynamicHID.h"

namespace hid {

constexpr uint8_t axisCount = 8;
constexpr uint8_t reportSize = axisCount * 2;

class HIDController {
   private:
    int32_t axes_[axisCount] = {0};
    uint8_t reportData[reportSize] = {0};
    int writeAsBytesAt(int32_t value, uint8_t location[]);

   public:
    HIDController();
    void setAxis(int index, int32_t value);
    void sendReport();
};

} // namespace hid