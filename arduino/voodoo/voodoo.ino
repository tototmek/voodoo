#include <Adafruit_MCP3008.h>

#include "HIDController.h"

hid::HIDController hidController;
Adafruit_MCP3008 adc;

constexpr uint8_t averagingWindowLength = 32;
constexpr uint8_t axisCount = hid::axisCount;
static uint16_t jointValueBuffer[axisCount][averagingWindowLength] = {0};

void initializeBuffer() {
  uint16_t adcMeasurement;
  for (int i=0; i<axisCount; ++i) {
    adcMeasurement = adc.readADC(i);
    for (int j=0; j<averagingWindowLength; ++j) {
      jointValueBuffer[i][j] = adcMeasurement;
    }
  }
}

void bufferPushBackMeasurement() {
  for (int i=0; i<axisCount; ++i) {
    for (int j=0; j<averagingWindowLength-1; ++j) {
      jointValueBuffer[i][j] = jointValueBuffer[i][j+1];
    }
    jointValueBuffer[i][averagingWindowLength-1] = adc.readADC(i);
  }
}

void setAveragedAxisValues() {
  int average;
  for (int i=0; i<axisCount; ++i) {
    average = 0;
    for (int j=0; j<averagingWindowLength; ++j) {
      average += jointValueBuffer[i][j];
    }
    average /= averagingWindowLength;
    hidController.setAxis(i, average);
  }
}


void setup()
{
  adc.begin(/*chipSelectPin=*/10);
  initializeBuffer();
}

void loop()
{
  bufferPushBackMeasurement();
  setAveragedAxisValues();
  hidController.sendReport();
  delay(5);
}
