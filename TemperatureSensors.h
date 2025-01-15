#ifndef __TEMPERATURESENSORS_H__
#define __TEMPERATURESENSORS_H__

#include <DallasTemperature.h>

class TemperatureSensors
{
  private:
    OneWire* oneWire;
    DallasTemperature* sensors;
    void PrintAddress(DeviceAddress deviceAddress);
  public:
    TemperatureSensors(uint8_t busPin);
    uint8_t GetAcumulator1Temperature();
    uint8_t GetAcumulator2Temperature();
    uint8_t GetAcumulator3Temperature();
    uint8_t GetAcumulator4Temperature();
    uint8_t GetAcumulatorOutputTemperature();

    void Loop();
    void Init();
};
#endif