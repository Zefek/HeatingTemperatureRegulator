#ifndef __TEMPERATURESENSORS_H__
#define __TEMPERATURESENSORS_H__

#include <DallasTemperature.h>

class TemperatureSensors
{
  private:
    OneWire* oneWire;
    DallasTemperature* sensors;
    void PrintAddress(DeviceAddress deviceAddress);
    void GetTemperature(DeviceAddress deviceAddress, uint8_t* temperature);
  public:
    TemperatureSensors(uint8_t busPin);
    void GetAcumulator1Temperature(uint8_t* temperature);
    void GetAcumulator2Temperature(uint8_t* temperature);
    void GetAcumulator3Temperature(uint8_t* temperature);
    void GetAcumulator4Temperature(uint8_t* temperature);
    void GetAcumulatorOutputTemperature(uint8_t* temperature);
    void Init();
};
#endif