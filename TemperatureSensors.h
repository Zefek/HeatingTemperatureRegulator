#ifndef __TEMPERATURESENSORS_H__
#define __TEMPERATURESENSORS_H__

#include <DallasTemperature.h>

class TemperatureSensors
{
  private:
    OneWire* oneWire;
    DallasTemperature* sensors;
    void PrintAddress(DeviceAddress deviceAddress);
    bool GetTemperature(DeviceAddress deviceAddress, uint8_t* temperature);
  public:
    TemperatureSensors(uint8_t busPin);
    bool GetAcumulator1Temperature(uint8_t* temperature);
    bool GetAcumulator2Temperature(uint8_t* temperature);
    bool GetAcumulator3Temperature(uint8_t* temperature);
    bool GetAcumulator4Temperature(uint8_t* temperature);
    bool GetAcumulatorOutputTemperature(uint8_t* temperature);
    bool GetReturnHeatingTemperature(uint8_t* temperature);
    bool GetCurrentHeatingTemperature(uint8_t* temperature);
    bool GetHeaterTemperature(uint8_t* temperature);
    bool GetBoilerTemperature(uint8_t* temperature);
    void Init();
    void RequestTemperatures();
};
#endif