#include "TemperatureSensors.h"
#include "sensorsconfig.h"

TemperatureSensors::TemperatureSensors(uint8_t busPin)
{
  this->oneWire = new OneWire(busPin);
  this->sensors = new DallasTemperature(oneWire);
}

void TemperatureSensors::Init()
{
  uint8_t count = sensors->getDeviceCount();
  DeviceAddress addresses[count];
  sensors->requestTemperatures();
  for(int i = 0; i < count; i++)
  {
    DeviceAddress address;
    sensors->getAddress(address, 0);
    this->PrintAddress(address);
    float temp = sensors->getTempC(address);
    Serial.print(": ");
    Serial.println(temp);
  }
}

void TemperatureSensors::Loop()
{
  sensors->requestTemperatures();
}

uint8_t TemperatureSensors::GetAcumulator1Temperature()
{
  return (uint8_t)round(sensors->getTempC(acumulatorFirst));
}

uint8_t TemperatureSensors::GetAcumulator2Temperature()
{
  return (uint8_t)round(sensors->getTempC(acumulatorSecond));
}

uint8_t TemperatureSensors::GetAcumulator3Temperature()
{
  return (uint8_t)round(sensors->getTempC(acumulatorThird));
}

uint8_t TemperatureSensors::GetAcumulator4Temperature()
{
  return (uint8_t)round(sensors->getTempC(acumulatorFour));
}

uint8_t TemperatureSensors::GetAcumulatorOutputTemperature()
{
  return (uint8_t)round(sensors->getTempC(acumulatorOutput));
}

void TemperatureSensors::PrintAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) 
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}