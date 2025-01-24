#include "TemperatureSensors.h"
#include "sensorsconfig.h"

TemperatureSensors::TemperatureSensors(uint8_t busPin)
{
  this->oneWire = new OneWire(busPin);
  this->sensors = new DallasTemperature(oneWire);
}

void TemperatureSensors::Init()
{
  this->sensors->begin();
  uint8_t count = sensors->getDeviceCount();
  Serial.println(count);
  DeviceAddress addresses[count];
  sensors->requestTemperatures();
  for(int i = 0; i < count; i++)
  {
    DeviceAddress address;
    sensors->getAddress(address, i);
    this->PrintAddress(address);
    this->sensors->setResolution(address, 9);
    float temp = sensors->getTempC(address);
    Serial.print(": ");
    Serial.println(temp);
  }
}

void TemperatureSensors::GetAcumulator1Temperature(uint8_t* temperature)
{
  GetTemperature(acumulatorFirst, temperature);
}

void TemperatureSensors::GetAcumulator2Temperature(uint8_t* temperature)
{
  GetTemperature(acumulatorSecond, temperature);
}

void TemperatureSensors::GetAcumulator3Temperature(uint8_t* temperature)
{
  GetTemperature(acumulatorThird, temperature);
}

void TemperatureSensors::GetAcumulator4Temperature(uint8_t* temperature)
{
  GetTemperature(acumulatorFour, temperature);
}

void TemperatureSensors::GetAcumulatorOutputTemperature(uint8_t* temperature)
{
  GetTemperature(acumulatorOutput, temperature);
}

void TemperatureSensors::GetTemperature(DeviceAddress deviceAddress, uint8_t* temperature)
{
  sensors->requestTemperaturesByAddress(deviceAddress);
  float temp = sensors->getTempC(deviceAddress);
  if(temp == -127)
  {
    return;
  }
  if(temp == -254)
  {
    return;
  }
  if(temp == -253)
  {
    return;
  }
  if(temp == -252)
  {
    return;
  }
  *temperature = (uint8_t)round(temp);
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