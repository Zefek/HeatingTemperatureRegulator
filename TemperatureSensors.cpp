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
  this->sensors->setResolution(currentHeating, 12);
}

void TemperatureSensors::RequestTemperatures()
{
  this->sensors->requestTemperatures();
}

bool TemperatureSensors::GetAcumulator1Temperature(uint8_t* temperature)
{
  return GetTemperature(acumulatorFirst, temperature);
}

bool TemperatureSensors::GetAcumulator2Temperature(uint8_t* temperature)
{
  return GetTemperature(acumulatorSecond, temperature);
}

bool TemperatureSensors::GetAcumulator3Temperature(uint8_t* temperature)
{
  return GetTemperature(acumulatorThird, temperature);
}

bool TemperatureSensors::GetAcumulator4Temperature(uint8_t* temperature)
{
  return GetTemperature(acumulatorFour, temperature);
}

bool TemperatureSensors::GetAcumulatorOutputTemperature(uint8_t* temperature)
{
  return GetTemperature(acumulatorOutput, temperature);
}

bool TemperatureSensors::GetReturnHeatingTemperature(uint8_t* temperature)
{
  return GetTemperature(returnHeating, temperature);
}

bool TemperatureSensors::GetHeaterTemperature(uint8_t* temperature)
{
  return GetTemperature(heaterTemperature, temperature);
}

bool TemperatureSensors::GetBoilerTemperature(uint8_t* temperature)
{
  return GetTemperature(boiler, temperature);
}

bool TemperatureSensors::GetCurrentHeatingTemperature(uint8_t* temperature)
{
  return GetTemperature(currentHeating, temperature);
}

bool TemperatureSensors::GetTemperature(DeviceAddress deviceAddress, uint8_t* temperature)
{
  float temp = sensors->getTempC(deviceAddress);
  if(temp <= -127.0f || temp < 0 || temp > 120)
  {
    return false;
  }
  *temperature = (uint8_t)round(temp);
  return true;
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