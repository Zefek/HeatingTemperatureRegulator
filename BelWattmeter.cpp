#include "BelWattmeter.h"

BelWattmeter::BelWattmeter(int* voltage, int* current, int* consumption, int* power)
{
  this->voltage = voltage;
  this->current = current;
  this->consumption = consumption;
  this->power = power;
}

void BelWattmeter::Loop()
{
  while(Serial2.available())
  {
    byte read = (byte)Serial2.read();
    buffer[bufferPosition++] = read;
    if(this->dataRead < this->dataLength)
    {
      if(read == 0xFD)
      {
        this->buffer[0] = this->buffer[1];
        this->buffer[1] = this->buffer[2];
        this->buffer[2] = this->buffer[3];
        this->bufferPosition = 3;
        if(!fdFirst)
        {          
          fdFirst = true;
          continue;
        }
        fdFirst = false;
      }
      if(this->dataRead == 2 || this-> dataRead == 3)
      {
        this->voltageTmp += dataRead == 2? read : read << 8;
      }
      if(this->dataRead == 4 || this->dataRead == 5)
      {
        this->currentTmp += this->dataRead == 4? read : read << 8;
      }
      if(this->dataRead == 6 || this->dataRead ==7)
      {
        this->consumptionTmp += this->dataRead == 6? read : read << 8;
      }
      if(this->dataRead == 19 || this->dataRead == 20)
      {
        this->powerTmp += this->dataRead == 19? read : read << 8;
      }
      if(this->dataRead == 26)
      {
        crcOk = crc == read;
      }
      crc += read;
      dataRead++;
    }
    if(this->buffer[0] == 0xFE && this->buffer[1] == 0xFE && this->buffer[2] == 0xFC && this->buffer[3] == 0x01 && this->dataLength == this->dataRead)
    {
      if(crcOk && voltageTmp < 360 && currentTmp < 1600 && powerTmp < 4000)
      {
        *voltage = ((unsigned long)*voltage * counter + voltageTmp) / (counter + 1);
        *current = ((unsigned long)*current * counter + currentTmp) / (counter + 1);
        *power = ((unsigned long)*power * counter + powerTmp) / (counter + 1);
        *consumption = consumptionTmp;
        Serial.println((unsigned long)*power * counter + powerTmp);
        counter++;
      }
      voltageTmp = 0;
      currentTmp = 0;
      powerTmp = 0;
      consumptionTmp = 0;
      crcOk = false;
      this->dataLength = Serial2.read() - 1;
      if(this->dataLength!=27)
      {
        this->dataLength = 0;
      }
      this->dataRead = 0;
      crc = 0x01 + this->dataLength + 1;
      fdFirst = false;
    }
    this->buffer[0] = this->buffer[1];
    this->buffer[1] = this->buffer[2];
    this->buffer[2] = this->buffer[3];
    this->bufferPosition = 3;
  }
}

void BelWattmeter::Reset()
{
  counter = 0;
}