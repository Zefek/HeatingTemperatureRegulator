#include "BelWattmeter.h"

BelWattmeter::BelWattmeter(int* voltage, int* current, int* consumption, int* power)
{
  this->voltage = voltage;
  this->current = current;
  this->consumption = consumption;
  this->power = power;
}

void BelWattmeter::Print(byte value)
{
  if(value < 16)
  {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}

void BelWattmeter::Loop()
{
  while(Serial2.available())
  {
    byte read = (byte)Serial2.read();
    //Print(read);
    buffer[bufferPosition++] = read;
    if(this->dataRead < this->dataLength)
    {
      if(read == 0xFD)
      {
        if(!fdFirst)
        {
          this->buffer[0] = this->buffer[1];
          this->buffer[1] = this->buffer[2];
          this->buffer[2] = this->buffer[3];
          this->bufferPosition = 3;
          fdFirst = true;
          continue;
        }
        fdFirst = false;
      }
      else
      {
        if(this->dataRead == 2 || this-> dataRead == 3)
        {
          this->voltageTmp += dataRead == 2? read : read << 8;
          if(this->dataRead == 3)
          {
            if(*this->voltage < this->voltageTmp)
            {
              *this->voltage = this->voltageTmp;
            }
            this->voltageTmp = 0;
          }
        }
        if(this->dataRead == 4 || this->dataRead == 5)
        {
          this->currentTmp += this->dataRead == 4? read : read << 8;
          if(this->dataRead == 5)
          {
            if(*this->current < this->currentTmp)
            {
              *this->current = this->currentTmp;
            }
            this->currentTmp = 0;
          }
        }
        if(this->dataRead == 6 || this->dataRead ==7)
        {
          this->consumptionTmp += this->dataRead == 6? read : read << 8;
          if(this->dataRead == 7)
          {
            *this->consumption = this->consumptionTmp;
            this->consumptionTmp = 0;
          }
        }
        if(this->dataRead == 19 || this->dataRead == 20)
        {
          this->powerTmp += this->dataRead == 19? read : read << 8;
          if(this->dataRead == 20)
          {
            if(*this->power < this->powerTmp)
            {
              *this->power = this->powerTmp;
            }
            this->powerTmp = 0;
          }
        }
      }
      dataRead++;
    }
    if(this->buffer[0] == 0xFE && this->buffer[1] == 0xFE && this->buffer[2] == 0xFC && this->buffer[3] == 0x01 && this->dataLength == this->dataRead)
    {
      this->dataLength = Serial2.read() - 1;
      if(this->dataLength!=27)
      {
        this->dataLength = 0;
      }
      this->dataRead = 0;
      fdFirst = false;
    }
    this->buffer[0] = this->buffer[1];
    this->buffer[1] = this->buffer[2];
    this->buffer[2] = this->buffer[3];
    this->bufferPosition = 3;
  }
}