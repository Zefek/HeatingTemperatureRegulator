#include "BelWattmeter.h"


void BelWattmeter::ProcessByte(int dataIndex, uint8_t data)
{
  if(dataIndex == 1) {  Serial.print("BEL DataLength: "); Serial.println(data); this->dataLength = data; }
  if(dataIndex == 4)  this->voltageTmp = data;
  if(dataIndex == 5)  this->voltageTmp |= data << 8;
  if(dataIndex == 6)  this->currentTmp = data;
  if(dataIndex == 7)  this->currentTmp |= data << 8;
  if(dataIndex == 8)  this->consumptionTmp = data;
  if(dataIndex == 9)  this->consumptionTmp |= data << 8;
  if(dataIndex == 21) this->powerTmp = data;
  if(dataIndex == 22) this->powerTmp |= data << 8;
  if(dataIndex == 28)
  {
    crcOk = crc == data;
    return;
  }
  crc += data;
}

void BelWattmeter::Loop()
{
  while(Serial2.available())
  {
    int raw = Serial2.read();
    if(raw < 0)
    {
      continue;
    }
    byte read = (byte)raw;
    switch(state)
    {
      case WAIT_FOR_START:
        if(read == 0xFC)
        {
          Serial.println("INTO FRAME");
          state = IN_FRAME;
          crc = 0;
          feCount = 0;
          dataIndex = 0;
          dataLength = 0;
          this->voltageTmp = 0;
          this->currentTmp = 0;
          this->consumptionTmp = 0;
          this->powerTmp = 0;
          crcOk = false;
        }
      break;
      case IN_FRAME:
        if(read == 0xFD)
        {
          state = ESCAPE_NEXT;
        }
        else if (read == 0xFE)
        {
          feCount++;
          if(feCount == 2)
          {
            Serial.print("BEL: ");
            Serial.print(voltageTmp);
            Serial.print("V | ");
            Serial.print(currentTmp);
            Serial.print("A | ");
            Serial.print(powerTmp);
            Serial.print("W | ");
            Serial.print(consumptionTmp);
            Serial.print("Wh | ");
            Serial.println(crcOk);
            if(crcOk && voltageTmp < 360 && currentTmp < 1600 && powerTmp < 4000)
            {
              data.voltage = ((unsigned long)data.voltage * counter + voltageTmp) / (counter + 1);
              data.current = ((unsigned long)data.current * counter + currentTmp) / (counter + 1);
              data.power = ((unsigned long)data.power * counter + powerTmp) / (counter + 1);
              data.consumption = consumptionTmp;
              counter++;
            }
            state = WAIT_FOR_START;
          }
        }
        else
        {
          if(dataIndex > 1 && (dataIndex > this->dataLength + 1 || this->dataLength != 28))
          {
            state = WAIT_FOR_START;
            break;
          }
          ProcessByte(dataIndex, read);
          feCount = 0;
          dataIndex++;
        }
      break;
      case ESCAPE_NEXT:
        ProcessByte(dataIndex, read);
        state = IN_FRAME;
        feCount = 0;
        dataIndex++;
      break;
    }
  }
}

BelData BelWattmeter::GetBelData()
{
  return data;
}

void BelWattmeter::Reset()
{
  data.voltage = 0;
  data.current = 0;
  data.power = 0;
  data.consumption = 0;
  counter = 0;
}