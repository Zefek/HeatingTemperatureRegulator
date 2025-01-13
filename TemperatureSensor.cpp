#include "Arduino.h" 
#include "TemperatureSensor.h"

unsigned int TemperatureSensor::syncIndex1 = 0;
unsigned int TemperatureSensor::syncIndex2 = 0;
bool TemperatureSensor::received = false;
int TemperatureSensor::interruptPin = 2;
unsigned long TemperatureSensor::receivedMillis = 0;
unsigned long TemperatureSensor::timings[RING_BUFFER_SIZE];
void (*TemperatureSensor::TemperatureChanged)(double, uint8_t, uint8_t, uint8_t*, bool);

bool TemperatureSensor::isSync(unsigned int idx)
{
  unsigned long t0 = timings[(idx+RING_BUFFER_SIZE-1) % RING_BUFFER_SIZE];
  unsigned long t1 = timings[idx];
  // on the temperature sensor, the sync signal
  // is roughtly 9.0ms. Accounting for error
  // it should be within 8.0ms and 10.0ms
  if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100) &&
    t1>(SYNC_LENGTH-1000) && t1<(SYNC_LENGTH+1000) &&
    digitalRead(interruptPin) == HIGH) {
    return true;
  }
  return false;
}

TemperatureSensor::TemperatureSensor(int interruptPin, void (*temperatureChanged)(double, uint8_t, uint8_t, uint8_t*, bool))
{
  interruptPin = interruptPin;
  received = false;
  syncIndex1 = 0;
  syncIndex2 = 0;
  receivedMillis = 0;
  TemperatureChanged = temperatureChanged;
}

void TemperatureSensor::Init()
{
  digitalWrite(3, HIGH);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
}

void TemperatureSensor::handler()
{
  static unsigned long duration = 0;
  static unsigned long lastTime = 0;
  static unsigned int ringIndex = 0;
  static unsigned int syncCount = 0;
  if (received == true || millis() - receivedMillis <= 1000) {
    return;
  }
  long time = micros();
  duration = time - lastTime;
  lastTime = time;
  // store data in ring buffer
  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  timings[ringIndex] = duration;
  
  // detect sync signal
  if (isSync(ringIndex)) {
    syncCount++;
    // first time sync is seen, record buffer index
    if (syncCount == 1) {
      syncIndex1 = (ringIndex+1) % RING_BUFFER_SIZE;
    } 
    else if (syncCount == 2) {
      // second time sync is seen, start bit conversion
      syncCount = 0;
      syncIndex2 = (ringIndex+1) % RING_BUFFER_SIZE;
      unsigned int changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2+RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
      // changeCount must be 86 -- 32 bits x 2 + 2 for sync
      if (changeCount != 92) {
        received = false;
        syncIndex1 = 0;
        syncIndex2 = 0;
      } 
      else {
        received = true;
      }
    }
  }
}

bool TemperatureSensor::Read(byte *bytes)
{
  unsigned long temp = 0;
  bool negative = false;
  bool fail = false;
  byte b = 0;
  int j = 0;
  int p = 0;
  for(unsigned int i=(syncIndex1+0)%RING_BUFFER_SIZE; i!=(syncIndex1+80)%RING_BUFFER_SIZE; i=(i+2)%RING_BUFFER_SIZE) 
  {
    unsigned long t0 = TemperatureSensor::timings[i], t1 = TemperatureSensor::timings[(i+1)%RING_BUFFER_SIZE];
    if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100)) 
    {
      if (t1>(BIT1_LENGTH-1000) && t1<(BIT1_LENGTH+1000)) 
      {
        b = (b << 1) + 1;
        j++;
      } 
      else if (t1>(BIT0_LENGTH-1000) && t1<(BIT0_LENGTH+1000)) 
      {
        b = (b << 1) + 0;
        j++;
      } 
      else
      {
        fail = true;
        break;
      }
    } 
    else 
    {
      fail = true;
      break;
    }
    if(j == 4)
    {
      bytes[p++] = b;
      b = 0;
      j = 0;
    }
  }
  return fail;
}

unsigned long TemperatureSensor::CheckCRC(byte *bytes, int crc)
{
  int rem = 0;
  for(int i = 0; i<9; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      if(rem & 0x08)
      {
        rem = (rem << 1) ^ 3;
      }
      else
      {
        rem <<= 1;
      }
    }
    rem ^= bytes[i];
  }
  return (rem & 0x0F) == crc;
}

void TemperatureSensor::CheckTemperature()
{
  if (received == true) {
    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(digitalPinToInterrupt(TemperatureSensor::interruptPin));
    // loop over the lowest 12 bits of the middle 2 bytes
    byte bytes[10];
    bool fail = TemperatureSensor::Read(bytes);
    if (!fail) 
    {
      byte toCheckCRC[10];
      for(int i = 0; i<10; i++)
      {
        toCheckCRC[i] = bytes[i];
        Serial.print(bytes[i]);
        Serial.print("|");
      }
      Serial.println("");
      toCheckCRC[2] = toCheckCRC[9];
      bool checkedCRC = TemperatureSensor::CheckCRC(toCheckCRC, bytes[2]);
      if(checkedCRC)
      {
        unsigned long sensorId = (bytes[0] << 4) + bytes[1];
        bool transmitedByButton = (bytes[3] & 0x08) != 0;
        double temperature = ((((bytes[4] << 8) + (bytes[5] << 4) + bytes[6]) * 0.1)-90-32) * ((double)5/9);
        int humidity = (bytes[7] * 10) + bytes[8];
        int channel = bytes[9];
        uint8_t rawData[5];
        rawData[0] = (bytes[0] << 4) + bytes[1];
        rawData[1] = (bytes[2] << 4) + bytes[3];
        rawData[2] = (bytes[4] << 4) + bytes[5];
        rawData[3] = (bytes[6] << 4) + bytes[7];
        rawData[4] = (bytes[8] << 4) + bytes[9];
        TemperatureChanged(temperature, channel, sensorId, rawData, transmitedByButton);
      }
      else
      {
        Serial.println("CRC Failed");
      }
      receivedMillis = millis();
    }
    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
  }
}