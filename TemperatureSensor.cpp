#include "Arduino.h" 
#include "TemperatureSensor.h"

unsigned int TemperatureSensor::syncIndex1 = 0;
unsigned int TemperatureSensor::syncIndex2 = 0;
bool TemperatureSensor::received = false;
int TemperatureSensor::interruptPin = 2;
unsigned long TemperatureSensor::receivedMillis = 0;
unsigned long TemperatureSensor::timings[RING_BUFFER_SIZE];
void (*TemperatureSensor::TemperatureChanged)(double, int);

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

TemperatureSensor::TemperatureSensor(int interruptPin, void (*temperatureChanged)(double, int))
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
  digitalWrite(11, HIGH);
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

bool TemperatureSensor::Read(int from, int to, unsigned long* value)
{
  unsigned long temp = 0;
  bool negative = false;
  bool fail = false;
  for(unsigned int i=(syncIndex1+from)%RING_BUFFER_SIZE; i!=(syncIndex1+to)%RING_BUFFER_SIZE; i=(i+2)%RING_BUFFER_SIZE) {
    unsigned long t0 = TemperatureSensor::timings[i], t1 = TemperatureSensor::timings[(i+1)%RING_BUFFER_SIZE];
    if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100)) {
      if (t1>(BIT1_LENGTH-1000) && t1<(BIT1_LENGTH+1000)) {
        temp = (temp << 1) + 1;
      } 
      else if (t1>(BIT0_LENGTH-1000) && t1<(BIT0_LENGTH+1000)) {
        temp = (temp << 1) + 0;
      } 
      else {
        fail = true;
      }
    } 
    else {
      fail = true;
    }
  }
  if(!fail)
  {
    *value = temp;
  }
  return fail;
}

void TemperatureSensor::CheckTemperature()
{
  if (received == true) {
    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(digitalPinToInterrupt(TemperatureSensor::interruptPin));
    // loop over the lowest 12 bits of the middle 2 bytes
    bool fail = false;
    unsigned long temp = 0;
    unsigned long channel = 1;
    fail = TemperatureSensor::Read(32, 56, &temp);

    if (!fail) 
    {
      TemperatureSensor::Read(72, 80, &channel);
      TemperatureChanged(((((double)temp*0.1)-90)-32)*((double)5/9), (int)channel);
      receivedMillis = millis();
    }
    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handler, CHANGE);
  }
}