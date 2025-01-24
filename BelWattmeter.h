#include <Arduino.h>

class BelWattmeter
{
  private:
    uint8_t buffer[4];
    uint8_t bufferPosition = 0;
    int dataLength = 0;
    int dataRead = 0;
    int voltageTmp;
    int currentTmp;
    int consumptionTmp;
    int powerTmp;
    int* voltage;
    int* current;
    int* consumption;
    int* power;
    bool fdFirst = false;
    void Print(byte value);
  public: 
    BelWattmeter(int* voltage, int* current, int* consumption, int* power);
    void Loop();
};