#include <Arduino.h>

struct BelData
{
  int voltage = 0;
  int current = 0;
  int power = 0;
  int consumption = 0;
};

class BelWattmeter
{
  private:
    uint8_t buffer[4];
    uint8_t bufferPosition = 0;
    int dataLength = 0;
    int dataRead = 0;
    unsigned int voltageTmp;
    unsigned int currentTmp;
    unsigned int consumptionTmp;
    unsigned int powerTmp;
    unsigned int* voltage;
    unsigned int* current;
    unsigned int* consumption;
    unsigned int* power;
    bool fdFirst = false;
    byte crc = 0;
    bool crcOk = false;
    int counter = 0;
    BelData data;
  public: 
    BelData GetBelData();
    void Loop();
    void Reset();
};