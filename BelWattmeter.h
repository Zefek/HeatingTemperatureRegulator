#include <Arduino.h>

struct BelData
{
  int voltage = 0;
  int current = 0;
  int power = 0;
  int consumption = 0;
};

enum ParseState { WAIT_FOR_START, IN_FRAME, ESCAPE_NEXT };

class BelWattmeter
{
  private:
    ParseState state = WAIT_FOR_START;
    int dataLength = 0;
    int dataIndex = 0;
    unsigned int voltageTmp;
    unsigned int currentTmp;
    unsigned int consumptionTmp;
    unsigned int powerTmp;
    byte crc = 0;
    bool crcOk = false;
    int counter = 0;
    BelData data;
    uint8_t feCount = 0;

    void ProcessByte(int dataIndex, uint8_t data);
    
  public: 
    BelData GetBelData();
    void Loop();
    void Reset();
};