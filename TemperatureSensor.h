#define RING_BUFFER_SIZE  256

#define SYNC_LENGTH  8000
#define SEP_LENGTH   500
#define BIT1_LENGTH  4000
#define BIT0_LENGTH  2000

class TemperatureSensor
{
  private:
    static unsigned long timings[RING_BUFFER_SIZE];
    static unsigned int syncIndex1;
    static unsigned int syncIndex2;
    static bool received;
    static int interruptPin;
    static unsigned long receivedMillis;
    static bool isSync(unsigned int idx);
    static void handler();
    static void (*TemperatureChanged)(double, int, int, int, bool, bool, int);
    static bool Read(byte *bytes);
    static unsigned long CheckCRC(byte *bytes, int);

  public:
    TemperatureSensor(int interruptPin, void (*temperatureChanged)(double, int, int, int, bool, bool, int));
    void Init();
    void CheckTemperature();
};