#include <Ds1302.h>
#include <LiquidCrystal_I2C.h>

class Display
{
  private:
  byte onChar[8] = {
	  0b11111,
	  0b11111,
	  0b11111,
	  0b11111,
	  0b11111,
	  0b11111,
	  0b11111,
	  0b11111
  };
  byte celsiusChar[8] = {
	  0b01010,
	  0b10101,
	  0b01100,
	  0b00100,
	  0b00100,
	  0b00100,
	  0b00101,
	  0b00010
  };
  byte flame1[8] = { B00100, B01000, B01010, B10010, B10101, B10101, B10001, B01110 };
  byte flame2[8] = { B00100, B00010, B01010, B01001, B10101, B10101, B10001, B01110 };

  byte pump1[8] = { B00100, B00100, B00000, B11011, B00000, B00100, B00100, B00000 };
  byte pump2[8] = { B01010, B01010, B00100, B00000, B00100, B01010, B01010, B00000 };

    LiquidCrystal_I2C *lcd;//(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
    uint8_t requiredTemperature = 0;
    double outTemperature = 14;
    uint8_t inputTemperature = 0;
    uint8_t currentHeatingTemperature = 0;
    uint8_t hours = 0;
    uint8_t minutes = 0;
    Ds1302* rtc = nullptr;
    unsigned long lastPrint = 0;
    bool shouldBlink = false;
    bool heatingOn = false;
    uint8_t mode = 0;
    int wasteGasTemperature = 0;
    bool outsideTemperatureWasSet = false;
    bool thermostat = false;
    bool initializing = false;
    bool forcePrint = false;
    bool heaterOn = false;

    void print2digits(uint8_t number);
    void printTime();
    void printOutTemperature(bool blink);
    void Blink();
    void PrintMode(uint8_t mode, bool blink);

  public:
    Display(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);
    void Init(Ds1302* rtc);
    void SetRequiredTemperature(uint8_t requiredTempareture);
    void Print();
    void SetCurrentHeatingTemperature(uint8_t currentHeatingTemperature);
    void SetInputTemperature(uint8_t inputTemperature);
    void SetOutTemperature(double outTemperature);
    void SetHeating(bool heatingOn);
    void SetMode(uint8_t mode);
    void SetWasteGasTemperature(int wasteGasTemperature);
    void SetThermostat(bool thermostat);
    void EndInitialize();
    void SetHeater(bool heaterOn);
};