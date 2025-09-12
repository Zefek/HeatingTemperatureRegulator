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
    LiquidCrystal_I2C *lcd;//(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
    uint8_t requiredTemperature = 0;
    double outTemperature = 14;
    uint8_t inputTemperature = 0;
    uint8_t currentHeatingTemperature = 0;
    uint8_t hours = 0;
    uint8_t minutes = 0;
    Ds1302* rtc;
    unsigned long lastPrint;
    bool shouldBlink = false;
    bool heatingOn = false;
    uint8_t mode = 0;
    int wasteGasTemperature = 0;
    bool outsideTemperatureWasSet = false;
    bool thermostat = false;
    bool initializing = false;
    bool forcePrint = false;

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
};