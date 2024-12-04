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
    double requiredTemperature = 0;
    double outTemperature = 0;
    int inputTemperature = 0;
    int currentHeatingTemperature = 0;
    int hours = 0;
    int minutes = 0;
    Ds1302* rtc;
    unsigned long lastPrint;
    unsigned long blinkCount = 0;
    bool heatingOn = false;

    void print2digits(int number);
    void printTime();
    void printOutTemperature();
    void Blink();

  public:
    Display(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);
    void Init(Ds1302* rtc, void (*timeChanged)(int, int));
    void BackLight();
    void SetRequiredTemperature(int requiredTempareture);
    void Print();
    void SetCurrentHeatingTemperature(int currentHeatingTemperature);
    void SetInputTemperature(int inputTemperature);
    void SetOutTemperature(double outTemperature);
    void SetHeating(bool heatingOn);
    void SetPower(int power);
    void (*TimeChanged)(int, int);
};