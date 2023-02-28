#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <virtuabotixRTC.h>

#define I2C_ADDR    0x27
#define LCD_COLUMNS 16
#define LCD_LINES   2
#define BETA  3950
#define POTPIN  0
#define TEMPPIN  1
#define OUTTEMPPIN  2
#define NORMTEMP 1 / (25 + 273.15)
#define R0 9800
#define R 10000
#define EKVITERM_KOEF 2.34
#define EKVITERM_SLOPE -0.7

LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
virtuabotixRTC myRtc(2, 3, 4);
unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long lastReadCelsius = 0;
unsigned long lastReadOutTemperature = 0;
unsigned long lastSetOutsideTemperature = 0;
long interval = 0;
long position = 0;
bool relayOn = false;
double value = 0;
double celsius = 0;
int direction = 0;
bool heatingOff = false;
int inputTemperature = 100;
double outsideTemperature = 99;
double tempOutsideTemperature = 0;
int countOutsideTemperature = 0;

double getTemperature(double pinValue, int beta)
{
  return 1 / (log(R0 / (1023 / pinValue - 1) / R) / beta + NORMTEMP) - 273.15;
}

void setup() {
  analogReference(EXTERNAL);
  lcd.init();
  lcd.backlight();
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  //relayOffMillis = millis() - 60000;
  wdt_enable(WDTO_1S);
  position = 60000;
  //myRtc.setDS1302Time(20, 21, 14, 1, 27, 2, 2023);
}

int getRequiredInsideTemperature()
{
  int hour = myRtc.hours;
  if(hour >= 8 && hour <23)
    return 22;
  return 19;
}

void computeRequiredTemperature()
{
  value = (outsideTemperature * EKVITERM_SLOPE) + (getRequiredInsideTemperature() * EKVITERM_KOEF);
  //value =  EKVITERM_KOEF * getRequiredInsideTemperature() + (1 - EKVITERM_KOEF) * outsideTemperature;
  lcd.setCursor(0, 0);
  lcd.print(value);
}

void setRelay(long setTemperature, double currentTemperature)
{
  //value - celsius - 1 - nastavená 50, naměřená 40 = 50-40-1 = 9 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 60 = 50-60 = -10 => value < 0, direction = -1
  direction = 0;
  if(setTemperature - currentTemperature + 1 < 0 && position > 0)
  {
    direction = -1;
    digitalWrite(8, HIGH);
    relayOn = true;
  }
  //value - celsis  - nastavená 50, naměřená 40 = 40-50=>10 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 50,68 = 50-50,68 = 0,68 - 0,5 = 0,18
  if(setTemperature - currentTemperature > 0 && position < 120000)
  {
    direction = 1;
    digitalWrite(9, HIGH);
    relayOn = true;
  }
}

void setRelayOff()
{
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  relayOn = false;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print("0");
  }
  lcd.print(number);
}

void printTime()
{
  myRtc.updateTime();
  lcd.setCursor(11, 0);
  print2digits(myRtc.hours);
  lcd.print(":");
  print2digits(myRtc.minutes);
}

void printOutTemperature(double outTemperature)
{
  //celkem 6 míst -10.00
  lcd.setCursor(10, 1);
  if(outTemperature >= 0 && outTemperature < 10)
  {
    //8.98
    lcd.print("  ");
  }
  if((outTemperature > -10 && outTemperature < 0) || outTemperature >= 10)
  {
    //-8.89
    lcd.print(" ");
  }
  lcd.print(outTemperature);
}

void checkHeating(unsigned long currentMillis)
{
  if(!heatingOff && ((value >= 40 && inputTemperature < 39) || (value < 40 && inputTemperature < value - 2)))
  {
    heatingOff = true;
    interval = 60000;
    direction = -1;
    digitalWrite(8, HIGH);
    relayOn = true;
    relayOnMillis = currentMillis;
  }
  if(heatingOff && ((value >= 40 && inputTemperature > 40) || (value < 40 && inputTemperature > value + 2)))
  {
    heatingOff = false;
  }
}

void computeOutsideTemperature(unsigned long currentMillis)
{
  double currentTemperature = getTemperature(analogRead(3), 3435);
  tempOutsideTemperature = (tempOutsideTemperature * countOutsideTemperature + currentTemperature) / (countOutsideTemperature + 1);
  countOutsideTemperature++;

  if(currentMillis - lastSetOutsideTemperature > 60000 || outsideTemperature >= 99)
  {
    outsideTemperature = tempOutsideTemperature;
    printOutTemperature(outsideTemperature);
    countOutsideTemperature = 1;
    lastSetOutsideTemperature = currentMillis;    
  }  
}

void computeInputTemperature()
{
  inputTemperature = getTemperature(analogRead(2), 3950);
  lcd.setCursor(7, 0);
  lcd.print(inputTemperature);
}

void readCurrentHeatingTemperature()
{
  celsius = getTemperature(analogRead(TEMPPIN), 3950);
  lcd.setCursor(0, 1);
  lcd.print(celsius);
}

void loop() {
  wdt_reset();
  printTime();  
  unsigned long currentMillis = millis();
  computeOutsideTemperature(currentMillis);
  if(currentMillis - lastReadCelsius > 1000)
  {
    readCurrentHeatingTemperature();
    computeRequiredTemperature();
    computeInputTemperature();
    lastReadCelsius = currentMillis;
  }
  if(!relayOn)
  {
    checkHeating(currentMillis);    
  }
  if(relayOn && currentMillis - relayOnMillis > interval)
  {
    setRelayOff();
    position += interval * direction;
    position = min(max(position, 0), 120000);
    relayOffMillis = currentMillis;
  }
  if(!heatingOff && !relayOn && currentMillis - relayOffMillis > 45000)
  {
    interval = min(abs(value-celsius) * 1875, 60000);
    if(interval >= 1000)
    {
      setRelay(value, celsius);
      relayOnMillis = currentMillis;
    }
  }
}
