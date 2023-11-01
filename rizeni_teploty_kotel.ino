#include <avr/wdt.h>
#include <Ds1302.h>
#include "display.h"
#include "TemperatureSensor.h"
#include "TemperatureList.h"

#define I2C_ADDR    0x27
#define LCD_COLUMNS 16
#define LCD_LINES   2
#define BETA  3950
#define TEMPPIN  A0
#define OUTTEMPPIN  A1
#define NORMTEMP 1 / (25 + 273.15)
#define R0 9800
#define R 10000
#define MOREHEATINGRELAYPIN 8
#define LESSHEATINGRELAYPIN 9
#define HEATINGPUMPRELAYPIN 10

void OutsideTemperatureChanged(double temperature, int channel);

Display lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Ds1302 rtc(5, 3, 4);
TemperatureSensor outsideTemperatureSensor(2, OutsideTemperatureChanged);
TemperatureList tempList;
unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long lastReadCelsius = 0;
unsigned long lastReadOutTemperature = 0;
unsigned long currentMillis = 0;
unsigned long averageCelsiusMillis = 0;
long interval = 0;
long position = 0;
bool relayOn = false;
double value = 25;
double celsius = 25;
double averageCelsius = 0;
int averageCelsiusCount = 0;
int direction = 0;
bool heatingOff = true;
double inputTemperature = 25;
double outsideTemperature = 25;

void setup() {
  Serial.begin(9600);
  Serial.println("Started.");
  analogReference(EXTERNAL);
  rtc.init();
  lcd.Init(&rtc);
  lcd.BackLight();
  pinMode(MOREHEATINGRELAYPIN, OUTPUT);
  pinMode(LESSHEATINGRELAYPIN, OUTPUT);
  pinMode(HEATINGPUMPRELAYPIN, OUTPUT);
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
  wdt_enable(WDTO_1S);
  position = 60000;
  outsideTemperatureSensor.Init();
  lcd.SetCurrentHeatingTemperature(celsius);
  lcd.SetInputTemperature((int)inputTemperature);
  lcd.SetOutTemperature(outsideTemperature);
  /*int hour, minute, second;
  sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);
  Ds1302::DateTime dt;
  dt.hour = hour;
  dt.minute = minute;
  dt.second = second;
  dt.day = 28;
  dt.month = 10;
  dt.year = 2023;
  rtc.setDateTime(&dt);*/
}

void OutsideTemperatureChanged(double temperature, int channel)
{
  lastReadOutTemperature = currentMillis;
  if(channel == 1)
  {
    Ds1302::DateTime now;
    rtc.getDateTime(&now);
    tempList.Add(now.hour, now.minute, temperature);
    outsideTemperature = tempList.GetAverageTemperature();
    lcd.SetOutTemperature(outsideTemperature);
  }
}

double getTemperature(double pinValue, int beta)
{
  return 1 / (log(R0 / (1023 / pinValue - 1) / R) / beta + NORMTEMP) - 273.15;
}

void computeRequiredTemperature()
{
  double newValue = (outsideTemperature * -0.005093696 * -0.005093696) + (outsideTemperature * -0.966171371) + 43.4724957;
  Ds1302::DateTime now;
  rtc.getDateTime(&now);
  if(now.hour < 8 || now.hour >= 23)
  {
    newValue *= 0.87;
  }
  //value = (outsideTemperature * EKVITERM_SLOPE) + (getRequiredInsideTemperature() * EKVITERM_KOEF);
  //value =  EKVITERM_KOEF * getRequiredInsideTemperature() + (1 - EKVITERM_KOEF) * outsideTemperature;
  if(newValue != value)
  {
    value = newValue;
    //Serial.print("Required temperature : ");
    //Serial.println(value);
    lcd.SetRequiredTemperature(value);
  }
}

void setRelay(long setTemperature, double currentTemperature)
{
  //value - celsius - 1 - nastavená 50, naměřená 40 = 50-40-1 = 9 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 60 = 50-60 = -10 => value < 0, direction = -1
  direction = 0;
  if(setTemperature - currentTemperature + 1 < 0 && position > 0)
  {
    direction = -1;
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    relayOn = true;
    //Serial.print("Less heating relay on, interval: ");
    //Serial.println(interval);
  }
  //value - celsis  - nastavená 50, naměřená 40 = 40-50=>10 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 50,68 = 50-50,68 = 0,68 - 0,5 = 0,18
  if(setTemperature - currentTemperature > 0 && position < 120000)
  {
    direction = 1;
    digitalWrite(MOREHEATINGRELAYPIN, LOW);
    relayOn = true;
    //Serial.print("More heating relay on, interval: ");
    //Serial.println(interval);
  }
}

void setRelayOff()
{
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  relayOn = false;
  //Serial.println("Relay off");
}

void checkHeating()
{
  if((!heatingOff && outsideTemperature >= 13) || inputTemperature < value - 2)
  {
    heatingOff = true;
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
    interval = 60000;
    direction = -1;
    relayOn = true;
    relayOnMillis = currentMillis;
    //Serial.println("Heating off");
  }
  if(heatingOff && outsideTemperature < 13  && inputTemperature >= value + 2)
  {
    heatingOff = false;
    digitalWrite(HEATINGPUMPRELAYPIN, LOW);
    //Serial.println("Heating on");
  }
  lcd.SetHeating(!heatingOff);
}

void computeInputTemperature()
{
  double newInputTemperature = getTemperature(analogRead(OUTTEMPPIN), 3950);
  if(newInputTemperature < 0)
  {
    //Serial.println("Error read input temperature");
    return;
  }
  if(newInputTemperature != inputTemperature)
  {
    inputTemperature = newInputTemperature;
    lcd.SetInputTemperature((int)inputTemperature);
  }
  
}

void readCurrentHeatingTemperature()
{
  double newCelsius = getTemperature(analogRead(TEMPPIN), 3950);
  if(newCelsius < 0)
  {
    //Serial.println("Error read current heating temperature");
    return;
  }
  averageCelsius = (averageCelsius * averageCelsiusCount + newCelsius) / ++averageCelsiusCount;
  if(averageCelsius != celsius && currentMillis - averageCelsiusMillis >= 1875)
  {
    celsius = averageCelsius;
    lcd.SetCurrentHeatingTemperature(celsius);
    averageCelsiusMillis = currentMillis;
    averageCelsiusCount = 1;
  }
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  outsideTemperatureSensor.CheckTemperature();
  lcd.Print();
  if(currentMillis - lastReadCelsius > 1000)
  {
    readCurrentHeatingTemperature();
    computeRequiredTemperature();
    computeInputTemperature();
    lastReadCelsius = currentMillis;
  }
  if(!relayOn)
  {
    checkHeating();    
  }
  if(relayOn && currentMillis - relayOnMillis > interval)
  {
    setRelayOff();
    position += interval * direction;
    position = min(max(position, 0), 120000);
    //Serial.print(F("Position: "));
    //Serial.println(position);
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
  if(currentMillis - lastReadOutTemperature > 1000*60*5)
  {
    //Serial.println("Error read outside temperature");
  }
}
