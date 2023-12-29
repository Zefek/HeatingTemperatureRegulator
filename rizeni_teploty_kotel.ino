#include <avr/wdt.h>
#include <Ds1302.h>
#include "display.h"
#include "TemperatureSensor.h"

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
unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long currentMillis = 0;
unsigned long temperatureReadMillis = 0;
long interval = 0;
long position = 60000;
bool relayOn = false;
double value = 25;
double celsius = 25;
short direction = 0;
bool heatingOff = true;
double inputTemperature = 25;
double outsideTemperature = 14;

void setup() {
  Serial.begin(9600);
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
  outsideTemperatureSensor.Init();
  lcd.SetCurrentHeatingTemperature(celsius);
  lcd.SetInputTemperature(inputTemperature);
  lcd.SetOutTemperature(outsideTemperature);
  computeRequiredTemperature();
  /*int hour, minute, second;
  sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);
  Ds1302::DateTime dt;
  dt.hour = hour;
  dt.minute = minute;
  dt.second = second;
  dt.day = 29;
  dt.month = 12;
  dt.year = 2023;
  rtc.setDateTime(&dt);*/
  wdt_enable(WDTO_1S);
}

void OutsideTemperatureChanged(double temperature, int channel)
{
  if(temperature < -35 && temperature > 50)
  {
    return;
  }
  if(channel == 1)
  {
    outsideTemperature = temperature;
    lcd.SetOutTemperature(temperature);
    computeRequiredTemperature();
  }
}

double getTemperature(double pinValue, int beta)
{
  return 1 / (log(R0 / (1023 / pinValue - 1) / R) / beta + NORMTEMP) - 273.15;
}

void computeRequiredTemperature()
{
  //nastavená teplota v místnosti
  double inTemp = 22;
  //nastavená teplota topné vody pro venkovní teplotu 0°C
  //touto proměnnou se nastavuje sklon topné křivky.
  double zeroTemp = 46;
  Ds1302::DateTime now;
  rtc.getDateTime(&now);
  if(now.hour < 15 || now.hour >= 23)
  {
    inTemp = 21;
    zeroTemp = 39;
  }
  //double newValue = (outsideTemperature * -0.005093696 * -0.005093696) + (outsideTemperature * -0.966171371) + 43.4724957;
  //double newValue = (outsideTemperature * -0.005805978 * -0.005805978) + (outsideTemperature * -0.9839375) + 44.47618789;
  int newValue = (int)round(inTemp + (zeroTemp - inTemp) * pow((outsideTemperature - inTemp) / (double) - inTemp, 0.76923));
  
  //value = (outsideTemperature * EKVITERM_SLOPE) + (getRequiredInsideTemperature() * EKVITERM_KOEF);
  //value =  EKVITERM_KOEF * getRequiredInsideTemperature() + (1 - EKVITERM_KOEF) * outsideTemperature;
  if(newValue < 10 || newValue > 80)
  {
    return;
  }
  if(newValue != value)
  {
    value = newValue;
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
  }
  //value - celsis  - nastavená 50, naměřená 40 = 40-50=>10 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 50,68 = 50-50,68 = 0,68 - 0,5 = 0,18
  if(setTemperature - currentTemperature > 0 && position < 120000)
  {
    direction = 1;
    digitalWrite(MOREHEATINGRELAYPIN, LOW);
    relayOn = true;
  }
}

void setRelayOff()
{
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  relayOn = false;
}

void checkHeating()
{
  int tmpValue = value >= 30 ? 30 : value;
  if(!heatingOff && (outsideTemperature > 13.5 || inputTemperature < tmpValue - 2))
  {
    heatingOff = true;
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
    interval = 60000;
    direction = -1;
    relayOn = true;
    relayOnMillis = currentMillis;
  }
  if(heatingOff && outsideTemperature <= 13  && inputTemperature >= tmpValue + 2)
  {
    heatingOff = false;
    digitalWrite(HEATINGPUMPRELAYPIN, LOW);
  }
  lcd.SetHeating(!heatingOff);
}

void readInputTemperature()
{
  double newInputTemperature = getTemperature(analogRead(OUTTEMPPIN), 3950);
  if(newInputTemperature < 0 || newInputTemperature > 100)
  {
    return;
  }
  if(newInputTemperature < inputTemperature - 0.5 || newInputTemperature > inputTemperature + 0.5)
  {
    inputTemperature = newInputTemperature;
    lcd.SetInputTemperature((int)round(inputTemperature));
  }
  
}

void readCurrentHeatingTemperature()
{
  double newCelsius = getTemperature(analogRead(TEMPPIN), 3950);
  if(newCelsius < 0 || newCelsius > 100)
  {
    return;
  }
  if(newCelsius < celsius - 0.5 || newCelsius > celsius + 0.5)
  {
    if((int)round(celsius)!= (int)round(newCelsius))
    {
      Serial.print("Variable 1:");
      Serial.println(newCelsius);
    }
    celsius = newCelsius;
    lcd.SetCurrentHeatingTemperature((int)round(celsius));
  }
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  outsideTemperatureSensor.CheckTemperature();
  lcd.Print();
  if(currentMillis - temperatureReadMillis > 1500)
  {
    readCurrentHeatingTemperature();
    readInputTemperature();
    temperatureReadMillis = currentMillis;
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
    relayOffMillis = currentMillis;
  }
  if(!heatingOff && !relayOn && currentMillis - relayOffMillis > 20000)
  {
    interval = min(abs(value - celsius) * 1000, 60000);
    if(interval >= 1000)
    {
      setRelay(value, celsius);
      relayOnMillis = currentMillis;
    }
  }
}
