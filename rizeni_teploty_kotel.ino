#include <Ds1302.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "display.h"
#include "TemperatureSensor.h"
#include "HomeAssistant.h"
#include "config.h"

#define I2C_ADDR            0x27
#define LCD_COLUMNS         16
#define LCD_LINES           2
#define BETA                3950
#define TEMPPIN             A0
#define OUTTEMPPIN          A1
#define RETTEMPPIN          A2
#define NORMTEMP            1 / (25 + 273.15)
#define R0                  10000
#define R                   10000
#define MOREHEATINGRELAYPIN 8
#define LESSHEATINGRELAYPIN 9
#define HEATINGPUMPRELAYPIN 10
#define MININPUTTEMPERATURE 30

void OutsideTemperatureChanged(double temperature, int channel, int humidity, int sensorId, bool transmitedByButton, bool batteryLow, int temperatureTrend);
void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length);
void TimeChanged(int hours, int minutes);
void computeRequiredTemperature();


Display lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Ds1302 rtc(4, 5, 6);
TemperatureSensor outsideTemperatureSensor(2, OutsideTemperatureChanged);
HomeAssistant homeAssistant(WifiSSID, WifiPassword, MQTTUsername, MQTTPassword, MQTTHost, "Heating", MQTTMessageReceive);
unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long currentMillis = 0;
unsigned long temperatureReadMillis = 0;
unsigned long lastMQTTSendMillis = 0;
unsigned long lastMQTTConnectTry = 0;
long interval = 0;
long position = 70000;
bool relayOn = false;
double value = 25;
double celsius = 25;
double retCelsius = 25;
double celsiusAfterSet = 25;
short direction = 0;
bool heatingOff = true;
double inputTemperature = 25;
double outsideTemperature = 14;
double currentInsideTemperature = 22;
double requiredInsideTemperature = 22;
int sensor = 1;
bool manualValve = false;
bool autoHeating = true;
bool thermostat = false;
double totalPower = 0;
int sensrId = 0;
int mode = 1; //0 - Off, 1 - Automat, 2 - Thermostat
int setTemperatureMode = 0; //0 - Automat, 1 - Manual

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  byte msb = EEPROM.read(0);
  byte lsb = EEPROM.read(1);
  sensrId = (msb << 4) + lsb;
  Serial.print("Sensor Id: ");
  Serial.println(sensrId);
  homeAssistant.Init();
  rtc.init();
  lcd.Init(&rtc, TimeChanged);
  lcd.BackLight();
  pinMode(MOREHEATINGRELAYPIN, OUTPUT);
  pinMode(LESSHEATINGRELAYPIN, OUTPUT);
  pinMode(HEATINGPUMPRELAYPIN, OUTPUT);
  pinMode(2, INPUT);
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
  int s = homeAssistant.Connect();
  homeAssistant.Subscribe("homeassistant/devices/heater/command/*");
  homeAssistant.Subscribe("homeassistant/status");
  homeAssistant.Subscribe("homeassistant/devices/heater/events/*");
  delay(1000);
  outsideTemperatureSensor.Init();
  lcd.SetCurrentHeatingTemperature(celsius);
  lcd.SetInputTemperature(inputTemperature);
  lcd.SetOutTemperature(outsideTemperature);
  computeRequiredTemperature();
}

void TimeChanged(int hours, int minutes)
{
  computeRequiredTemperature();
}

void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length)
{
  char* p = new char[length+1];
  for(int i = 0; i<length; i++)
  {
    p[i] = (char)payload[i];
  }
  p[length] = '\0';
  //Termostat on/off
  if(strcmp(topic, "homeassistant/devices/heater/command/thermostat") == 0)
  {
    thermostat = !thermostat;
    homeAssistant.SetSensor(thermostat?"ON":"OFF", "homeassistant/devices/heater/thermostat");
  }
  //Nastavení módu - Off (vypnuto), Automatic (automatické), Thermostat (ovládání termostatem)
  if(strcmp(topic, "homeassistant/devices/heater/command/mode") == 0)
  {
    if(strcmp(p, "Off") == 0)
    {        
      mode = 0;
    }
    else if(strcmp(p, "Automatic") == 0)
    {
      mode = 1;
    }
    else if (strcmp(p, "Thermostat") == 0)
    {
      mode = 2;
    }
    homeAssistant.SetSensor(p, "homeassistant/devices/heater/mode");
    lcd.SetMode(mode);
  }
  //Mód topné křivky - Automatický, manuální
  if(strcmp(topic, "homeassistant/devices/heater/command/settemperaturemode") == 0)
  {
    if(strcmp(p, "Automatic") == 0)
    { 
      setTemperatureMode = 0;
    }
    if(strcmp(p, "Manual") == 0)
    {
        setTemperatureMode = 1;
    }
    homeAssistant.SetSensor(p, "homeassistant/devices/heater/settemperaturemode");
  }
  //Nastavená teplota topné vody
  if(strcmp(topic, "homeassistant/devices/heater/command/settemperature") == 0)
  {
    int t = 0;
    sscanf(p, "%d", &t);
    value = t;
    setTemperatureMode = 1;
    lcd.SetRequiredTemperature(value);
    homeAssistant.SetSensor(value, "homeassistant/devices/heater/setTemperature");
    homeAssistant.SetSensor("Manual", "homeassistant/devices/heater/settemperaturemode");
  }
  //Nastavení data a času
  if(strcmp(topic, "homeassistant/devices/heater/command/currentDateTime") == 0)
  {
    int day, month, year, hour, minute, second;
    sscanf(p, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);
    Ds1302::DateTime dt;
    dt.hour = hour;
    dt.minute = minute;
    dt.second = second;
    dt.day = day;
    dt.month = month;
    dt.year = year;
    rtc.setDateTime(&dt);
  }
}

void OutsideTemperatureChanged(double temperature, int channel, int humidity, int sensorId, bool transmitedByButton, bool batteryLow, int temperatureTrend)
{
  if(temperature < -35 && temperature > 50)
  {
    return;
  }
  
  if(transmitedByButton && channel == 1)
  {
    sensrId = sensorId;
    char lsb = (unsigned)sensorId & 0xF;
    char msb = (unsigned)sensorId >> 4;
    
    EEPROM.put(0, msb);
    EEPROM.put(1, lsb);
  }
  
  if(channel == 1 && sensrId == sensorId)
  {
    //if(outsideTemperature != temperature)
    {
      outsideTemperature = temperature;
      JsonDocument doc;
      doc["t"] = temperature;
      doc["h"] = humidity;
      doc["b"] = batteryLow;
      doc["tt"] = temperatureTrend;
      String output;
      serializeJson(doc, output);
      lcd.SetOutTemperature(temperature);

      homeAssistant.SetSensor(output.c_str(), "homeassistant/devices/heater/outsideTemperature");
      Serial.println(output);
      lcd.SetOutTemperature(temperature);
      computeRequiredTemperature();
    }
  }
}

double getTemperature(double pinValue, int beta)
{
  return 1 / (log((1023 / pinValue - 1)) / beta + NORMTEMP) - 273.15;
}

void computeRequiredTemperature()
{
  if(setTemperatureMode == 1)
  {
    return;
  }
  //nastavená teplota v místnosti
  double inTemp = 22.5;
  //nastavená teplota topné vody pro venkovní teplotu 0°C
  //touto proměnnou se nastavuje sklon topné křivky.
  double zeroTemp = 40;
  Ds1302::DateTime now;
  rtc.getDateTime(&now);
  if(now.hour < 15 || now.hour >= 23)
  {
    inTemp = 21.5;
    zeroTemp = 38;
  }
  int newValue = (int)round(inTemp + (zeroTemp - inTemp) * pow((outsideTemperature - inTemp) / (double) - inTemp, 0.76923));

  if(newValue < 10 || newValue > 80)
  {
    return;
  }
  if(newValue != value)
  {
    value = newValue;
    lcd.SetRequiredTemperature(value);
    homeAssistant.SetSensor(value, "homeassistant/devices/heater/setTemperature");
  }
}

void setRelay(int pDirection)
{
  //value - celsius - 1 - nastavená 50, naměřená 40 = 50-40-1 = 9 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 60 = 50-60 = -10 => value < 0, direction = -1
  if(pDirection == -1)
  {
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    homeAssistant.SetSensor("ON", "homeassistant/devices/heater/valvemovement");
    relayOn = true;
    relayOnMillis = currentMillis;
    direction = -1;
  }
  //value - celsis  - nastavená 50, naměřená 40 = 40-50=>10 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 50,68 = 50-50,68 = 0,68 - 0,5 = 0,18
  if(pDirection == 1)
  {
    digitalWrite(MOREHEATINGRELAYPIN, LOW);
    homeAssistant.SetSensor("ON", "homeassistant/devices/heater/valvemovement");
    relayOn = true;
    relayOnMillis = currentMillis;
    direction = 1;
  }
}

void setRelayOff()
{
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  relayOn = false;
  homeAssistant.SetSensor("OFF", "homeassistant/devices/heater/valvemovement");
}

bool ShouldBeHeatingOff()
{
  if(mode == 0)
  {
    return true;
  }
  if(mode == 1)
  {
    return outsideTemperature > 14.5 || inputTemperature < MININPUTTEMPERATURE - 1;
  }
  if(mode == 2)
  {
    return !thermostat || inputTemperature < MININPUTTEMPERATURE - 1;
  }
  return false;
}

bool ShouldBeHeatingOn()
{
  if(mode == 0)
  {
    return false;
  }
  if(mode == 1)
  {
    return inputTemperature > MININPUTTEMPERATURE + 1 && outsideTemperature <= 14;
  }
  if(mode == 2)
  {
    return thermostat && inputTemperature > MININPUTTEMPERATURE + 1;
  }
  return false;
}

void checkHeating()
{
  if(!heatingOff && ShouldBeHeatingOff())
  {
    heatingOff = true;
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
    interval = 70000;
    direction = -1;
    relayOn = true;
    relayOnMillis = currentMillis;
    homeAssistant.SetSensor("ON", "homeassistant/devices/heater/valvemovement");
    homeAssistant.SetSensor("OFF", "homeassistant/devices/heater/heater");
  }
  if(heatingOff && ShouldBeHeatingOn())
  {
    heatingOff = false;
    digitalWrite(HEATINGPUMPRELAYPIN, LOW);
    homeAssistant.SetSensor("ON", "homeassistant/devices/heater/heater");
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
  if((int)round(inputTemperature) != (int)round(newInputTemperature))
  { 
    lcd.SetInputTemperature((int)round(inputTemperature));
  }
  if(newInputTemperature < inputTemperature - 0.5 || newInputTemperature > inputTemperature + 0.5)
  {
    inputTemperature = newInputTemperature;
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
    celsius = newCelsius;
    lcd.SetCurrentHeatingTemperature((int)round(celsius));
  }
}

void readCurrentReturnTemperature()
{
  double newCelsius = getTemperature(analogRead(RETTEMPPIN), 3950);
  if(newCelsius < 0 || newCelsius > 100)
  {
    return;
  }
  if(newCelsius < retCelsius - 0.5 || newCelsius > retCelsius + 0.5)
  {
    retCelsius = newCelsius;
  }
}

void computeEnergy()
{
  //Wh
  totalPower += max(0, (celsius - retCelsius) * 4180 / 3600);
  int power = (int)max(0, (celsius - retCelsius) * 4180 / 3600);
  lcd.SetPower(power);
}

void sendToHomeAssistant()
{
  if(currentMillis - lastMQTTSendMillis > 15000)
  {
    switch(sensor)
    {
      case 1 : homeAssistant.SetSensor(inputTemperature, "homeassistant/devices/heater/inputTemperature"); break;
      case 2 : homeAssistant.SetSensor(celsius, "homeassistant/devices/heater/currentTemperature"); break;
      case 3 : homeAssistant.SetSensor(retCelsius, "homeassistant/devices/heater/returnTemperature"); break;
      case 4 : homeAssistant.SetSensor(totalPower, "homeassistant/devices/heater/totalPower"); totalPower = 0; break;
    }
    sensor = sensor == 4 ? 1 : sensor+1;
    lastMQTTSendMillis = currentMillis;
  }
}

void loop() {
  currentMillis = millis();
  if(!homeAssistant.Loop())
  {
    homeAssistant.Connect();
    homeAssistant.Subscribe("homeassistant/devices/heater/command/*");
    homeAssistant.Subscribe("homeassistant/status");
    homeAssistant.Subscribe("homeassistant/devices/heater/events/*");
  }
  
  outsideTemperatureSensor.CheckTemperature();
  lcd.Print();
  if(currentMillis - temperatureReadMillis > 1000)
  {
    readCurrentHeatingTemperature();
    readCurrentReturnTemperature();
    readInputTemperature();
    computeEnergy();
    temperatureReadMillis = currentMillis;
  }
  sendToHomeAssistant();
  if(!relayOn)
  {
    checkHeating();    
  }
  if(relayOn && currentMillis - relayOnMillis >= interval)
  {
    setRelayOff();
    position += interval * direction;
    position = min(max(position, 0), 70000);
    homeAssistant.SetSensor((int)round(position / (double)700), "homeassistant/devices/heater/valve");
    relayOffMillis = currentMillis;
    celsiusAfterSet = celsius;
  }
 
  if(!heatingOff && !relayOn && currentMillis - relayOffMillis > 10000 && !manualValve)
  {
    long intervalTmp = (value - celsius) * 700 - (celsius - celsiusAfterSet) * 1500;
    interval = min(abs(intervalTmp), 70000);
    if(interval >= 400)
    {

      if(intervalTmp <= 0)
      {
        setRelay(-1);
      }
      else if(value - celsius > 0)
      {
        setRelay(1);
      }
    }
  }
}