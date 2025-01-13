#include <Ds1302.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "display.h"
#include "TemperatureSensor.h"
#include "HomeAssistant.h"
#include "config.h"
#include <avr/wdt.h>

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

void OutsideTemperatureChanged(double temperature, uint8_t channel, uint8_t sensorId, uint8_t* rawData, bool transmitedByButton);
void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length);
void TimeChanged(int hours, int minutes);
void computeRequiredTemperature();
void OnMQTTConnected(bool success);

void convert_to_utf8(const uint8_t* input, uint8_t length, char* output) {
    unsigned char *out_ptr = output;
    int j = 0;
    for(int i = 0; i < length; i++)
    {
      uint8_t first = input[i] >> 4;
      uint8_t second = input[i] & 0x0F;
      output[j++] = first<10? (char)('0'+first):(char)('7'+first);
      output[j++] = second<10? (char)('0'+second):(char)('7'+second);   
    }

    output[j] = '\0'; // Null-terminate the UTF-8 string
}

/*
1 - currentTemperature
2 - inputTemperature
3 - returnTempareture
4 - setTemperature
5 - valve
6 - heater
7 - acumulator temperature 1
8 - acumulator temperature 2
9 - acumulator temperature 3
10 - acumulator temperature 4
*/
uint8_t states[10];

Display lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Ds1302 rtc(4, 5, 6);
TemperatureSensor outsideTemperatureSensor(2, OutsideTemperatureChanged);
HomeAssistant homeAssistant(WifiSSID, WifiPassword, MQTTUsername, MQTTPassword, MQTTHost, "Heating", MQTTMessageReceive, OnMQTTConnected);
unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long currentMillis = 0;
unsigned long temperatureReadMillis = 0;
unsigned long lastMQTTSendMillis = 0;
long interval = 0;
long position = 70000;
bool relayOn = false;
double value = 25;
double celsius = 25;
double celsiusAfterSet = 25;
short direction = 0;
bool heatingOff = true;
double outsideTemperature = 14;
int sensor = 1;
bool manualValve = false;
bool autoHeating = true;
bool thermostat = false;
double totalPower = 0;
uint8_t sensrId = 0;
uint8_t mode = 1; //0 - Off, 1 - Automat, 2 - Thermostat
uint8_t equithermalCurveZeroPoint = 41;
double insideTemperature = 23;
unsigned char utf8Buffer[32];

void setup() {
  Serial.begin(57600);
  //AT+UART_DEF=57600,8,1,0,0
  Serial1.begin(57600);
  sensrId = EEPROM.read(0);
  Serial.print("Sensor Id: ");
  Serial.println(sensrId);
  rtc.init();
  lcd.Init(&rtc, TimeChanged);
  lcd.BackLight();
  outsideTemperatureSensor.Init();
  homeAssistant.Init();
  homeAssistant.Connect();
  delay(1000);
  pinMode(MOREHEATINGRELAYPIN, OUTPUT);
  pinMode(LESSHEATINGRELAYPIN, OUTPUT);
  pinMode(HEATINGPUMPRELAYPIN, OUTPUT);
  pinMode(2, INPUT);
  digitalWrite(MOREHEATINGRELAYPIN, HIGH);
  digitalWrite(LESSHEATINGRELAYPIN, HIGH);
  digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
  lcd.SetMode(mode);
  readCurrentHeatingTemperature();
  readCurrentReturnTemperature();
  readInputTemperature();
  lcd.Print();
  //wdt_enable(WDTO_8S);
}

void OnMQTTConnected(bool success)
{
  if(success)
  {
    //homeAssistant.Subscribe("homeassistant/devices/heater/command/*");
    homeAssistant.Subscribe("cmd/thermostat");
    homeAssistant.Subscribe("cmd/mode");
    homeAssistant.Subscribe("cmd/zeroPoint");
    homeAssistant.Subscribe("cmd/currentDateTime");
    homeAssistant.Subscribe("events/insidetemperaturesetchanged");
  }
  if(!success)
  {
    mode = 1;
    lcd.SetMode(mode);
    equithermalCurveZeroPoint = 41;
    insideTemperature = 23;
  }
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
  if(strcmp(topic, "cmd/thermostat") == 0)
  {
    Serial.println(p);
    if(strcmp(p, "OFF") == 0)
    {
       thermostat = false;
    }
    else if(strcmp(p, "ON") == 0)
    {
      thermostat = true;
    }
  }
  //Nastavení módu - Off (vypnuto), Automatic (automatické), Thermostat (ovládání termostatem)
  if(strcmp(topic, "cmd/mode") == 0)
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
    lcd.SetMode(mode);
  }
  //Bod na nule v topné křivce
  if(strcmp(topic, "heater/cmd/zeroPoint") == 0)
  {
    int t = 0;
    sscanf(p, "%d", &t);
    equithermalCurveZeroPoint = t;
  }
  //Nastavení data a času
  if(strcmp(topic, "heater/cmd/currentDateTime") == 0)
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
  if(strcmp(topic, "events/insidetemperaturesetchanged") == 0)
  {
    insideTemperature = atof(p);
  }
}

void OutsideTemperatureChanged(double temperature, uint8_t channel, uint8_t sensorId, uint8_t* rawData, bool transmitedByButton)
{
  if(temperature < -35 && temperature > 50)
  {
    return;
  }
  
  if(transmitedByButton && channel == 1)
  {
    sensrId = sensorId; 
    EEPROM.put(0, sensorId);
  }
  
  if(channel == 1 && sensrId == sensorId)
  {
    outsideTemperature = temperature;
    convert_to_utf8(rawData, 5, utf8Buffer);
    homeAssistant.SetSensor((const char*) utf8Buffer, TOPIC_OUTSIDETEMPERATURE, true);
    lcd.SetOutTemperature(temperature);
    computeRequiredTemperature();
  }
}

double getTemperature(double pinValue, int beta)
{
  return 1 / (log((1023 / pinValue - 1)) / beta + NORMTEMP) - 273.15;
}

void computeRequiredTemperature()
{
  //nastavená teplota topné vody pro venkovní teplotu 0°C
  //touto proměnnou se nastavuje sklon topné křivky.
  double zeroTemp = equithermalCurveZeroPoint;
  Ds1302::DateTime now;
  rtc.getDateTime(&now);
  if(now.hour < 15 || now.hour >= 23)
  {
    zeroTemp = equithermalCurveZeroPoint - 4;
  }
  int newValue = (int)round(insideTemperature + (zeroTemp - insideTemperature) * pow((outsideTemperature - insideTemperature) / (double) - insideTemperature, 0.76923));

  if(newValue < 10 || newValue > 80)
  {
    return;
  }
  if(newValue != value)
  {
    value = newValue;
    lcd.SetRequiredTemperature(value);
    states[3] =  value;
  }
}

void setRelay(int pDirection)
{
  //value - celsius - 1 - nastavená 50, naměřená 40 = 50-40-1 = 9 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 60 = 50-60 = -10 => value < 0, direction = -1
  if(pDirection == -1)
  {
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    relayOn = true;
    relayOnMillis = currentMillis;
    direction = -1;
  }
  //value - celsis  - nastavená 50, naměřená 40 = 40-50=>10 => value > 0, direction = 1
  //value - celsius - nastavená 50, naměřená 50,68 = 50-50,68 = 0,68 - 0,5 = 0,18
  if(pDirection == 1)
  {
    digitalWrite(MOREHEATINGRELAYPIN, LOW);
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
}

bool ShouldBeHeatingOff()
{
  if(mode == 0)
  {
    return true;
  }
  if(mode == 1)
  {
    return outsideTemperature > 14.5 || states[1] < MININPUTTEMPERATURE - 1;
  }
  if(mode == 2)
  {
    return !thermostat || states[1] < MININPUTTEMPERATURE - 1;
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
    return states[1] > MININPUTTEMPERATURE + 1 && outsideTemperature <= 14;
  }
  if(mode == 2)
  {
    return thermostat && states[1] > MININPUTTEMPERATURE + 1;
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
    states[5] = 0;
  }
  if(heatingOff && ShouldBeHeatingOn())
  {
    heatingOff = false;
    states[5] = 1;
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
  if(states[1] != (int)round(newInputTemperature))
  { 
    lcd.SetInputTemperature((int)round(newInputTemperature));
    states[1] = (uint8_t)round(newInputTemperature);
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
    states[0] = (uint8_t)round(celsius);
  }
}

void readCurrentReturnTemperature()
{
  double newCelsius = getTemperature(analogRead(RETTEMPPIN), 3950);
  if(newCelsius < 0 || newCelsius > 100)
  {
    return;
  }
  if(newCelsius < states[2] - 0.5 || newCelsius > states[2] + 0.5)
  {
    states[2] = (uint8_t)round(newCelsius);
  }
}

void sendToHomeAssistant()
{
  if(currentMillis - lastMQTTSendMillis > 20000)
  {
    homeAssistant.SetSensor(states, 10, TOPIC_HEATERSTATE, true);
    lastMQTTSendMillis = currentMillis;
  }
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  if(!homeAssistant.Loop())
  {
    homeAssistant.Connect();
  }
  
  outsideTemperatureSensor.CheckTemperature();
  lcd.Print();
  if(currentMillis - temperatureReadMillis > 1000)
  {
    readCurrentHeatingTemperature();
    readCurrentReturnTemperature();
    readInputTemperature();
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
    states[4] = (uint8_t)round(position/(double)700);
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