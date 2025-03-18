#include <Ds1302.h>
#include <EEPROM.h>
#include "display.h"
#include "TX07K-TXC.h"
#include "config.h"
#include <avr/wdt.h>
#include "TemperatureSensors.h"
#include "BelWattmeter.h"
#include <MQTTClient.h>
#include <EspDrv.h>

#define I2C_ADDR            0x27
#define LCD_COLUMNS         16
#define LCD_LINES           2
#define MOREHEATINGRELAYPIN 8
#define LESSHEATINGRELAYPIN 9
#define HEATINGPUMPRELAYPIN 10
#define MININPUTTEMPERATURE 30
#define ONEWIREBUSPIN       7

void OutsideTemperatureChanged(double temperature, uint8_t channel, uint8_t sensorId, uint8_t* rawData, bool transmitedByButton);
void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length);
void computeRequiredTemperature();

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
0 - currentTemperature
1 - inputTemperature
2 - returnTempareture
3 - setTemperature
4 - valve
5 - heater
6 - acumulator temperature 1
7 - acumulator temperature 2
8 - acumulator temperature 3
9 - acumulator temperature 4
10 - heater temperature
11 - heater Waste temperature B0 (LSB)
12 - heater Waste temperature B1
13 - heater Waste temperature B2
14 - heater Waste temperature B3 (MSB)
15 - heater return temperature
*/
uint8_t states[16];
Display lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Ds1302 rtc(4, 5, 6);
TX07KTXC outsideTemperatureSensor(2, 3, OutsideTemperatureChanged);
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "Heater", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 }; 

EspDrv drv(&Serial1);
MQTTClient client(&drv, MQTTMessageReceive);

TemperatureSensors tempSensors(ONEWIREBUSPIN);

unsigned long relayOnMillis = 0;
unsigned long relayOffMillis = 0;
unsigned long currentMillis = 0;
unsigned long temperatureReadMillis = 0;
unsigned long lastMQTTSendMillis = 0;
unsigned long lastRegulatorMeassurement = 0;
long interval = 0;
long position = 70000;
bool relayOn = false;
float value = 25;
float celsius = 25;
double lastCelsius = 25;
short direction = 0;
bool heatingOff = true;
double outsideTemperature = 14;
int sensor = 1;
bool thermostat = false;
double totalPower = 0;
uint8_t sensrId = 0;
uint8_t mode = 1; //0 - Off, 1 - Automat, 2 - Thermostat
uint8_t equithermalCurveZeroPoint = 40;
double insideTemperature = 23;
unsigned char utf8Buffer[32];
unsigned char mqttReceivedData[24];
unsigned int voltage = 0; 
unsigned int current = 0;
unsigned int consumption = 0;
unsigned int power = 0;
unsigned int averageWasteGasTemperature = 0;
unsigned long averageWasteGasTemperatureCount = 0;
BelWattmeter belWattmeter(&voltage, &current, &consumption, &power);

void setup() {
  Serial.begin(57600);
  //AT+UART_DEF=57600,8,1,0,0
  Serial1.begin(57600);
  Serial2.begin(9600);
  sensrId = EEPROM.read(0);
  Serial.print("Sensor Id: ");
  Serial.println(sensrId);
  rtc.init();
  lcd.Init(&rtc);
  lcd.BackLight();
  outsideTemperatureSensor.Init();
  tempSensors.Init();
  drv.Init(64);
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
  lastCelsius = celsius;
  readInputTemperature();
  ComputeWasteGasTemperature();
  lcd.SetWasteGasTemperature(averageWasteGasTemperature);
  lcd.Print();
  wdt_enable(WDTO_8S);
}

void MQTTConnect()
{
  int wifiStatus = drv.GetConnectionStatus();
  Serial.print("Wifi status ");
  Serial.println(wifiStatus);
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = drv.Connect(WifiSSID, WifiPassword);
  }
  if(wifiConnected)
  {
    bool isConnected = client.IsConnected();
    if(!isConnected)
    {
      Serial.println("Connect");
      if(client.Connect(mqttConnectData))
      {
        Serial.println("Subscribes");
        client.Subscribe(TOPIC_THERMOSTAT);
        client.Subscribe(TOPIC_MODE);
        client.Subscribe(TOPIC_ZEROPOINT);
        client.Subscribe(TOPIC_THERMOSTATSETCHANGED);
        client.Subscribe(TOPIC_CURRENTDATETIEM);
      }
      else
      {
        mode = 1;
        lcd.SetMode(mode);
        equithermalCurveZeroPoint = 40;
        insideTemperature = 22.5;
      }
    }
  }
}

void MQTTMessageReceive(char* topic, uint8_t* payload, unsigned int length)
{
  for(int i = 0; i<length; i++)
  {
    mqttReceivedData[i] = (char)payload[i];
  }
  mqttReceivedData[length] = '\0';
  //Termostat on/off
  if(strcmp(topic, TOPIC_THERMOSTAT) == 0)
  {
    if(strcmp(mqttReceivedData, "OFF") == 0)
    {
       thermostat = false;
    }
    else if(strcmp(mqttReceivedData, "ON") == 0)
    {
      thermostat = true;
    }
  }
  //Nastavení módu - Off (vypnuto), Automatic (automatické), Thermostat (ovládání termostatem)
  if(strcmp(topic, TOPIC_MODE) == 0)
  {
    if(strcmp(mqttReceivedData, "Off") == 0)
    {        
      mode = 0;
    }
    else if(strcmp(mqttReceivedData, "Automatic") == 0)
    {
      mode = 1;
    }
    else if (strcmp(mqttReceivedData, "Thermostat") == 0)
    {
      mode = 2;
    }
    lcd.SetMode(mode);
  }
  //Bod na nule v topné křivce
  if(strcmp(topic, TOPIC_ZEROPOINT) == 0)
  {
    int t = 0;
    sscanf(mqttReceivedData, "%d", &t);
    equithermalCurveZeroPoint = t;
    computeRequiredTemperature();
  }
  //Nastavení data a času
  if(strcmp(topic, TOPIC_CURRENTDATETIEM) == 0)
  {
    int day, month, year, hour, minute, second;
    sscanf(mqttReceivedData, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);
    Ds1302::DateTime dt;
    dt.hour = hour;
    dt.minute = minute;
    dt.second = second;
    dt.day = day;
    dt.month = month;
    dt.year = year;
    rtc.setDateTime(&dt);
  }
  if(strcmp(topic, TOPIC_THERMOSTATSETCHANGED) == 0)
  {
    insideTemperature = atof(mqttReceivedData);
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
    client.Publish(TOPIC_OUTSIDETEMPERATURE, (const char*) utf8Buffer, true);
    lcd.SetOutTemperature(temperature);
    computeRequiredTemperature();
  }
}

void computeRequiredTemperature()
{
  //nastavená teplota topné vody pro venkovní teplotu 0°C
  //touto proměnnou se nastavuje sklon topné křivky.
  float zeroTemp = equithermalCurveZeroPoint;
  int newValue = (int)round(insideTemperature + (zeroTemp - insideTemperature) * pow((outsideTemperature - insideTemperature) / (float) - insideTemperature, 0.76923));

  if(newValue < 10 || newValue > 80)
  {
    return;
  }
  if(newValue != value)
  {
    value = newValue;
    lcd.SetRequiredTemperature((uint8_t)round(value));
    states[3] =  value;
  }
}

void ComputeWasteGasTemperature()
{
  unsigned long gasTempValue = analogRead(A0);
  gasTempValue = 1024 - gasTempValue;
  double R1 = (gasTempValue * 10000) / ((double)1024 - gasTempValue);
  int T = (int)((sqrt((-0.00232 * R1) + 17.59246) - 3.908) / 0.00116) * (-1);
  averageWasteGasTemperature = ((averageWasteGasTemperature * averageWasteGasTemperatureCount) + T) / (averageWasteGasTemperatureCount + 1);
  averageWasteGasTemperatureCount++;
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
    return outsideTemperature > 14.5 || states[1] < MININPUTTEMPERATURE;
  }
  if(mode == 2)
  {
    return !thermostat || states[1] < MININPUTTEMPERATURE;
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
  uint8_t newInputTemperature = states[1];
  tempSensors.GetAcumulatorOutputTemperature(&newInputTemperature);
  if(states[1] != newInputTemperature)
  { 
    lcd.SetInputTemperature(newInputTemperature);
    states[1] = newInputTemperature;
  }
}

void readCurrentHeatingTemperature()
{
  float newCelsius = celsius;
  tempSensors.GetCurrentHeatingTemperature(&newCelsius);
  if(newCelsius < 0 || newCelsius > 100)
  {
    return;
  }
  if(newCelsius < celsius - 0.5 || newCelsius > celsius + 0.5)
  {
    celsius = newCelsius;
    lcd.SetCurrentHeatingTemperature((uint8_t)round(celsius));
    states[0] = (uint8_t)round(celsius);
  }
}

unsigned int sendIndex = 0;
void sendToHomeAssistant()
{
  if(currentMillis - lastMQTTSendMillis > 30000)
  {
    if(sendIndex == 0)
    {
      sendHeaterToHomeAssistant();
      sendIndex = 1;
    }
    else
    {
      sendFVEToHomeAssistant();
      sendIndex = 0;
    }
    lastMQTTSendMillis = currentMillis;
  }
}
void sendHeaterToHomeAssistant()
{
  tempSensors.GetAcumulator1Temperature(&states[6]);
  tempSensors.GetAcumulator2Temperature(&states[7]);
  tempSensors.GetAcumulator3Temperature(&states[8]);
  tempSensors.GetAcumulator4Temperature(&states[9]);
  tempSensors.GetReturnHeatingTemperature(&states[2]);
  tempSensors.GetHeaterTemperature(&states[10]);
  lcd.SetWasteGasTemperature(averageWasteGasTemperature);
  int T = averageWasteGasTemperature;
  for(int i = 11; i < 15; i++)
  {
    uint8_t v = T & 0x0F;
    states[i] = v < 10? (char)('0'+v):(char)('7'+v);
    T = T >> 4;
  }
  averageWasteGasTemperatureCount = 0;
  client.Publish(TOPIC_HEATERSTATE, states, 16, true);
  Serial.println("Heater publish");
}

void sendFVEToHomeAssistant()
{
  uint8_t fveData[8];
  fveData[0] = (voltage >> 8) & 0xFF;
  fveData[1] = voltage & 0xFF;
  fveData[2] = (current >> 8) & 0xFF;
  fveData[3] = current & 0xFF;
  fveData[4] = (consumption >> 8) & 0xFF;
  fveData[5] = consumption & 0xFF;
  fveData[6] = (power >> 8) & 0xFF;
  fveData[7] = power & 0xFF;
  convert_to_utf8(fveData, 8, utf8Buffer);
  client.Publish(TOPIC_FVE, (const char*)utf8Buffer, 16, true);
  Serial.println("FVE publish");
  voltage = 0;
  current = 0;
  power = 0;
  consumption = 0;
  belWattmeter.Reset();
}

void loop() {
  wdt_reset();
  currentMillis = millis();
  if(!client.Loop())
  {
    MQTTConnect();
  }
  outsideTemperatureSensor.CheckTemperature();
  belWattmeter.Loop();
  if(currentMillis - temperatureReadMillis > 1000)
  {
    readCurrentHeatingTemperature();
    readInputTemperature();
    ComputeWasteGasTemperature();
    temperatureReadMillis = currentMillis;
  }
  lcd.Print();
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
  }
 
  if(!heatingOff && currentMillis - lastRegulatorMeassurement > 20000)
  {
    /*
    Serial.print("P: ");
    Serial.print(value);
    Serial.print(" - ");
    Serial.print(celsius);
    Serial.print(" = ");
    Serial.print(value - celsius);
    Serial.print(" D: ");
    Serial.print(celsius);
    Serial.print(" - ");
    Serial.print(lastCelsius);
    Serial.print(" = ");
    Serial.println(celsius - lastCelsius);
    */
    long intervalTmp = ((value - celsius) * 600) - ((celsius - lastCelsius) * 4500);
    lastCelsius = celsius;
    if(intervalTmp <= 0 && !relayOn)
    {
      interval = min(abs(intervalTmp), 70000);
      setRelay(-1);
    }
    else if(value - celsius > 0 && !relayOn)
    {
      interval = min(abs(intervalTmp), 70000);
      setRelay(1);
    }
    lastRegulatorMeassurement = currentMillis;
  }
}