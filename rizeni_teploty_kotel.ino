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
#define MININPUTTEMPERATURE 30 //Minimální teplota na výstupu z akumulace
#define ONEWIREBUSPIN       7
#define SERVOMAXRANGE       70000 //Časový interval pohybu serva mezi krajními hodnotami
#define SERVO1PC            700 //Jedno procento z intervalu serva
#define PCONST              1500  //P konstanta pro PID
#define DCONST              2000000 //D konstanta pro PID
#define MINSERVOINTERVAL    1500    //Minimální interval pro aktivaci serva
#define TEMPCHECKINTERVAL   20000   //Vzorkovací interval
#define AVGOUTTEMPVALUES    180     //Počet hodnot pro výpočet průměrné venkovní teploty (počet minut)
#define AVGWASTETEMPVALUES  60.0      //Počet hodnot pro výpočet průměrné teploty v komínu (počet sekund)

void OutsideTemperatureChanged(double temperature, uint8_t channel, uint8_t sensorId, uint8_t* rawData, bool transmitedByButton);
void MQTTMessageReceive(char* topic, uint8_t* payload, uint16_t length);
void computeRequiredTemperature();

void convertToHalfByte(int value, uint8_t* result, uint8_t length)
{
  for(int i = 0; i<length; i++)
  {
    uint8_t v = value & 0x0F;
    result[i] = v < 10? (char)('0'+v):(char)('7'+v);
    value = value >> 4;
  }
}

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
16 - boiler temperature
17 - averageOutsideTemperature B0 (LSB)
18 - averageOutsideTemperature B1
19 - averageOutsideTemperature B2
20 - averageOutsideTemperature B3 (MSB)
*/
#pragma pack(push, 1)
struct HeaterState {
  uint8_t currentTemp;
  uint8_t inputTemp;
  uint8_t returnTemp;
  uint8_t setTemp;
  uint8_t valvePosition;
  uint8_t heatingActive;
  uint8_t acum1;
  uint8_t acum2;
  uint8_t acum3;
  uint8_t acum4;
  uint8_t heaterTemp;
  uint8_t wasteGasTemp[4];
  uint8_t returnHeaterTemp;
  uint8_t boilerTemp;
  uint8_t outsideAvgTemp[4];
};

struct FVEData
{
  uint8_t voltage[4];
  uint8_t current[4];
  uint8_t power[4];
  uint8_t consumption[4];
};
#pragma pack(pop)

HeaterState currentState;
HeaterState lastState;

FVEData currentFveData;
FVEData lastFveData;
BelData belData;

uint8_t temperatureDataToCompare[5];
Display lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);
Ds1302 rtc(4, 5, 6);
TX07KTXC outsideTemperatureSensor(2, 3, OutsideTemperatureChanged);
MQTTConnectData mqttConnectData = { MQTTHost, 1883, "Heater", MQTTUsername, MQTTPassword, "", 0, false, "", false, 0x0 }; 

EspDrv drv(&Serial1);
MQTTClient client(&drv, MQTTMessageReceive);

TemperatureSensors tempSensors(ONEWIREBUSPIN);

enum HeatingMode : uint8_t {
  MODE_OFF = 0,
  MODE_AUTOMATIC = 1,
  MODE_THERMOSTAT = 2
};

uint32_t relayOnMillis = 0;
uint32_t relayOffMillis = 0;
uint32_t currentMillis = 0;
uint32_t temperatureReadMillis = 0;
uint32_t lastMQTTSendMillis = 0;
uint32_t lastRegulatorMeasurement  = 0;
long interval = 0;
long position = SERVOMAXRANGE;
bool relayOn = false;
int8_t direction = 0;
double outsideTemperature = 14;
double outsideTemperatureAverage = 0;
int outsideTemperatureAverageCount = 0;
int sensor = 1;
bool thermostat = false;
uint8_t sensrId = 0;
int lastCurrentTemp = 0;
HeatingMode mode = MODE_AUTOMATIC;
float equithermalCurveZeroPoint = 40;
double insideTemperature = 23;
unsigned char utf8Buffer[32];
unsigned char mqttReceivedData[24];
double averageWasteGasTemperature = 0;
BelWattmeter belWattmeter;
unsigned long mqttConnectionTimeout = 0;
unsigned long mqttLastConnectionTry = 0;
double exponent = 0.76923;

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
  readInputTemperature();
  ComputeWasteGasTemperature();
  lcd.SetWasteGasTemperature(averageWasteGasTemperature);
  ComputeOutsideTemperatureAverage();
  lcd.Print();
  //wdt_enable(WDTO_8S);
}

void MQTTConnect()
{
  if(currentMillis - mqttLastConnectionTry < mqttConnectionTimeout)
  {
    return;
  }
  int wifiStatus = drv.GetConnectionStatus();
  Serial.print("Wifi status ");
  Serial.println(wifiStatus);
  bool wifiConnected = wifiStatus == WL_CONNECTED;
  if(wifiStatus == WL_DISCONNECTED || wifiStatus == WL_IDLE_STATUS)
  {
    wifiConnected = drv.Connect(WifiSSID, WifiPassword);
    mqttLastConnectionTry = currentMillis;
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
        mqttLastConnectionTry = currentMillis;
        mqttConnectionTimeout = 0;
      }
      else
      {
        mode = MODE_AUTOMATIC;
        lcd.SetMode(mode);
        equithermalCurveZeroPoint = 40;
        exponent = 0.76923;
        insideTemperature = 22.5;
        mqttLastConnectionTry = currentMillis;
        mqttConnectionTimeout = min(mqttConnectionTimeout + random(5000, 30000), 300000);
      }
    }
  }
  else
  {
    mqttConnectionTimeout = min(mqttConnectionTimeout + random(5000, 30000), 300000);
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
      mode = MODE_OFF;
    }
    else if(strcmp(mqttReceivedData, "Automatic") == 0)
    {
      mode = MODE_AUTOMATIC;
    }
    else if (strcmp(mqttReceivedData, "Thermostat") == 0)
    {
      mode = MODE_THERMOSTAT;
    }
    lcd.SetMode(mode);
  }
  //Bod na nule v topné křivce
  if(strcmp(topic, TOPIC_ZEROPOINT) == 0)
  {
    char* token;
    token = strtok(mqttReceivedData, ";");
    equithermalCurveZeroPoint = atof(token);
    token = strtok(NULL, ";");
    exponent = atof(token);
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
  if(temperature < -35 || temperature > 50)
  {
    return;
  }
  
  if(transmitedByButton && channel == 1)
  {
    sensrId = sensorId; 
    EEPROM.update(0, sensorId);
  }
  
  if(channel == 1 && sensrId == sensorId)
  {
    outsideTemperature = temperature;
    if (memcmp(rawData, temperatureDataToCompare, sizeof(rawData)) != 0)
    {
      convert_to_utf8(rawData, 5, utf8Buffer);
      client.Publish(TOPIC_OUTSIDETEMPERATURE, (const char*) utf8Buffer, true);
      memcpy(temperatureDataToCompare, rawData, sizeof(rawData));
    }
    lcd.SetOutTemperature(temperature);
  }
}

void computeRequiredTemperature()
{
  //nastavená teplota topné vody pro venkovní teplotu 0°C
  //touto proměnnou se nastavuje sklon topné křivky.
  float zeroTemp = equithermalCurveZeroPoint;
  int newValue = (int)round(insideTemperature + (zeroTemp - insideTemperature) * pow((outsideTemperatureAverage - insideTemperature) / (-insideTemperature), exponent));

  if(newValue < 10 || newValue > 80)
  {
    return;
  }
  if(newValue != currentState.setTemp)
  {
    currentState.setTemp = newValue;
    lcd.SetRequiredTemperature(currentState.setTemp);
  }
}

void ComputeOutsideTemperatureAverage()
{
  double oldAverage = outsideTemperatureAverage;
  if(outsideTemperatureAverageCount < AVGOUTTEMPVALUES)
  {
    if(outsideTemperatureAverageCount == 0)
    {
      outsideTemperatureAverage = outsideTemperature;
    }
    else
    {
      outsideTemperatureAverage = ((outsideTemperatureAverage * outsideTemperatureAverageCount) + outsideTemperature) / (outsideTemperatureAverageCount + 1);
    }
    outsideTemperatureAverageCount++;
  }
  else
  {
    outsideTemperatureAverage = ((1 / (double)outsideTemperatureAverageCount) * outsideTemperature) + (((outsideTemperatureAverageCount - 1) / (double)outsideTemperatureAverageCount) * outsideTemperatureAverage);
  }
  if(oldAverage != outsideTemperatureAverage)
  {
    int T = (int)(outsideTemperatureAverage * 10);
    convertToHalfByte(T, currentState.outsideAvgTemp, 4);
    computeRequiredTemperature();
  }
}

void ComputeWasteGasTemperature()
{
  uint32_t gasTempValue = analogRead(A0);
  gasTempValue = 1024 - gasTempValue;
  double R1 = (gasTempValue * 10000) / ((double)1024 - gasTempValue);
  int T = (int)((sqrt((-0.00232 * R1) + 17.59246) - 3.908) / 0.00116) * (-1);
  if(T > 0 && T < 450)
  {
    if(averageWasteGasTemperature == 0)
    {
      averageWasteGasTemperature = T;
    }
    else
    {
      averageWasteGasTemperature = ((1 / AVGWASTETEMPVALUES) * T) + (((AVGWASTETEMPVALUES - 1) / AVGWASTETEMPVALUES) * averageWasteGasTemperature);
    }
    lcd.SetWasteGasTemperature((int)averageWasteGasTemperature);
  }
}

void setRelay(int pDirection)
{
  if(pDirection == -1)
  {
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    relayOn = true;
    relayOnMillis = currentMillis;
    direction = -1;
  }
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
    return currentState.currentTemp < MININPUTTEMPERATURE;
  }
  if(mode == 2)
  {
    return !thermostat || currentState.currentTemp < MININPUTTEMPERATURE;
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
    return currentState.currentTemp > MININPUTTEMPERATURE + 1;
  }
  if(mode == 2)
  {
    return thermostat && currentState.currentTemp > MININPUTTEMPERATURE + 1;
  }
  return false;
}

void checkHeating()
{
  if(currentState.heatingActive == 1 && ShouldBeHeatingOff())
  {
    digitalWrite(LESSHEATINGRELAYPIN, LOW);
    digitalWrite(HEATINGPUMPRELAYPIN, HIGH);
    interval = SERVOMAXRANGE;
    direction = -1;
    relayOn = true;
    relayOnMillis = currentMillis;
    currentState.heatingActive = 0;
  }
  if(currentState.heatingActive == 0 && ShouldBeHeatingOn())
  {
    currentState.heatingActive = 1;
    digitalWrite(HEATINGPUMPRELAYPIN, LOW);
  }
  lcd.SetHeating(currentState.heatingActive == 1? true : false);
}

void readInputTemperature()
{
  uint8_t newInputTemperature = currentState.inputTemp;
  tempSensors.GetAcumulatorOutputTemperature(&newInputTemperature);
  if(currentState.inputTemp != newInputTemperature)
  { 
    lcd.SetInputTemperature(newInputTemperature);
    currentState.inputTemp = newInputTemperature;
  }
}

void readCurrentHeatingTemperature()
{
  uint8_t newCelsius = currentState.currentTemp;
  tempSensors.GetCurrentHeatingTemperature(&newCelsius);
  if(newCelsius > 100)
  {
    return;
  }
  if(newCelsius != currentState.currentTemp)
  {
    currentState.currentTemp = (uint8_t)round(newCelsius);
    lcd.SetCurrentHeatingTemperature(currentState.currentTemp);
  }
}

unsigned int sendIndex = 0;
void sendToHomeAssistant()
{
  if(currentMillis - lastMQTTSendMillis > 30000)
  {
    if(sendIndex == 0)
    {
      ComputeOutsideTemperatureAverage();
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
  tempSensors.GetAcumulator1Temperature(&currentState.acum1);
  tempSensors.GetAcumulator2Temperature(&currentState.acum2);
  tempSensors.GetAcumulator3Temperature(&currentState.acum3);
  tempSensors.GetAcumulator4Temperature(&currentState.acum4);
  tempSensors.GetReturnHeatingTemperature(&currentState.returnTemp);
  tempSensors.GetHeaterTemperature(&currentState.heaterTemp);
  tempSensors.GetBoilerTemperature(&currentState.boilerTemp);
  convertToHalfByte((int)averageWasteGasTemperature, currentState.wasteGasTemp, 4);
  if (memcmp(&currentState, &lastState, sizeof(HeaterState)) != 0)
  {
    uint8_t buffer[sizeof(HeaterState)];
    memcpy(buffer, &currentState, sizeof(HeaterState));
    client.Publish(TOPIC_HEATERSTATE, buffer, sizeof(HeaterState), true);
    Serial.println("Heater publish");
    memcpy(&lastState, &currentState, sizeof(HeaterState));
  }
}

void sendFVEToHomeAssistant()
{
  BelData data = belWattmeter.GetBelData();
  convertToHalfByte(data.voltage, currentFveData.voltage, 4);
  convertToHalfByte(data.current, currentFveData.current, 4);
  convertToHalfByte(data.consumption, currentFveData.consumption, 4);
  convertToHalfByte(data.power, currentFveData.power, 4);
  if (memcmp(&currentFveData, &lastFveData, sizeof(FVEData)) != 0)
  {
    uint8_t buffer[sizeof(FVEData)];
    memcpy(buffer, &currentFveData, sizeof(FVEData));
    client.Publish(TOPIC_FVE, (char*)buffer, sizeof(buffer), true);
    Serial.println("FVE publish");
    memcpy(&lastFveData, &currentFveData, sizeof(FVEData));
  }
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
    position = min(max(position, 0), SERVOMAXRANGE);
    currentState.valvePosition = (uint8_t)round(position/(double)SERVO1PC);
    relayOffMillis = currentMillis;
  }
 
  if(currentState.heatingActive == 1 && currentMillis - lastRegulatorMeasurement  > TEMPCHECKINTERVAL)
  {
    int diff = (int)currentState.setTemp - (int)currentState.currentTemp;
    double change = ((int)lastCurrentTemp - (int)currentState.currentTemp) / ((double)(currentMillis - lastRegulatorMeasurement));
    long newInterval = constrain((diff * PCONST) + (change * DCONST), -SERVOMAXRANGE, SERVOMAXRANGE);
    lastCurrentTemp = currentState.currentTemp;
    if(abs(newInterval) > MINSERVOINTERVAL && !relayOn)
    {
      interval = abs(newInterval);
      setRelay(newInterval < 0? - 1 : 1);
    }
    lastRegulatorMeasurement  = currentMillis;
  }
}