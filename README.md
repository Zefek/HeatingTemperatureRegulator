Blokové schéma zapojení
![heater_regulator](https://github.com/user-attachments/assets/4e85691c-065a-4078-b046-f0943ee74d2e)

Na pinu 7 jsou DS18B20 snímače pro snímání teplot kotle, akumulace a bojleru. Snímač pro teplotu spalin je PT1000. 
Pro zprovoznění regulátoru je potřeba soubor config.h
```
//SSID Wifi pro připojení k wifi
#define WifiSSID "WifiSSID"
//Heslo k wifi
#define WifiPassword "WifiPassword"

//Uživatelské jméno pro MQTT
#define MQTTUsername "MQTTUserName"
//Heslo pro MQTT
#define MQTTPassword "MQTTPassword"
//Host MQTT
#define MQTTHost "MQTTHost"

//MQTT Topic pro ovládání termostatem (ON/OFF)
#define TOPIC_THERMOSTAT "TOPIC_THERMOSTAT"
//MQTT Topic pro nastavení regulátoru (Off - Vypnuto, Automatic - topí dokud je akumulace nad 30°C, Thermostat - ovládání termostatem)
#define TOPIC_MODE "TOPIC_MODE"
//MQTT Topic pro nastavení topné křivky
#define TOPIC_ZEROPOINT "TOPIC_ZEROPOINT"
//MQTT Topic pro synchronizaci data a času
#define TOPIC_CURRENTDATETIEM "TOPIC_CURRENTDATETIEM"
//MQTT Topic pro nastavení požadované vnitřní teploty (ovlivňuje topnou křivku)
#define TOPIC_THERMOSTATSETCHANGED "TOPIC_THERMOSTATSETCHANGED"
//MQTT Topic pro odeslání dat o venkovní teplotě
#define TOPIC_OUTSIDETEMPERATURE "TOPIC_OUTSIDETEMPERATURE"
//MQTT Topic pro odeslání stavu regulátoru
#define TOPIC_HEATERSTATE "TOPIC_HEATERSTATE"
//MQTT Topic pro odeslání stavu BEL (FVE)
#define TOPIC_FVE "TOPIC_FVE"
```
A také soubor sensorconfig.h
```
DeviceAddress acumulatorFirst =    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorSecond =   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorThird =    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorFour =     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorOutput =   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress boiler =             { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress currentHeating =     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress returnHeating =      { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress heaterTemperature =  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```
