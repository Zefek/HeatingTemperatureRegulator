#define WifiSSID "WifiSSID"
#define WifiPassword "WifiPassword"
#define MQTTUsername "MQTTUserName"

#define MQTTPassword "MQTTPassword"

#define MQTTHost "MQTTHost"

#define TOPIC_THERMOSTAT "TOPIC_THERMOSTAT"
#define TOPIC_MODE "TOPIC_MODE"
#define TOPIC_HEATERSELECTOR "TOPIC_HEATERSELECTOR"
#define TOPIC_ZEROPOINT "TOPIC_ZEROPOINT"
#define TOPIC_CURRENTDATETIEM "TOPIC_CURRENTDATETIEM"
#define TOPIC_THERMOSTATSETCHANGED "TOPIC_THERMOSTATSETCHANGED"
#define TOPIC_OUTSIDETEMPERATURE "TOPIC_OUTSIDETEMPERATURE"
#define TOPIC_HEATERSTATE "TOPIC_HEATERSTATE"
#define TOPIC_FVE "TOPIC_FVE"
#define TOPIC_FVE_STATE "TOPIC_FVE_STATE"

#define MAXTEMPDIFFERENCE 0
#define PCONST            0  //P konstanta pro PID
#define DCONST            0 //D konstanta pro PID
#define TEMPCHECKINTERVAL   0   //Vzorkovací interval
#define AUTOMATICTEMPERATURE 0
#define HEATEROVERHEATINGTEMPERATURE 0
#define OVERHEATINGTEMPERATURE 0
#define MININPUTTEMPERATURE 0
#define MAXIMALSERVOSTEP 3

#define HeaterOn(ht, wgt)  false
#define HeaterOff(ht, wgt)  false

double equithermalCurveZeroPoint = 0;
double insideTemperature = 0.0;
double exponent = 0.0;

