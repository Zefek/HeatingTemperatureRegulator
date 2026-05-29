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
#define TOPIC_DIAG "TOPIC_DIAG"

#define MAXTEMPDIFFERENCE 0
#define PCONST            0  //P konstanta pro PID
#define DCONST            0 //D konstanta pro PID
#define TEMPCHECKINTERVAL   0   //Vzorkovací interval
#define AUTOMATICTEMPERATURE 0
#define HEATEROVERHEATINGTEMPERATURE 0
#define OVERHEATINGTEMPERATURE 0
#define MININPUTTEMPERATURE 0
#define MAXIMALSERVOSTEP 3
#define SERVOMAXRANGE               70000 //Časový interval pohybu serva mezi krajními hodnotami
#define SERVO1PC                    700L //Jedno procento z intervalu serva
#define MINSERVOINTERVAL            700    //Minimální interval pro aktivaci serva
#define OUTSIDEAVGTAU               180.0   //Časová konstanta průměru venkovní teploty [min] (≈ původní 1/180)
#define OUTSIDETEMPERATURETIMEOUT   600000UL //Po této době bez dat ze snímače [ms] => na displeji --.-


#define HeaterOn(ht, wgt)  false
#define HeaterOff(ht, wgt)  false

double equithermalCurveZeroPoint = 0;
double insideTemperature = 0.0;
double exponent = 0.0;
unsigned long heaterStartTimeout = 0;
float wasteGasMinGradient = 0;
float wasteGasMaxGradient = 0;
short heaterStartTryCount = 0;
