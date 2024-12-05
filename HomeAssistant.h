#include <WiFiEspClient.h>
#include <PubSubClient.h>

#define MQTT_CONNECTED 0
#define WIFI_SHIELD_NOT_PRESENT 1
#define WIFI_NOT_CONNECTED 2
#define MQTT_NOT_CONNECTED 3

class HomeAssistant
{
    private:
      char* ssid = "Berek2";
      int status;
      char* wifiPassword;
      char* mqttUsername;
      char* mqttPassword;
      char* mqttServer;
      char* deviceId;
      WiFiEspClient* espClient;
      PubSubClient* mqttClient;
      void (*callback)(char*, uint8_t*, unsigned int);
    
    public: 
      HomeAssistant(char* ssid, char* wifiPassword, char* mqttUsername, char* mqttPassword, char* mqttServer, char* deviceId, void (*callback)(char*, uint8_t*, unsigned int));
      int Connect();
      void SetSensor(int value, const char* topic);
      void SetSensor(double value, const char* topic);
      void SetSensor(const char* value, const char* topic);
      void Subscribe(const char* topic);
      bool Connected();
      void Init();
      bool Loop();
};