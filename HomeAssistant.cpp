#include "HomeAssistant.h"
#include <PubSubClient.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(13, 12); // RX, TX
#endif

 HomeAssistant::HomeAssistant(char* ssid, char* wifiPassword, char* mqttUsername, char* mqttPassword, char* mqttServer, char* deviceId, void (*callback)(char*, uint8_t*, unsigned int))
{
  this->ssid = ssid;
  this->wifiPassword = wifiPassword;
  this->mqttUsername = mqttUsername;
  this->mqttPassword = mqttPassword;
  this->mqttServer = mqttServer;
  this->deviceId = deviceId;
  this->status = WL_IDLE_STATUS;
  this->callback = callback;
}

void HomeAssistant::Init()
{
  WiFi.init(&Serial1);
  espClient = new WiFiEspClient();
  mqttClient = new PubSubClient(*espClient);
  mqttClient->setServer(mqttServer, 1883);
  mqttClient->setCallback(callback);
  mqttClient->setKeepAlive(30);
}

int HomeAssistant::Connect()
{
  if (WiFi.status() == WL_NO_SHIELD) 
  {
    return WIFI_SHIELD_NOT_PRESENT;
  }
  if (status != WL_CONNECTED) 
  {
    status = WiFi.begin(ssid, wifiPassword);
  }
  if (status != WL_CONNECTED) 
  {
    return WIFI_NOT_CONNECTED;
  }
  if (!mqttClient->connected()) 
  {
    if (mqttClient->connect(deviceId, mqttUsername, mqttPassword)) 
    {
      return MQTT_CONNECTED;
    }
    else
    {
      return MQTT_NOT_CONNECTED;
    }
  }
  return MQTT_CONNECTED;
}

void HomeAssistant::SetSensor(int value, const char* topic)
{
  char cstr[16];
  dtostrf(value, 6, 2, cstr);
  SetSensor(cstr, topic);
}

void HomeAssistant::SetSensor(double value, const char* topic)
{
  char cstr[16];
  dtostrf(value, 6, 2, cstr);
  SetSensor(cstr, topic);
}

void HomeAssistant::SetSensor(const char* value, const char* topic)
{
  if(Connect() == MQTT_CONNECTED)
  {
    mqttClient->publish(topic, value);
  }
}

void HomeAssistant::Subscribe(const char* topic)
{
  mqttClient->subscribe(topic);
}

bool HomeAssistant::Connected()
{
  return mqttClient->connected();
}

bool HomeAssistant::Loop()
{
  return mqttClient->loop();
}