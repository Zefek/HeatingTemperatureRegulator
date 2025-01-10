#include "HomeAssistant.h"
#include <PubSubClient.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(13, 12); // RX, TX
#endif

HomeAssistant::HomeAssistant(char* ssid, char* wifiPassword, char* mqttUsername, char* mqttPassword, char* mqttServer, char* deviceId, void (*callback)(char*, uint8_t*, unsigned int), void (*onConnected)(bool success))
{
  this->ssid = ssid;
  this->wifiPassword = wifiPassword;
  this->mqttUsername = mqttUsername;
  this->mqttPassword = mqttPassword;
  this->mqttServer = mqttServer;
  this->deviceId = deviceId;
  this->status = WL_IDLE_STATUS;
  this->callback = callback;
  this->onConnected = onConnected;
}

void HomeAssistant::Init()
{
  WiFi.init(&Serial1);
  espClient = new WiFiEspClient();
  mqttClient = new PubSubClient(*espClient);
  mqttClient->setServer(mqttServer, 1883);
  mqttClient->setCallback(callback);
  mqttClient->setKeepAlive(30);
  mqttClient->setSocketTimeout(15);
}

int HomeAssistant::Connect()
{
  status = WiFi.status();
  if (status == WL_NO_SHIELD) 
  {
    onConnected(false);
    return WIFI_SHIELD_NOT_PRESENT;
  }
  if (status != WL_CONNECTED) 
  {
    status = WiFi.begin(ssid, wifiPassword);
  }
  if (status != WL_CONNECTED) 
  {
    onConnected(false);
    return WIFI_NOT_CONNECTED;
  }
  if (!mqttClient->connected()) 
  {
    if (mqttClient->connect(deviceId, mqttUsername, mqttPassword)) 
    {
      onConnected(true);
      return MQTT_CONNECTED;
    }
    else
    {
      onConnected(false);
      return MQTT_NOT_CONNECTED;
    }
  }
  onConnected(true);
  return MQTT_CONNECTED;
}

void HomeAssistant::SetSensor(int value, const char* topic, bool retain = false)
{
  char cstr[16];
  dtostrf(value, 6, 2, cstr);
  SetSensor(cstr, topic, retain);
}

void HomeAssistant::SetSensor(double value, const char* topic, bool retain = false)
{
  char cstr[16];
  dtostrf(value, 6, 2, cstr);
  SetSensor(cstr, topic, retain);
}

void HomeAssistant::SetSensor(const char* value, const char* topic, bool retain = false)
{
  if(Connect() == MQTT_CONNECTED)
  {
    mqttClient->loop();
    mqttClient->publish(topic, value, retain);
    mqttClient->loop();
  }
}

void HomeAssistant::SetSensor(uint8_t* payload, unsigned int plength, const char* topic, bool retain = false)
{
  if(Connect() == MQTT_CONNECTED)
  {
    mqttClient->loop();
    mqttClient->publish(topic, payload, plength, retain);
    mqttClient->loop();
  }
}

void HomeAssistant::Subscribe(const char* topic)
{
  mqttClient->loop();
  mqttClient->subscribe(topic);
  mqttClient->loop();
}

bool HomeAssistant::Connected()
{
  return mqttClient->connected();
}

bool HomeAssistant::Loop()
{
  return mqttClient->loop();
}