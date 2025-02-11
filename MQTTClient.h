#ifndef __MQTTCLIENT_H
#define __MQTTCLIENT_H

#include "EspDrv.h"

#define MQTT_VERSION_3_1      3
#define MQTT_VERSION_3_1_1    4

// MQTT_VERSION : Pick the version
//#define MQTT_VERSION MQTT_VERSION_3_1
#ifndef MQTT_VERSION
#define MQTT_VERSION MQTT_VERSION_3_1_1
#endif

#ifndef MQTT_KEEPALIVE
#define MQTT_KEEPALIVE 15
#endif

#define MQTT_MAX_HEADER_SIZE 5

#define CHECK_STRING_LENGTH(l,s) if (l+2+strnlen(s, this->bufferSize) > this->bufferSize) {return false;}

#define MQTTCONNECT     1 << 4  // Client request to connect to Server
#define MQTTCONNACK     2 << 4  // Connect Acknowledgment
#define MQTTPUBLISH     3 << 4  // Publish message
#define MQTTPUBACK      4 << 4  // Publish Acknowledgment
#define MQTTPUBREC      5 << 4  // Publish Received (assured delivery part 1)
#define MQTTPUBREL      6 << 4  // Publish Release (assured delivery part 2)
#define MQTTPUBCOMP     7 << 4  // Publish Complete (assured delivery part 3)
#define MQTTSUBSCRIBE   8 << 4  // Client Subscribe request
#define MQTTSUBACK      9 << 4  // Subscribe Acknowledgment
#define MQTTUNSUBSCRIBE 10 << 4 // Client Unsubscribe request
#define MQTTUNSUBACK    11 << 4 // Unsubscribe Acknowledgment
#define MQTTPINGREQ     12 << 4 // PING Request
#define MQTTPINGRESP    13 << 4 // PING Response
#define MQTTDISCONNECT  14 << 4 // Client is Disconnecting
#define MQTTReserved    15 << 4 // Reserved

#define MQTTQOS0        (0 << 1)
#define MQTTQOS1        (1 << 1)
#define MQTTQOS2        (2 << 1)

#define MQTT_CONNECTED 0
#define MQTT_NOTCONNECTED 1

class MQTTClient
{
  private:
    EspDrv* client;
    uint8_t* buffer;
    uint16_t bufferSize = 256;
    uint16_t keepAlive = 30;
    unsigned long lastOutActivity;
    unsigned long lastInActivity;
    static bool pingOutstanding;
    uint32_t nextMsgId;
    static void DataReceived(uint8_t* data, int length);
    void Login(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession);
    uint16_t WriteString(const char* string, uint8_t* buf, uint16_t pos);
    void Write(uint8_t header, uint8_t* buf, uint16_t length);
    size_t BuildHeader(uint8_t header, uint8_t* buf, uint16_t length);
    static void (*callback)(char* topic, uint8_t* payload, uint16_t plength);
    void (*connected)();
    static uint8_t connectionState;
    static uint8_t stateChangedToConnected;
  public:
    MQTTClient(EspDrv *espDriver, void(*callback)(char* topic, uint8_t* payload, uint16_t plength), void(*connected)());
    void Connect(const char* url, uint16_t port, const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession);
    void Disconnect();
    void Subscribe(const char* topic);
    void Subscribe(const char* topic, uint8_t qos);
    void Publish(const char* topic, const char* payload);
    void Publish(const char* topic, const char* payload, boolean retained);
    void Publish(const char* topic, const uint8_t* payload, unsigned int plength);
    void Publish(const char* topic, const uint8_t* payload, unsigned int plength, boolean retained);
    void Loop();
    uint8_t GetState();
};

#endif