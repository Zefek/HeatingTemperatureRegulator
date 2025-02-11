#include "MQTTClient.h"

static bool MQTTClient::pingOutstanding = false;
static void (*MQTTClient::callback)(char* topic, uint8_t* payload, uint16_t plength) = 0;
static uint8_t MQTTClient::connectionState = MQTT_NOTCONNECTED;
static uint8_t MQTTClient::stateChangedToConnected = false;

static void MQTTClient::DataReceived(uint8_t* data, int length)
{
  Serial.println(data[0]&0xF0);
  switch(data[0]&0xF0)
  {
    case MQTTCONNACK: 
      MQTTClient::connectionState = MQTT_CONNECTED;
      MQTTClient::stateChangedToConnected = true;
    break;
    case MQTTPINGRESP: 
      Serial.println("Ping response");
      MQTTClient::pingOutstanding = false;
      Serial.println(MQTTClient::pingOutstanding);
    break;
    case MQTTPUBLISH:
      Serial.println("MQTTPUBLISH");
      uint8_t l = data[1];
      uint16_t tl = (data[2]<<8)+data[3];
      memmove(data+3, data+4, tl);
      data[tl+3] = '\0';
      char* topic = data+3;
      Serial.println(topic);
      uint8_t* payload = data+tl+4;
      callback(topic,payload,l - tl - 2);
      break;
  }
}

MQTTClient::MQTTClient(EspDrv *espDriver, void(*callback)(char* topic, uint8_t* payload, uint16_t plength), void (*connected)())
{
  this->client = espDriver;
  this->client->DataReceived = &DataReceived;
  this->buffer = new uint8_t[bufferSize];
  this->callback = callback;
  this->connected = connected;
}

void MQTTClient::Connect(const char* url, uint16_t port, const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession)
{

  this->client->TCPConnect(url, port);
  this->Login(id, user, pass, willTopic, willQos, willRetain, willMessage, cleanSession);
}

void MQTTClient::Login(const char *id, const char *user, const char *pass, const char* willTopic, uint8_t willQos, boolean willRetain, const char* willMessage, boolean cleanSession)
{
  uint16_t length = MQTT_MAX_HEADER_SIZE;
  unsigned int j;

#if MQTT_VERSION == MQTT_VERSION_3_1
  uint8_t d[9] = {0x00,0x06,'M','Q','I','s','d','p', MQTT_VERSION};
#define MQTT_HEADER_VERSION_LENGTH 9
#elif MQTT_VERSION == MQTT_VERSION_3_1_1
  uint8_t d[7] = {0x00,0x04,'M','Q','T','T',MQTT_VERSION};
#define MQTT_HEADER_VERSION_LENGTH 7
#endif
  for (j = 0;j<MQTT_HEADER_VERSION_LENGTH;j++) {
    this->buffer[length++] = d[j];
  }

  uint8_t v;
  if (willTopic) {
    v = 0x04|(willQos<<3)|(willRetain<<5);
  } else {
    v = 0x00;
  }
  if (cleanSession) {
    v = v|0x02;
  }

  if(user != NULL) {
    v = v|0x80;
    if(pass != NULL) {
      v = v|(0x80>>1);
    }
  }
  this->buffer[length++] = v;

  this->buffer[length++] = ((this->keepAlive) >> 8);
  this->buffer[length++] = ((this->keepAlive) & 0xFF);

  CHECK_STRING_LENGTH(length,id)
  length = WriteString(id,this->buffer,length);
  if (willTopic) {
    CHECK_STRING_LENGTH(length,willTopic)
    length = WriteString(willTopic,this->buffer,length);
    CHECK_STRING_LENGTH(length,willMessage)
    length = WriteString(willMessage,this->buffer,length);
  }

  if(user != NULL) {
    CHECK_STRING_LENGTH(length,user)
    length = WriteString(user,this->buffer,length);
    if(pass != NULL) {
      CHECK_STRING_LENGTH(length,pass)
      length = WriteString(pass,this->buffer,length);
    }
  }
  Write(MQTTCONNECT,this->buffer,length-MQTT_MAX_HEADER_SIZE);
}

uint16_t MQTTClient::WriteString(const char* string, uint8_t* buf, uint16_t pos) {
  const char* idp = string;
  uint16_t i = 0;
  pos += 2;
  while (*idp) {
    buf[pos++] = *idp++;
    i++;
  }
  buf[pos-i-2] = (i >> 8);
  buf[pos-i-1] = (i & 0xFF);
  return pos;
}

void MQTTClient::Subscribe(const char* topic) {
    Subscribe(topic, 0);
}

void MQTTClient::Subscribe(const char *topic, uint8_t qos)
{
  if(MQTTClient::connectionState == MQTT_CONNECTED)
  {
    size_t topicLength = strnlen(topic, this->bufferSize);
    if (topic == 0) 
    {
      return false;
    }
    if (qos > 1) 
    {
      return false;
    }
    if (this->bufferSize < 9 + topicLength) 
    {
      // Too long
      return false;
    }
    // Leave room in the buffer for header and variable length field
    uint16_t length = MQTT_MAX_HEADER_SIZE;
    nextMsgId++;
    if (nextMsgId == 0) 
    {
      nextMsgId = 1;
    }
    this->buffer[length++] = (nextMsgId >> 8);
    this->buffer[length++] = (nextMsgId & 0xFF);
    length = WriteString((char*)topic, this->buffer,length);
    this->buffer[length++] = qos;
    Write(MQTTSUBSCRIBE|MQTTQOS1,this->buffer,length-MQTT_MAX_HEADER_SIZE);
  }
}

void MQTTClient::Publish(const char* topic, const char* payload) 
{
  Publish(topic,(const uint8_t*)payload, payload ? strnlen(payload, this->bufferSize) : 0,false);
}

void MQTTClient::Publish(const char* topic, const char* payload, boolean retained) 
{
  Publish(topic,(const uint8_t*)payload, payload ? strnlen(payload, this->bufferSize) : 0,retained);
}

void MQTTClient::Publish(const char* topic, const uint8_t* payload, unsigned int plength) 
{
  Publish(topic, payload, plength, false);
}

void MQTTClient::Publish(const char* topic, const uint8_t* payload, unsigned int plength, boolean retained)
{
  if (this->bufferSize < MQTT_MAX_HEADER_SIZE + 2+strnlen(topic, this->bufferSize) + plength) 
  {
    // Too long
    return false;
  }
  // Leave room in the buffer for header and variable length field
  uint16_t length = MQTT_MAX_HEADER_SIZE;
  length = WriteString(topic,this->buffer,length);

  // Add payload
  uint16_t i;
  for (i=0;i<plength;i++) {
    this->buffer[length++] = payload[i];
  }

  // Write the header
  uint8_t header = MQTTPUBLISH;
  if (retained) {
    header |= 1;
  }
  Write(header,this->buffer,length-MQTT_MAX_HEADER_SIZE);
  Serial.println("Publish OK");
}

void MQTTClient::Write(uint8_t header, uint8_t* buf, uint16_t length) 
{
  if(MQTTClient::connectionState == MQTT_CONNECTED || header == MQTTCONNECT)
  {
    Serial.println("Write");
    uint16_t rc;
    uint8_t hlen = BuildHeader(header, buf, length);

#ifdef MQTT_MAX_TRANSFER_SIZE
    uint8_t* writeBuf = buf+(MQTT_MAX_HEADER_SIZE-hlen);
    uint16_t bytesRemaining = length+hlen;  //Match the length type
    uint8_t bytesToWrite;
    boolean result = true;
    while((bytesRemaining > 0) && result) 
    {
      bytesToWrite = (bytesRemaining > MQTT_MAX_TRANSFER_SIZE)?MQTT_MAX_TRANSFER_SIZE:bytesRemaining;
      client->Write(writeBuf,bytesToWrite);
      result = (rc == bytesToWrite);
      bytesRemaining -= rc;
      writeBuf += rc;
    }
#else
    client->Write(buf+(MQTT_MAX_HEADER_SIZE-hlen),length+hlen);
    lastOutActivity = millis();
#endif
  }
}

size_t MQTTClient::BuildHeader(uint8_t header, uint8_t* buf, uint16_t length) {
    uint8_t lenBuf[4];
    uint8_t llen = 0;
    uint8_t digit;
    uint8_t pos = 0;
    uint16_t len = length;
    do {

        digit = len  & 127; //digit = len %128
        len >>= 7; //len = len / 128
        if (len > 0) {
            digit |= 0x80;
        }
        lenBuf[pos++] = digit;
        llen++;
    } while(len>0);

    buf[4-llen] = header;
    for (int i=0;i<llen;i++) {
        buf[MQTT_MAX_HEADER_SIZE-llen+i] = lenBuf[i];
    }
    return llen+1; // Full header size is variable length bit plus the 1-byte fixed header
}

void MQTTClient::Loop()
{
  if(MQTTClient::stateChangedToConnected)
  {
    MQTTClient::stateChangedToConnected = false;
    connected();
  }
  unsigned long currentMillis = millis();
  if(currentMillis - lastOutActivity >= keepAlive * 1000 && MQTTClient::connectionState == MQTT_CONNECTED)
  {
    if(MQTTClient::pingOutstanding)
    {
      MQTTClient::connectionState = MQTT_NOTCONNECTED;
      uint8_t* buffer = new uint8_t[2];
      buffer[0] = MQTTDISCONNECT;
      buffer[1] = 0;
      this->client->Write(buffer, 2);
    }
    else
    {
      //Send ping request
      Serial.println("Ping request");
      MQTTClient::pingOutstanding = true;
      lastOutActivity = currentMillis;
      lastInActivity = currentMillis;
      uint8_t* buffer = new uint8_t[2];
      buffer[0] = MQTTPINGREQ;
      buffer[1] = 0;
      this->client->Write(buffer, 2);
    }
  }

  this->client->Loop();
}

uint8_t MQTTClient::GetState()
{
  if(this->client->GetConnectionStatus() == 0)
  {
    return MQTT_NOTCONNECTED;
  }
  return MQTTClient::connectionState;
}