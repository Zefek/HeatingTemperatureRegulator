#include "EspDrv.h"
#include <Arduino.h>

const char* ESPTAGS[] = {
  "OK",
  "ERROR",
  "FAIL",
  "SEND OK",
  ">"
};

EspDrv::EspDrv(Stream* serial) {
  this->serial = serial;
  this->buffer = new uint8_t[32];
  //this->DataReceived = dataReceived;
}

void EspDrv::Loop() 
{
  
  while (this->serial->available()) 
  {
    if (this->state == ESPREADSTATE_DATA_LENGTH1) 
    {
      char c = this->serial->read();
      Serial.print(c);
      if (c == ':') 
      {
        buffer[bufLength++] = '\0';
        sscanf(buffer, "%d", &receivedDataLength);
        bufLength = 0;
        dataRead = 0;
        Serial.print("Data length ");
        Serial.println(receivedDataLength);
        this->receivedDataBuffer = new uint8_t[receivedDataLength];
        this->state = ESPREADSTATE_DATA2;
        startDataReadMillis = millis();
      } 
      else 
      {
        this->buffer[bufLength++] = c;
      }
    } 
    else if (this->state == ESPREADSTATE_DATA2) 
    {
      uint8_t r = this->serial->read();
      receivedDataBuffer[dataRead++] = r;
      if (dataRead == receivedDataLength) 
      {
        Serial.println("Data read ok");
        this->state = ESPREADSTATE1;
        DataReceived(receivedDataBuffer, dataLength);
        delete(receivedDataBuffer);
      }
      if(millis() - startDataReadMillis > 5000)
      {
        Serial.println("Data read error");
        this->state = ESPREADSTATE1;
        delete(receivedDataBuffer);
      }
    } 
    else if (this->state == ESPREADSTATE2) 
    {
      if (this->serial->read() == '\n') 
      {
        this->state = ESPREADSTATE1;
        this->bufLength = 0;
      }
    } 
    else 
    {
      char c = this->serial->read();
      Serial.print(c);
      if (c == '\r') 
      {
        this->state = ESPREADSTATE2;
        continue;
      }
      this->buffer[bufLength++] = c;
      this->buffer[bufLength]='\0';
      for (int i = 0; i < 5; i++) 
      {
        int compare = strncmp(ESPTAGS[i], this->buffer, strlen(ESPTAGS[i]));
        //Serial.println(compare);
        if (compare == 0) 
        {
          TagReceived(ESPTAGS[i]);
          bufLength = 0;
          break;
        } 
        else if (strncmp("+IPD,", this->buffer, strlen("+IPD,")) == 0) 
        {
          //Serial.println("IPD");
          this->state = ESPREADSTATE_DATA_LENGTH1;
          bufLength = 0;
        }
        else if (strncmp("CLOSED", this->buffer, strlen("CLOSED")) == 0) 
        {
          connectionState = ESP_NOTCONNECTED;
          bufLength = 0;
        }
      }
    } 
  }
}

int EspDrv::Connect(const char* ssid, const char* password) {
  this->SendCmd(F("ATE0"));
  WaitForTag("OK", 10000);
  this->tag = "";
  delay(1000);
  this->SendCmd(F("AT+CWJAP_CUR=\"%s\",\"%s\""), ssid, password);
  WaitForTag("OK", 10000);
  this->tag = "";
  delay(1000);
  connectionState = ESP_CONNECTED;
  return 0;
}

int EspDrv::TCPConnect(const char* url, int port) {
  this->SendCmd(F("AT+CIPSTART=\"TCP\",\"%s\",%d"), url, port);
  WaitForTag("OK", 10000);
  this->tag = "";
  return 0;
}

void EspDrv::Write(uint8_t* data, uint16_t length) {
  this->SendCmd(F("AT+CIPSEND=%d"), length);
  WaitForTag(">", 1000);
  this->data = data;
  this->dataLength = length;
  Serial.println("Sending data");
  SendData();
  WaitForTag("SEND OK", 1000);
  delay(1000);
}

void EspDrv::SendData() {
  this->serial->write(this->data, this->dataLength);
}

void EspDrv::SendCmd(const __FlashStringHelper* cmd, ...) {
  char cmdBuf[CMD_BUFFER_SIZE];
  va_list args;
  va_start(args, cmd);
  vsnprintf_P(cmdBuf, CMD_BUFFER_SIZE, (char*)cmd, args);
  va_end(args);
  Serial.println(cmdBuf);
  this->serial->println(cmdBuf);
}

bool EspDrv::WaitForTag(const char* pTag, unsigned long timeout) 
{
  Serial.print("Wait For Tag ");
  unsigned long m = millis();
  while (strncmp(this->tag, pTag, strlen(pTag)) != 0 && millis() - m < timeout) 
  {
    this->Loop();
  }
  bool result = strncmp(this->tag, pTag, strlen(pTag)) == 0;
  this->tag = "";
  return result;
}

void EspDrv::TagReceived(const char* pTag) {
  Serial.print("Tag received ");
  Serial.println(pTag);
  this->tag = pTag;
}

uint8_t EspDrv::GetConnectionStatus()
{
  return connectionState;
}
