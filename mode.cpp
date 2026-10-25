#include "mode.h"
#include <string.h>

void ModeInit(ModeArbiter& arbiter, uint32_t lostDelay)
{
  arbiter.requested = AUTOMATIC;
  arbiter.supervisorOnline = false;
  arbiter.supervisorSeen = false;
  arbiter.lastSupervisorOk = 0;
  arbiter.lostDelay = lostDelay;
}

void ModeRequest(ModeArbiter& arbiter, const char* payload)
{
  if(strcmp(payload, "Off") == 0)
  {
    arbiter.requested = OFF;
  }
  else if(strcmp(payload, "Automatic") == 0)
  {
    arbiter.requested = AUTOMATIC;
  }
  else if(strcmp(payload, "Thermostat") == 0)
  {
    arbiter.requested = THERMOSTAT;
  }
}

void ModeSupervisorStatus(ModeArbiter& arbiter, const char* payload)
{
  if(strcmp(payload, "online") == 0)
  {
    arbiter.supervisorOnline = true;
  }
  else if(strcmp(payload, "offline") == 0)
  {
    arbiter.supervisorOnline = false;
  }
}

HeatingMode ModeResolve(ModeArbiter& arbiter, bool mqttConnected, bool burning, uint32_t now)
{
  if(mqttConnected && arbiter.supervisorOnline)
  {
    arbiter.lastSupervisorOk = now;
    arbiter.supervisorSeen = true;
  }
  if(burning)
  {
    return AUTOMATIC;
  }
  bool supervisorLost = !arbiter.supervisorSeen || (uint32_t)(now - arbiter.lastSupervisorOk) > arbiter.lostDelay;
  if(arbiter.requested == THERMOSTAT && supervisorLost)
  {
    return AUTOMATIC;
  }
  return arbiter.requested;
}
