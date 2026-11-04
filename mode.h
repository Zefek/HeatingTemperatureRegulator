#ifndef MODE_H
#define MODE_H

#include <stdint.h>

enum HeatingMode : uint8_t {
  OFF = 0,
  AUTOMATIC = 1,
  THERMOSTAT = 2
};

struct ModeArbiter {
  HeatingMode requested;
  bool supervisorOnline;
  bool supervisorSeen;
  uint32_t lastSupervisorOk;
  uint32_t lostDelay;
};

void ModeInit(ModeArbiter& arbiter, uint32_t lostDelay);
void ModeRequest(ModeArbiter& arbiter, const char* payload);
void ModeSupervisorStatus(ModeArbiter& arbiter, const char* payload);
HeatingMode ModeResolve(ModeArbiter& arbiter, bool mqttConnected, bool burning, uint32_t now);

#endif
