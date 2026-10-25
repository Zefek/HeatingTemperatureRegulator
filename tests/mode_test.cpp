#include "../mode.h"
#include <stdio.h>

int failures = 0;
int checks = 0;

#define CHECK(name, actual, expected) \
  do { \
    checks++; \
    if((actual) != (expected)) { \
      failures++; \
      printf("FAIL %s: %s = %d, expected %d (line %d)\n", name, #actual, (int)(actual), (int)(expected), __LINE__); \
    } \
  } while(0)

const uint32_t Delay = 120000UL;

ModeArbiter Started(const char* mode, uint32_t now)
{
  ModeArbiter a;
  ModeInit(a, Delay);
  ModeRequest(a, mode);
  ModeSupervisorStatus(a, "online");
  ModeResolve(a, true, false, now);
  return a;
}

void BootWithoutMessagesIsAutomatic()
{
  ModeArbiter a;
  ModeInit(a, Delay);
  CHECK("boot", ModeResolve(a, false, false, 0), AUTOMATIC);
  CHECK("boot connected", ModeResolve(a, true, false, 1000), AUTOMATIC);
}

void BootRequestedThermostatWaitsForOnline()
{
  ModeArbiter a;
  ModeInit(a, Delay);
  ModeRequest(a, "Thermostat");
  CHECK("no status yet", ModeResolve(a, true, false, 1000), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  CHECK("online arrives", ModeResolve(a, true, false, 1100), THERMOSTAT);
}

void OfflineWithinDelayKeepsThermostat()
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeSupervisorStatus(a, "offline");
  CHECK("offline 1 min", ModeResolve(a, true, false, 10000 + 60000), THERMOSTAT);
  CHECK("offline exactly delay", ModeResolve(a, true, false, 10000 + Delay), THERMOSTAT);
}

void OfflineBeyondDelayFallsBackAndReturns()
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeSupervisorStatus(a, "offline");
  CHECK("offline past delay", ModeResolve(a, true, false, 10000 + Delay + 1), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  CHECK("online again", ModeResolve(a, true, false, 10000 + Delay + 2), THERMOSTAT);
}

void MqttDisconnectBeyondDelayFallsBackAndReturns()
{
  ModeArbiter a = Started("Thermostat", 10000);
  CHECK("disconnected short", ModeResolve(a, false, false, 15000), THERMOSTAT);
  CHECK("disconnected long", ModeResolve(a, false, false, 10000 + Delay + 1), AUTOMATIC);
  CHECK("reconnected", ModeResolve(a, true, false, 10000 + Delay + 5000), THERMOSTAT);
}

void OffStaysOffWhenSupervisorLost()
{
  ModeArbiter a = Started("Off", 10000);
  ModeSupervisorStatus(a, "offline");
  CHECK("off offline", ModeResolve(a, true, false, 10000 + Delay + 1), OFF);
  CHECK("off disconnected", ModeResolve(a, false, false, 10000 + 10 * Delay), OFF);
}

void AutomaticStaysAutomatic()
{
  ModeArbiter a = Started("Automatic", 10000);
  ModeSupervisorStatus(a, "offline");
  CHECK("automatic offline", ModeResolve(a, true, false, 10000 + Delay + 1), AUTOMATIC);
}

void BurningForcesAutomatic()
{
  ModeArbiter a = Started("Thermostat", 10000);
  CHECK("burning thermostat", ModeResolve(a, true, true, 11000), AUTOMATIC);
  CHECK("burn ended", ModeResolve(a, true, false, 12000), THERMOSTAT);
  ModeRequest(a, "Off");
  CHECK("burning off", ModeResolve(a, true, true, 13000), AUTOMATIC);
  CHECK("burn ended off", ModeResolve(a, true, false, 14000), OFF);
}

void ModeChangeDuringBurnAppliesAfter()
{
  ModeArbiter a = Started("Thermostat", 10000);
  CHECK("burning", ModeResolve(a, true, true, 11000), AUTOMATIC);
  ModeRequest(a, "Off");
  CHECK("still burning", ModeResolve(a, true, true, 12000), AUTOMATIC);
  CHECK("after burn", ModeResolve(a, true, false, 13000), OFF);
}

void UnknownPayloadsAreIgnored()
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeRequest(a, "thermostat");
  ModeRequest(a, "");
  ModeRequest(a, "Heat");
  CHECK("unknown mode", ModeResolve(a, true, false, 11000), THERMOSTAT);
  ModeSupervisorStatus(a, "Online");
  ModeSupervisorStatus(a, "");
  CHECK("unknown status keeps online", ModeResolve(a, true, false, 10000 + Delay + 100), THERMOSTAT);
  ModeSupervisorStatus(a, "offline");
  ModeSupervisorStatus(a, "garbage");
  CHECK("unknown status keeps offline", ModeResolve(a, true, false, 10000 + 3 * Delay), AUTOMATIC);
}

uint32_t After(uint32_t base, uint32_t offset)
{
  return (uint32_t)(base + offset);
}

void MillisOverflow()
{
  uint32_t nearWrap = (uint32_t)0xFFFFFFFFUL - 30000UL;
  ModeArbiter a = Started("Thermostat", nearWrap);
  ModeSupervisorStatus(a, "offline");
  CHECK("wrapped", After(nearWrap, 100000UL) < nearWrap, true);
  CHECK("before wrap", ModeResolve(a, true, false, After(nearWrap, 20000UL)), THERMOSTAT);
  CHECK("after wrap within delay", ModeResolve(a, true, false, After(nearWrap, 100000UL)), THERMOSTAT);
  CHECK("after wrap beyond delay", ModeResolve(a, true, false, After(nearWrap, Delay + 1UL)), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  CHECK("online after wrap", ModeResolve(a, true, false, After(nearWrap, Delay + 2UL)), THERMOSTAT);
}

void LongOnlineRefreshesContinuously()
{
  ModeArbiter a = Started("Thermostat", 0);
  uint32_t now = 0;
  for(int i = 0; i < 100; i++)
  {
    now += 60000UL;
    ModeResolve(a, true, false, now);
  }
  ModeSupervisorStatus(a, "offline");
  CHECK("offline after long online", ModeResolve(a, true, false, now + 60000UL), THERMOSTAT);
  CHECK("offline long after", ModeResolve(a, true, false, now + Delay + 1UL), AUTOMATIC);
}

int main()
{
  BootWithoutMessagesIsAutomatic();
  BootRequestedThermostatWaitsForOnline();
  OfflineWithinDelayKeepsThermostat();
  OfflineBeyondDelayFallsBackAndReturns();
  MqttDisconnectBeyondDelayFallsBackAndReturns();
  OffStaysOffWhenSupervisorLost();
  AutomaticStaysAutomatic();
  BurningForcesAutomatic();
  ModeChangeDuringBurnAppliesAfter();
  UnknownPayloadsAreIgnored();
  MillisOverflow();
  LongOnlineRefreshesContinuously();
  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
