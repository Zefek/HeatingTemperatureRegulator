#include "../mode.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <format>
#include <source_location>
#include <string>
#include <vector>

struct TestResult
{
  std::string name;
  int checks = 0;
  std::vector<std::string> failures;
  std::chrono::microseconds duration{0};
};

struct Checker
{
  int failures = 0;
  int checks = 0;
  std::vector<TestResult> results;
  std::chrono::system_clock::time_point started = std::chrono::system_clock::now();

  void Check(const char* name, int actual, int expected, std::source_location location = std::source_location::current())
  {
    checks++;
    if(!results.empty())
    {
      results.back().checks++;
    }
    if(actual != expected)
    {
      failures++;
      std::string message = std::format("{}: {}, expected {} (tests/mode_test.cpp:{})", name, actual, expected, location.line());
      printf("FAIL %s\n", message.c_str());
      if(!results.empty())
      {
        results.back().failures.push_back(message);
      }
    }
  }

  void Run(const char* name, void (*test)(Checker&))
  {
    results.push_back(TestResult{name});
    auto begin = std::chrono::steady_clock::now();
    test(*this);
    results.back().duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
  }
};

std::string XmlEscape(const std::string& text)
{
  std::string escaped;
  for(char ch : text)
  {
    switch(ch)
    {
      case '&': escaped += "&amp;"; break;
      case '<': escaped += "&lt;"; break;
      case '>': escaped += "&gt;"; break;
      case '"': escaped += "&quot;"; break;
      default: escaped += ch;
    }
  }
  return escaped;
}

std::string TrxDuration(std::chrono::microseconds duration)
{
  long long us = duration.count();
  return std::format("{:02}:{:02}:{:02}.{:07}", us / 3600000000LL, us / 60000000LL % 60, us / 1000000LL % 60, us % 1000000LL * 10);
}

std::string Timestamp(std::chrono::system_clock::time_point point)
{
  time_t seconds = std::chrono::system_clock::to_time_t(point);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", gmtime(&seconds));
  return buffer;
}

std::string Guid(int kind, size_t index)
{
  return std::format("{:08x}-0000-4000-8000-{:012x}", kind, index);
}

void WriteTrx(const Checker& c, const char* path)
{
  FILE* f = fopen(path, "w");
  if(f == nullptr)
  {
    printf("Cannot write %s\n", path);
    return;
  }
  const char* testType = "13cdc9d9-ddb5-4fa4-a97d-d965ccfc6d4b";
  const char* testList = "8c84fa94-04c1-424b-9868-57a2d4851a1d";
  std::string start = Timestamp(c.started);
  std::string finish = Timestamp(std::chrono::system_clock::now());
  size_t passed = 0;
  for(const TestResult& r : c.results)
  {
    passed += r.failures.empty() ? 1 : 0;
  }
  fprintf(f, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
  fprintf(f, "<TestRun id=\"%s\" name=\"mode_test\" xmlns=\"http://microsoft.com/schemas/VisualStudio/TeamTest/2010\">\n", Guid(0, 0).c_str());
  fprintf(f, "  <Times creation=\"%s\" queuing=\"%s\" start=\"%s\" finish=\"%s\" />\n", start.c_str(), start.c_str(), start.c_str(), finish.c_str());
  fprintf(f, "  <Results>\n");
  for(size_t i = 0; i < c.results.size(); i++)
  {
    const TestResult& r = c.results[i];
    fprintf(f, "    <UnitTestResult executionId=\"%s\" testId=\"%s\" testName=\"%s\" computerName=\"ci\" duration=\"%s\" startTime=\"%s\" endTime=\"%s\" testType=\"%s\" outcome=\"%s\" testListId=\"%s\">",
      Guid(2, i).c_str(), Guid(1, i).c_str(), XmlEscape(r.name).c_str(), TrxDuration(r.duration).c_str(), start.c_str(), finish.c_str(),
      testType, r.failures.empty() ? "Passed" : "Failed", testList);
    if(r.failures.empty())
    {
      fprintf(f, "</UnitTestResult>\n");
      continue;
    }
    std::string message;
    for(const std::string& failure : r.failures)
    {
      message += failure + "\n";
    }
    fprintf(f, "\n      <Output><ErrorInfo><Message>%s</Message></ErrorInfo></Output>\n    </UnitTestResult>\n", XmlEscape(message).c_str());
  }
  fprintf(f, "  </Results>\n  <TestDefinitions>\n");
  for(size_t i = 0; i < c.results.size(); i++)
  {
    const std::string name = XmlEscape(c.results[i].name);
    fprintf(f, "    <UnitTest name=\"%s\" storage=\"mode_test\" id=\"%s\"><Execution id=\"%s\" /><TestMethod codeBase=\"mode_test\" adapterTypeName=\"executor://mode_test\" className=\"ModeArbiter\" name=\"%s\" /></UnitTest>\n",
      name.c_str(), Guid(1, i).c_str(), Guid(2, i).c_str(), name.c_str());
  }
  fprintf(f, "  </TestDefinitions>\n  <TestEntries>\n");
  for(size_t i = 0; i < c.results.size(); i++)
  {
    fprintf(f, "    <TestEntry testId=\"%s\" executionId=\"%s\" testListId=\"%s\" />\n", Guid(1, i).c_str(), Guid(2, i).c_str(), testList);
  }
  fprintf(f, "  </TestEntries>\n  <TestLists>\n    <TestList name=\"Results Not in a List\" id=\"%s\" />\n  </TestLists>\n", testList);
  fprintf(f, "  <ResultSummary outcome=\"%s\">\n    <Counters total=\"%zu\" executed=\"%zu\" passed=\"%zu\" failed=\"%zu\" />\n  </ResultSummary>\n</TestRun>\n",
    passed == c.results.size() ? "Completed" : "Failed", c.results.size(), c.results.size(), passed, c.results.size() - passed);
  fclose(f);
}

void WriteSummary(const Checker& c, const char* path)
{
  FILE* f = fopen(path, "a");
  if(f == nullptr)
  {
    return;
  }
  size_t passed = 0;
  for(const TestResult& r : c.results)
  {
    passed += r.failures.empty() ? 1 : 0;
  }
  fprintf(f, "## Unit testy: mode\n\n%s %zu / %zu testů prošlo, %d kontrol, %d chyb\n\n",
    passed == c.results.size() ? "✅" : "❌", passed, c.results.size(), c.checks, c.failures);
  fprintf(f, "| Test | Výsledek | Kontrol | Čas |\n|---|---|---|---|\n");
  for(const TestResult& r : c.results)
  {
    fprintf(f, "| %s | %s | %d | %.3f ms |\n", r.name.c_str(), r.failures.empty() ? "✅" : "❌", r.checks, r.duration.count() / 1000.0);
  }
  for(const TestResult& r : c.results)
  {
    for(const std::string& failure : r.failures)
    {
      fprintf(f, "\n- ❌ **%s**: `%s`", r.name.c_str(), failure.c_str());
    }
  }
  fprintf(f, "\n");
  fclose(f);
}

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

uint32_t After(uint32_t base, uint32_t offset)
{
  return base + offset;
}

void BootWithoutMessagesIsAutomatic(Checker& c)
{
  ModeArbiter a;
  ModeInit(a, Delay);
  c.Check("boot", ModeResolve(a, false, false, 0), AUTOMATIC);
  c.Check("boot connected", ModeResolve(a, true, false, 1000), AUTOMATIC);
}

void BootRequestedThermostatWaitsForOnline(Checker& c)
{
  ModeArbiter a;
  ModeInit(a, Delay);
  ModeRequest(a, "Thermostat");
  c.Check("no status yet", ModeResolve(a, true, false, 1000), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  c.Check("online arrives", ModeResolve(a, true, false, 1100), THERMOSTAT);
}

void OfflineWithinDelayKeepsThermostat(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeSupervisorStatus(a, "offline");
  c.Check("offline 1 min", ModeResolve(a, true, false, 10000 + 60000), THERMOSTAT);
  c.Check("offline exactly delay", ModeResolve(a, true, false, 10000 + Delay), THERMOSTAT);
}

void OfflineBeyondDelayFallsBackAndReturns(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeSupervisorStatus(a, "offline");
  c.Check("offline past delay", ModeResolve(a, true, false, 10000 + Delay + 1), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  c.Check("online again", ModeResolve(a, true, false, 10000 + Delay + 2), THERMOSTAT);
}

void MqttDisconnectBeyondDelayFallsBackAndReturns(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  c.Check("disconnected short", ModeResolve(a, false, false, 15000), THERMOSTAT);
  c.Check("disconnected long", ModeResolve(a, false, false, 10000 + Delay + 1), AUTOMATIC);
  c.Check("reconnected", ModeResolve(a, true, false, 10000 + Delay + 5000), THERMOSTAT);
}

void OffStaysOffWhenSupervisorLost(Checker& c)
{
  ModeArbiter a = Started("Off", 10000);
  ModeSupervisorStatus(a, "offline");
  c.Check("off offline", ModeResolve(a, true, false, 10000 + Delay + 1), OFF);
  c.Check("off disconnected", ModeResolve(a, false, false, 10000 + 10 * Delay), OFF);
}

void AutomaticStaysAutomatic(Checker& c)
{
  ModeArbiter a = Started("Automatic", 10000);
  ModeSupervisorStatus(a, "offline");
  c.Check("automatic offline", ModeResolve(a, true, false, 10000 + Delay + 1), AUTOMATIC);
}

void BurningForcesAutomatic(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  c.Check("burning thermostat", ModeResolve(a, true, true, 11000), AUTOMATIC);
  c.Check("burn ended", ModeResolve(a, true, false, 12000), THERMOSTAT);
  ModeRequest(a, "Off");
  c.Check("burning off", ModeResolve(a, true, true, 13000), AUTOMATIC);
  c.Check("burn ended off", ModeResolve(a, true, false, 14000), OFF);
}

void ModeChangeDuringBurnAppliesAfter(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  c.Check("burning", ModeResolve(a, true, true, 11000), AUTOMATIC);
  ModeRequest(a, "Off");
  c.Check("still burning", ModeResolve(a, true, true, 12000), AUTOMATIC);
  c.Check("after burn", ModeResolve(a, true, false, 13000), OFF);
}

void UnknownPayloadsAreIgnored(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 10000);
  ModeRequest(a, "thermostat");
  ModeRequest(a, "");
  ModeRequest(a, "Heat");
  c.Check("unknown mode", ModeResolve(a, true, false, 11000), THERMOSTAT);
  ModeSupervisorStatus(a, "Online");
  ModeSupervisorStatus(a, "");
  c.Check("unknown status keeps online", ModeResolve(a, true, false, 10000 + Delay + 100), THERMOSTAT);
  ModeSupervisorStatus(a, "offline");
  ModeSupervisorStatus(a, "garbage");
  c.Check("unknown status keeps offline", ModeResolve(a, true, false, 10000 + 3 * Delay), AUTOMATIC);
}

void MillisOverflow(Checker& c)
{
  uint32_t nearWrap = UINT32_MAX - 30000UL;
  ModeArbiter a = Started("Thermostat", nearWrap);
  ModeSupervisorStatus(a, "offline");
  c.Check("wrapped", After(nearWrap, 100000UL) < nearWrap, true);
  c.Check("before wrap", ModeResolve(a, true, false, After(nearWrap, 20000UL)), THERMOSTAT);
  c.Check("after wrap within delay", ModeResolve(a, true, false, After(nearWrap, 100000UL)), THERMOSTAT);
  c.Check("after wrap beyond delay", ModeResolve(a, true, false, After(nearWrap, Delay + 1UL)), AUTOMATIC);
  ModeSupervisorStatus(a, "online");
  c.Check("online after wrap", ModeResolve(a, true, false, After(nearWrap, Delay + 2UL)), THERMOSTAT);
}

void LongOnlineRefreshesContinuously(Checker& c)
{
  ModeArbiter a = Started("Thermostat", 0);
  uint32_t now = 0;
  for(int i = 0; i < 100; i++)
  {
    now += 60000UL;
    ModeResolve(a, true, false, now);
  }
  ModeSupervisorStatus(a, "offline");
  c.Check("offline after long online", ModeResolve(a, true, false, now + 60000UL), THERMOSTAT);
  c.Check("offline long after", ModeResolve(a, true, false, now + Delay + 1UL), AUTOMATIC);
}

int main(int argc, char** argv)
{
  Checker c;
  c.Run("BootWithoutMessagesIsAutomatic", BootWithoutMessagesIsAutomatic);
  c.Run("BootRequestedThermostatWaitsForOnline", BootRequestedThermostatWaitsForOnline);
  c.Run("OfflineWithinDelayKeepsThermostat", OfflineWithinDelayKeepsThermostat);
  c.Run("OfflineBeyondDelayFallsBackAndReturns", OfflineBeyondDelayFallsBackAndReturns);
  c.Run("MqttDisconnectBeyondDelayFallsBackAndReturns", MqttDisconnectBeyondDelayFallsBackAndReturns);
  c.Run("OffStaysOffWhenSupervisorLost", OffStaysOffWhenSupervisorLost);
  c.Run("AutomaticStaysAutomatic", AutomaticStaysAutomatic);
  c.Run("BurningForcesAutomatic", BurningForcesAutomatic);
  c.Run("ModeChangeDuringBurnAppliesAfter", ModeChangeDuringBurnAppliesAfter);
  c.Run("UnknownPayloadsAreIgnored", UnknownPayloadsAreIgnored);
  c.Run("MillisOverflow", MillisOverflow);
  c.Run("LongOnlineRefreshesContinuously", LongOnlineRefreshesContinuously);
  printf("%zu tests, %d checks, %d failures\n", c.results.size(), c.checks, c.failures);
  for(int i = 1; i + 1 < argc; i++)
  {
    if(std::string(argv[i]) == "--trx")
    {
      WriteTrx(c, argv[i + 1]);
    }
  }
  if(const char* summary = getenv("GITHUB_STEP_SUMMARY"))
  {
    WriteSummary(c, summary);
  }
  return c.failures == 0 ? 0 : 1;
}
