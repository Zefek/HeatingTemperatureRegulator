# Parametry regulátoru (config.h)

## WiFi a MQTT

| Parametr | Popis |
|----------|-------|
| `WifiSSID` | SSID WiFi sítě |
| `WifiPassword` | Heslo WiFi sítě |
| `MQTTUsername` | Uživatelské jméno MQTT brokeru |
| `MQTTPassword` | Heslo MQTT brokeru |
| `MQTTHost` | IP adresa nebo hostname MQTT brokeru |

## MQTT topicy

| Parametr | Směr | Popis |
|----------|------|-------|
| `TOPIC_THERMOSTAT` | IN | Stav termostatu z HomeAssistant (`ON` / `OFF`) |
| `TOPIC_MODE` | IN | Režim regulátoru (`Off` / `Automatic` / `Thermostat`) |
| `TOPIC_ZEROPOINT` | IN | Parametry topné křivky ve formátu `zeroPoint;exponent` |
| `TOPIC_CURRENTDATETIEM` | IN | Synchronizace času ve formátu `YYYY-MM-DD HH:MM:SS` |
| `TOPIC_THERMOSTATSETCHANGED` | IN | Požadovaná vnitřní teplota z termostatu (float) |
| `TOPIC_OUTSIDETEMPERATURE` | OUT | Surová data z venkovního čidla (hex-encoded 5 bajtů) |
| `TOPIC_HEATERSTATE` | OUT | Binární paket `HeaterState` (22 bajtů) se stavem regulátoru |
| `TOPIC_FVE` | OUT | **Volitelný** — binární paket `FVEData` (16 bajtů) s daty z wattmetru |
| `TOPIC_FVE_STATE` | OUT | **Volitelný** — stav FVE (`Online` / `Offline`) |

> U všech IN topiců je důležité nastavit MQTT **retain**, aby se po restartu regulátoru obnovil původní stav.

## Parametry regulace

| Parametr | Typ | Popis | Kdy měnit |
|----------|-----|-------|-----------|
| `MAXTEMPDIFFERENCE` | int | Maximální rozdíl mezi požadovanou teplotou a vratkou pro udržení teplotního spádu [°C] | Při změně topného systému (podlahové vs. radiátorové) |
| `PCONST` | int | P konstanta PD regulátoru — ovlivňuje sílu reakce na odchylku teploty | Při ladění rychlosti regulace |
| `DCONST` | long | D konstanta PD regulátoru — tlumí kmitání, reaguje na rychlost změny teploty | Při ladění stability (oscilace ventilu) |
| `TEMPCHECKINTERVAL` | int | Vzorkovací interval PD regulátoru [ms] | Obvykle neměnit (závisí na setrvačnosti systému) |
| `AUTOMATICTEMPERATURE` | int | Teplota kotle [°C], nad kterou se přepne z THERMOSTAT na plně ekvitermní (AUTOMATIC) režim | Podle parametrů kotle |
| `HEATEROVERHEATINGTEMPERATURE` | int | Teplota kotle [°C], při které se aktivuje režim přetopení (nucený odvod tepla) | Bezpečnostní limit kotle |
| `OVERHEATINGTEMPERATURE` | int | Požadovaná teplota topné vody [°C] při přetopení — nastaví se místo ekvitermního výpočtu | Maximální bezpečná teplota pro radiátory |
| `MININPUTTEMPERATURE` | int | Minimální teplota na výstupu akumulace [°C] pro povolení topení | Práh, pod kterým nemá smysl topit (akumulace vybitá) |
| `MAXIMALSERVOSTEP` | int | Maximální krok ventilu za jeden cyklus PD regulátoru [%] | Omezení agresivity regulace |

## Parametry detekce kotle (spalinový termostat)

| Parametr | Typ | Popis | Kdy měnit |
|----------|-----|-------|-----------|
| `HeaterOn(ht, wgt)` | makro | Podmínka pro detekci běžícího kotle — `ht` = teplota kotle, `wgt` = teplota spalin | Podle parametrů konkrétního kotle |
| `HeaterOff(ht, wgt)` | makro | Podmínka pro detekci vypnutého kotle | Podle parametrů konkrétního kotle |
| `heaterStartTimeout` | unsigned long | Timeout [ms] po aktivaci spalinového termostatu — pokud kotel nenastartuje, termostat se vypne | Podle doby náběhu kotle |
| `wasteGasMinGradient` | float | Minimální gradient teploty spalin [°C/min] pro detekci rozehřívání kotle | Podle charakteristiky kotle |
| `wasteGasMaxGradient` | float | Maximální gradient teploty spalin [°C/min] — filtruje falešné špičky | Podle charakteristiky kotle |
| `heaterStartTryCount` | short | Počet po sobě jdoucích cyklů s pozitivním gradientem před aktivací termostatu | Zvýšit při falešných detekcích |

## Proměnné ekvitermní křivky

| Proměnná | Výchozí | Popis |
|----------|---------|-------|
| `equithermalCurveZeroPoint` | 40 | Cílová teplota topné vody při venkovní teplotě 0°C [°C] — posouvá celou křivku |
| `insideTemperature` | 22.5 | Referenční vnitřní teplota [°C] — fallback při výpadku MQTT |
| `exponent` | 0.76923 | Exponent topné křivky — strmost nelineární závislosti (rozsah 0.6–1.0) |

> Tyto proměnné se za běhu aktualizují přes MQTT z HomeAssistant. Výchozí hodnoty platí pouze při výpadku komunikace.
