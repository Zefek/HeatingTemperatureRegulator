Regulátor slouží k řízení topného systému s akumulační nádrží a venkovním čidlem. Cílem je udržovat požadovanou vnitřní teplotu v objektu efektivním ohřevem na základě aktuálních podmínek — venkovní teploty, stavu akumulace a případného požadavku z termostatu. Regulace topení využívá hybridní režim, kdy při teplotě kotle nad 80°C je plně ekvitermní režim (AUTOMATIC) a řízení topení není ovlivňováno stavem termostat. Při poklesu teploty kotle pod 76°C je režim přepnutý na THERMOSTAT a řízení topení je ovlivňováno stavem termostatu. Zároveň na základě změny vnitřní teploty a požadované teploty nastavené na termostatu dochází ke změně kompenzace a nastavování sklonu topné křivky. 

## Základní režimy provozu
Regulátor může běžet ve třech režimech:

- VYPNUTO (OFF)
Topení je zcela deaktivováno.

- AUTOMATICKÝ REŽIM
Teplota výstupní vody se vypočítává automaticky podle průměrné venkovní teploty a zadané referenční teploty (tzv. ekvitermní regulace).

- TERMOSTATICKÝ REŽIM
Regulace se řídí externím termostatem, který přes MQTT posílá signál pro zapnutí nebo vypnutí topení.

Režim je možné přepínat vzdáleně přes MQTT zprávy.

## Řízení ohřevu
Topný systém se aktivuje, pokud:

je systém v režimu AUTOMATIC nebo THERMOSTAT a zároveň je dosaženo požadovaných vstupních podmínek.
- v režimu AUTOMATIC je teplota na výstupu akumulace vyšší než 30°C,
- v režimu THERMOSTAT je teplota na výstupu akumulace vyšší než 30°C a zároveň je požadavek na topení z externího termostatu.

Relé ovládají:
- směr ventilu (zvyšování nebo snižování topné vody),
- čerpadlo topného okruhu.

V případě ztráty komunikace s MQTT se nastavuje režim AUTOMATIC. U MQTT zpráv je důležité nastavit parametr Retention, aby v případě opětovného připojení regulátoru na MQTT mohlo dojít k nastavení původního režimu. 

## Ekvitermní regulace
Ekvitermní regulace upravuje teplotu topné vody podle venkovní teploty. Využívá výpočtovou křivku, která má dva nastavitelné parametry:
- požadovaná výstupní teplota topné vody při venkovní teplotě 0 °C. Tato hodnota posouvá celou topnou křivku nahoru nebo dolů.
- Exponent – určuje nelineární sklon křivky (strmost závislosti).

### Rovnice ekvitermní křivky
$$
\[
T_{\text{výstup}} = T_{\text{vnitřní}} + (T_{\text{zero}} - T_{\text{vnitřní}}) \cdot \left( \frac{T_{\text{venkovní}} - T_{\text{vnitřní}}}{-T_{\text{vnitřní}}} \right)^{exp}
\]
$$
### Proměnné:
| Symbol | Význam |
|--------|--------|
| $\( T_{\text{výstup}} \)$     | Výpočtem určená požadovaná výstupní teplota topné vody |
| $\( T_{\text{vnitřní}} \)$    | Referenční vnitřní teplota (např. 21 °C) |
| $\( T_{\text{venkovní}} \)$  | Průměrná venkovní teplota (např. za 3 hodiny) |
| $\( T_{\text{zero}} \)$      | Nulový bod – cílová teplota vody při venkovní teplotě 0 °C |
| \( exp \)              | Exponent určující strmost křivky (neměnná konstanta) |

V případě výpadku připojení na MQTT se nastavují výchozí hodnoty
- Vnitřní teplota - 22,5C
- zero - 40
- exp - 0,76923

## Regulace pomocí vnitřní teploty v místnosti
Vnitřní teplota v místnosti je snímána stejným teplotním snímačem jako venkovní teplota v regulátoru - TX07K-TXC. Hodnoty z tohoto snímače jsou přečteny pomocí Arduino a SRX882S a po sériovém rozhraní jsou přenášeny do aplikace, která na základě kombinace ID snímače a kanálu, na kterém snímač vysílá identifikuje místnost. Tyto informace jsou pak odeslány do MQTT. Aplikace je dostupná v [repozitáři](https://github.com/Zefek/ArduinoSerialReader). Pro čtení snímačů je použita [knihovna](https://github.com/Zefek/TX07K-TXC) a její výstup je jen zapisován na sériový výstup Arduino pomocí Serial.write(...). 

Na MQTT je připojený HomeAssistant, do kterého je integrován jak samotný regulátor, tak teplotní snímače z jednotlivých místností. V HomeAssistant přes Pomocníka kombinace několika senzorů je hodnota z jednotlivých místností průměrována na referenční teplotu v celém bytě. Tato teplota je pak použita jako vstup do diferečního senzoru, pomocí kterého je řešena hodinová předpověď pohybu teploty. Součtem referenční teploty a výstupu diferenčního senzoru je předpovídaná hodnota teploty. 

![image](https://github.com/user-attachments/assets/27e6ca47-70c2-404f-90a7-86dbbde6d4fb)

Tato předpovídaná hodnota (vnitřní teplota) je pak vstupem do termostatu (dostupný v HomeAssistant). 
![image](https://github.com/user-attachments/assets/a884c9d2-8f4d-4b6e-aeea-51a9df0cf887)

Stav termostatu (ON/OFF) se pak posílá přes MQTT do regulátoru. Pokud je regulátor v režimu THERMOSTAT a v akumulaci je dostatečné množství energie, regulátor dle požadavku termostatu topení vypíná nebo zapíná. 

Na základě rozdílu vnitřní teploty a teploty nastavené na termostatu (požadovaná teplota) se zároveň nastavuje kompenzace - teplotní křivka se nastavuje v rozsahu +-3. 
```
- name: "Heating curve offset"
        unique_id: "HCO"
        unit_of_measurement: "°C"
        device_class: temperature
        state: >
          {% set target = state_attr('climate.termostat', 'temperature') | default(21) | float %}
          {% set current = states('sensor.vnitrni_teplota') | float %}
          {% set delta = target - current %}
          {{ [[(delta * 2.5), 3] | min, -3] | max | round(1) }}
```
Na základě změny rychlosti vnitřní teploty se pak nastavuje sklon topné křivky pomocí parametru exp v rozsahu 0,6 - 1. 
```
- name: "Heating curve exponent"
        unique_id: "HCE"
        state: >
          {% set base_exp = 0.76923 %}
          {% set deriv = states('sensor.teplotni_trend') | float %}
          {{ [[(base_exp + deriv * -0.2), 1] | min, 0.6] | max | round(5) }}
```

Hodnoty senzorů se poté odešlo do MQTT a regulátor upraví teplotu topné vody na základě těchto teplot. 
```
automation:
  - alias: "Send heating curve offset to MQTT"
    id: "send_heating_offset_mqtt"
    trigger:
      - platform: state
        entity_id: sensor.heating_curve_offset
    action:
      - service: mqtt.publish
        data:
          topic: "home/heating/offset"
          payload: "{{ states('sensor.heating_curve_offset') }}"
          retain: true
```

## Blokové schéma zapojení
![heater_regulator](https://github.com/user-attachments/assets/4e85691c-065a-4078-b046-f0943ee74d2e)

Na pinu 7 jsou DS18B20 snímače pro snímání teplot kotle, akumulace a bojleru. Snímač pro teplotu spalin je PT1000. 
Pro zprovoznění regulátoru je potřeba soubor config.h
```
//SSID Wifi pro připojení k wifi
#define WifiSSID "WifiSSID"
//Heslo k wifi
#define WifiPassword "WifiPassword"

//Uživatelské jméno pro MQTT
#define MQTTUsername "MQTTUserName"
//Heslo pro MQTT
#define MQTTPassword "MQTTPassword"
//Host MQTT
#define MQTTHost "MQTTHost"

//MQTT Topic pro ovládání termostatem (ON/OFF)
#define TOPIC_THERMOSTAT "TOPIC_THERMOSTAT"
//MQTT Topic pro nastavení regulátoru (Off - Vypnuto, Automatic - topí dokud je akumulace nad 30°C, Thermostat - ovládání termostatem)
#define TOPIC_MODE "TOPIC_MODE"
//MQTT Topic pro nastavení topné křivky
#define TOPIC_ZEROPOINT "TOPIC_ZEROPOINT"
//MQTT Topic pro synchronizaci data a času
#define TOPIC_CURRENTDATETIEM "TOPIC_CURRENTDATETIEM"
//MQTT Topic pro nastavení požadované vnitřní teploty (ovlivňuje topnou křivku)
#define TOPIC_THERMOSTATSETCHANGED "TOPIC_THERMOSTATSETCHANGED"
//MQTT Topic pro odeslání dat o venkovní teplotě
#define TOPIC_OUTSIDETEMPERATURE "TOPIC_OUTSIDETEMPERATURE"
//MQTT Topic pro odeslání stavu regulátoru
#define TOPIC_HEATERSTATE "TOPIC_HEATERSTATE"
//MQTT Topic pro odeslání stavu BEL (FVE)
#define TOPIC_FVE "TOPIC_FVE"
```
A také soubor sensorconfig.h
```
DeviceAddress acumulatorFirst =    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorSecond =   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorThird =    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorFour =     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress acumulatorOutput =   { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress boiler =             { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress currentHeating =     { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress returnHeating =      { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
DeviceAddress heaterTemperature =  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
```
