# Hardware

## Typ desky a modulů

- **Arduino Mega 2560** se sensor shieldem
- **ESP-01 (ESP8266)** s napájecím adaptérem 3.3/5V — [hadex.cz M431A](https://www.hadex.cz/m431a-esp8266---adapter-pro-wifi-modul-esp-01-napajeni-33-i-5v/)
- **USB adaptér pro ESP-01** (pro konfiguraci AT firmware) — [hadex.cz M431](https://www.hadex.cz/m431-esp8266---usb-adapter-pro-esp-01/)

## BOM (Bill of Materials)

| Součástka | Počet | Poznámka |
|-----------|-------|----------|
| Arduino Mega 2560 | 1 | Hlavní řídící deska |
| Sensor shield pro Mega 2560 | 1 | Zjednodušení zapojení pinů |
| ESP-01 (ESP8266) | 1 | WiFi modul, AT firmware @ 57600 baud |
| Adaptér pro ESP-01 (3.3/5V) | 1 | Napájení a level-shifting pro Mega |
| USB adaptér pro ESP-01 | 1 | Pouze pro konfiguraci AT firmware |
| DS1302 RTC modul | 1 | Hodiny reálného času |
| I2C LCD 16×2 | 1 | Displej, adresa 0x27 |
| SRX882S 433 MHz přijímač | 1 | Příjem dat z bezdrátových teplotních čidel |
| TX07K-TXC bezdrátový teplotní senzor | 1+ | Venkovní / pokojová teplota |
| DS18B20 vodotěsná sonda | 9 | Teploty akumulace (4×), výstup akumulace, kotel, výstup topení, vratka topení, bojler |
| PT1000 teplotní sonda | 1 | Teplota spalin |
| Rezistor 10 kΩ | 1 | Dělič napětí pro PT1000 |
| Rezistor 4.7 kΩ | 1 | Pull-up pro OneWire sběrnici DS18B20 |
| Deska 4× SSR relé | 1 | Ovládání ventilu (2×), čerpadla, termostatu spalin |
| BEL wattmetr | 1 | **Volitelný** — monitoring FVE přes Serial2 @ 9600 baud |

## Pinout

| Pin | Směr | Funkce | Modul |
|-----|------|--------|-------|
| 2 | INPUT | DATA přijímač | SRX882S (433 MHz) |
| 3 | INPUT | Interrupt přijímač | SRX882S (TX07K-TXC knihovna) |
| 4 | OUTPUT | CE | DS1302 RTC |
| 5 | I/O | IO (data) | DS1302 RTC |
| 6 | OUTPUT | SCLK | DS1302 RTC |
| 7 | I/O | OneWire bus | DS18B20 (9 čidel) |
| 8 | OUTPUT | Relé — ventil více topení | SSR relé (aktivní LOW) |
| 9 | OUTPUT | Relé — ventil méně topení | SSR relé (aktivní LOW) |
| 10 | OUTPUT | Relé — čerpadlo topení | SSR relé (aktivní LOW) |
| 11 | OUTPUT | Relé — termostat spalin | SSR relé (aktivní LOW) |
| A0 | INPUT | Analogový vstup PT1000 | PT1000 + dělič 10 kΩ |
| SDA (20) | I/O | I2C data | LCD 16×2 |
| SCL (21) | I/O | I2C clock | LCD 16×2 |
| TX1/RX1 (18/19) | I/O | Serial1 @ 57600 | ESP-01 (WiFi) |
| TX2/RX2 (16/17) | I/O | Serial2 @ 9600 | BEL wattmetr (FVE) — volitelný |

> Všechna SSR relé jsou spínaná aktivní úrovní LOW (`digitalWrite(pin, LOW)` = sepnuto).

## Schéma zapojení

Blokové schéma regulátoru:

![heater_regulator](https://github.com/user-attachments/assets/4e85691c-065a-4078-b046-f0943ee74d2e)

Rozložení teplotních čidel:

![sensors](https://github.com/user-attachments/assets/df01f1ed-c163-43c2-801e-ac2c6a9aa728)

## Zapojení DS18B20

- Všech 9 čidel je na společné OneWire sběrnici na **pinu 7**
- Pull-up rezistor **4.7 kΩ** mezi DATA a VCC
- Každé čidlo má unikátní 8-bajtovou adresu — adresy se konfigurují v `sensorconfig.h`
- Čidla: akumulace 1–4, výstup akumulace, kotel, výstup topení, vratka topení, bojler

## Zapojení relé

4× SSR relé deska připojená na piny 8–11:

| Pin | Relé | Ovládá |
|-----|------|--------|
| 8 | SSR 1 | Třícestný ventil — směr VÍCE topení |
| 9 | SSR 2 | Třícestný ventil — směr MÉNĚ topení |
| 10 | SSR 3 | Oběhové čerpadlo topného okruhu |
| 11 | SSR 4 | Termostat kotle na odpadní plyn |

Relé jsou aktivní LOW — v klidovém stavu jsou piny nastaveny na HIGH.

## Zapojení PT1000 (spaliny)

PT1000 sonda je zapojena jako napěťový dělič s referenčním rezistorem 10 kΩ na **pin A0**:

```
VCC ──[PT1000]──┬──[10kΩ]── GND
                │
               A0
```

Firmware převádí ADC hodnotu na teplotu pomocí Callendar–Van Dusen aproximace.

## Zapojení RTC DS1302

| DS1302 pin | Arduino pin |
|------------|-------------|
| CE | 4 |
| IO (data) | 5 |
| SCLK | 6 |
| VCC | 5V |
| GND | GND |

Čas se synchronizuje přes MQTT topic `TOPIC_CURRENTDATETIEM`.

## Zapojení LCD I2C

| LCD pin | Arduino pin |
|---------|-------------|
| SDA | 20 (SDA) |
| SCL | 21 (SCL) |
| VCC | 5V |
| GND | GND |

- I2C adresa: **0x27**
- Rozlišení: **16×2** znaků

## Zapojení WiFi modulu (ESP8266)

ESP-01 je připojený přes napájecí adaptér (M431A) na **Serial1** (piny 18/19):

| ESP-01 adaptér | Arduino Mega |
|-----------------|-------------|
| TX | RX1 (pin 19) |
| RX | TX1 (pin 18) |
| VCC | 5V (adaptér konvertuje na 3.3V) |
| GND | GND |

- Baud rate: **57600** (nastavit na ESP příkazem `AT+UART_DEF=57600,8,1,0,0`)
- Komunikace přes AT příkazy pomocí knihovny EspDrv

## Zapojení SRX882S (bezdrátový přijímač)

| SRX882S pin | Arduino pin |
|-------------|-------------|
| DATA | 2 |
| (interrupt) | 3 |
| VCC | 5V |
| GND | GND |

Přijímá data z bezdrátových čidel TX07K-TXC na frekvenci 433 MHz. Data jsou dekódována knihovnou TX07K-TXC a přeposílána přes Serial do aplikace [ArduinoSerialReader](https://github.com/Zefek/ArduinoSerialReader).
