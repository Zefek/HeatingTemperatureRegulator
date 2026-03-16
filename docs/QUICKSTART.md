# Quickstart — od součástek k funkčnímu regulátoru

## Prerekvizity

- Arduino IDE 1.8+ nebo Arduino IDE 2.x
- Arduino Mega 2560 se sensor shieldem
- Všechny součástky dle [BOM](HARDWARE.md#bom-bill-of-materials)
- MQTT broker (např. Mosquitto) běžící v síti
- USB kabel pro Arduino Mega (USB-B)
- USB adaptér pro ESP-01 ([M431](https://www.hadex.cz/m431-esp8266---usb-adapter-pro-esp-01/)) pro počáteční konfiguraci

## 1. Instalace Arduino knihoven

Všechny knihovny jsou dostupné přes Arduino Library Manager (Sketch → Include Library → Manage Libraries). Nainstalujte:

- **DallasTemperature** (nainstaluje i závislost **OneWire**)
- **Ds1302**
- **LiquidCrystal_I2C**
- **TX07K-TXC**
- **MQTTESP8266** (obsahuje MQTTClient i EspDrv)

Viz kompletní přehled v [LIBRARIES.md](LIBRARIES.md).

## 2. Sestavení hardware

Zapojení součástek dle [HARDWARE.md](HARDWARE.md):
1. Nasadit sensor shield na Arduino Mega
2. Připojit ESP-01 přes adaptér na Serial1 (piny 18/19)
3. Připojit DS18B20 čidla na pin 7 s pull-up rezistorem 4.7 kΩ
4. Připojit PT1000 s děličem 10 kΩ na A0
5. Připojit 4× SSR relé desku na piny 8–11
6. Připojit DS1302 RTC na piny 4, 5, 6
7. Připojit I2C LCD na SDA/SCL (piny 20/21)
8. Připojit SRX882S přijímač na piny 2, 3

## 3. Zjištění adres DS18B20

### Nahrání discovery sketche

Vytvořte nový sketch s tímto kódem a nahrajte ho na Mega:

```cpp
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(7);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(57600);
  sensors.begin();
  Serial.print("Nalezeno cidel: ");
  Serial.println(sensors.getDeviceCount());

  DeviceAddress addr;
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    if (sensors.getAddress(addr, i)) {
      Serial.print("Cidlo ");
      Serial.print(i);
      Serial.print(": { ");
      for (int j = 0; j < 8; j++) {
        Serial.print("0x");
        if (addr[j] < 0x10) Serial.print("0");
        Serial.print(addr[j], HEX);
        if (j < 7) Serial.print(", ");
      }
      Serial.println(" }");
    }
  }
}

void loop() {
  sensors.requestTemperatures();
  for (int i = 0; i < sensors.getDeviceCount(); i++) {
    Serial.print("Cidlo ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensors.getTempCByIndex(i));
    Serial.println(" C");
  }
  delay(2000);
}
```

### Čtení adres ze Serial monitoru

Otevřete Serial Monitor (57600 baud). Uvidíte adresy a aktuální teploty všech čidel.

### Přiřazení adres k fyzickým pozicím

Postupně zahřívejte jednotlivá čidla (rukou, teplou vodou) a sledujte, které čidlo reaguje. Zapište si přiřazení:

| Pozice | Adresa |
|--------|--------|
| acumulatorFirst | `{ 0x.., 0x.., ... }` |
| acumulatorSecond | `{ 0x.., 0x.., ... }` |
| acumulatorThird | `{ 0x.., 0x.., ... }` |
| acumulatorFour | `{ 0x.., 0x.., ... }` |
| acumulatorOutput | `{ 0x.., 0x.., ... }` |
| boiler | `{ 0x.., 0x.., ... }` |
| currentHeating | `{ 0x.., 0x.., ... }` |
| returnHeating | `{ 0x.., 0x.., ... }` |
| heaterTemperature | `{ 0x.., 0x.., ... }` |

### Vyplnění sensorconfig.h

Zkopírujte `sensorsconfig_default.h` na `sensorsconfig.h` a doplňte zjištěné adresy:

```cpp
DeviceAddress acumulatorFirst =    { 0x28, 0xAA, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };
// ... atd.
```

## 4. Konfigurace config.h

Zkopírujte `config_default.h` na `config.h` a vyplňte hodnoty.

### WiFi

```cpp
#define WifiSSID "MojeWifi"
#define WifiPassword "MojeHeslo"
```

### MQTT

```cpp
#define MQTTUsername "arduino"
#define MQTTPassword "tajne_heslo"
#define MQTTHost "192.168.1.100"
```

### MQTT topicy

Nastavte topicy odpovídající vaší konfiguraci HomeAssistant. Příklad:
```cpp
#define TOPIC_THERMOSTAT "home/heating/thermostat"
#define TOPIC_MODE "home/heating/mode"
#define TOPIC_ZEROPOINT "home/heating/zeropoint"
#define TOPIC_CURRENTDATETIEM "home/heating/datetime"
#define TOPIC_THERMOSTATSETCHANGED "home/heating/setpoint"
#define TOPIC_OUTSIDETEMPERATURE "home/heating/outside"
#define TOPIC_HEATERSTATE "home/heating/state"
#define TOPIC_FVE "home/fve/data"
```

### Parametry regulace

Viz podrobný popis všech parametrů v [PARAMETERS.md](PARAMETERS.md).

## 5. Upload firmware

1. V Arduino IDE vyberte desku: **Tools → Board → Arduino Mega or Mega 2560**
2. Vyberte port: **Tools → Port → COMx** (váš Arduino)
3. Nahrajte: **Sketch → Upload**

## 6. Ověření funkce

### Serial monitor

Otevřete Serial Monitor (57600 baud). Po startu byste měli vidět:
```
Sensor Id: <id_vašeho_venkovního_čidla>
Wifi status 3
Connect
Subscribes
```

### MQTT Explorer — ověření publikace dat

1. Nainstalujte [MQTT Explorer](https://mqtt-explorer.com/)
2. Připojte se na váš MQTT broker
3. Ověřte, že se každých 30 s objevují zprávy na topicích `TOPIC_HEATERSTATE` a `TOPIC_FVE`

## 7. Nastavení MQTT brokeru

### Instalace Mosquitto

```bash
# Debian/Ubuntu
sudo apt install mosquitto mosquitto-clients

# Docker
docker run -d --name mosquitto -p 1883:1883 eclipse-mosquitto
```

### Vytvoření uživatele a hesla

```bash
sudo mosquitto_passwd -c /etc/mosquitto/passwd arduino
```

V konfiguraci `/etc/mosquitto/mosquitto.conf`:
```
listener 1883
allow_anonymous false
password_file /etc/mosquitto/passwd
```

### Ověření konektivity

```bash
# Odběr všech zpráv
mosquitto_sub -h localhost -u arduino -P heslo -t "home/heating/#" -v

# Odeslání testovací zprávy
mosquitto_pub -h localhost -u arduino -P heslo -t "home/heating/mode" -m "Automatic" -r
```

## 8. Bezdrátová pokojová čidla

### Zapojení SRX882S přijímače

Viz [HARDWARE.md — Zapojení SRX882S](HARDWARE.md#zapojení-srx882s-bezdrátový-přijímač). Přijímač je na pinech 2 (DATA) a 3 (interrupt).

### Nastavení TX07K-TXC snímačů

1. Vložte baterie do snímače TX07K-TXC
2. Nastavte kanál na **1** (přepínač na snímači)
3. Stiskněte TX tlačítko na snímači — Arduino si uloží ID snímače do EEPROM

### Spuštění ArduinoSerialReader

Aplikace [ArduinoSerialReader](https://github.com/Zefek/ArduinoSerialReader) čte data z Serial výstupu Arduina a posílá je do MQTT:
1. Naklonujte repozitář
2. Nakonfigurujte připojení k sériovému portu a MQTT brokeru
3. Spusťte aplikaci

### Ověření dat v MQTT

V MQTT Exploreru ověřte, že se na příslušných topicích objevují teploty z jednotlivých místností.
