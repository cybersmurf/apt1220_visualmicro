# APT1220 — eMISTR Terminál

Firmware pro průmyslový terminál eMISTR postavený na ESP32 (OLIMEX ESP32-POE).  
Terminál slouží k evidenci docházky a výrobních operací pomocí RFID čteček, s online/offline režimem a automatickou synchronizací dat s PC serverem.

## Hardwarová platforma

| Komponenta | Typ |
|---|---|
| MCU | ESP32 (OLIMEX ESP32-POE) |
| Displej | LCD 20×4 I2C (Surenoo SLC1602A, adresa 0x27) |
| RTC | DS1307 (I2C) |
| RFID čtečka C | Sériový port 1 (GPIO 36 RX / 14 TX, 9600 baud) |
| RFID čtečka D | Sériový port 2 (GPIO 32 RX / 33 TX, 9600 baud) |
| Síť | Ethernet (LAN8720, vestavěný na OLIMEX ESP32-POE) + WiFi |
| Úložiště | LittleFS (offline buffer `/apt1220/aptbuffer.txt`, max 128 kB) |

## Funkce

- **Dokumentace** — [Uživatelská příručka (CZ)](docs/MANUAL_CZ.md)
- **Dual RFID čtečky** — podpora standardních i ID-12 čteček na dvou sériových portech
- **TCP klient** — binární protokol pro komunikaci s PC serverem (port 54321)
- **Offline buffer** — při výpadku sítě se data ukládají do LittleFS a automaticky odesílají po obnovení spojení
- **OTA aktualizace** — kontrola a stahování nového firmware přes HTTP/HTTPS
- **AP režim** — konfigurační WiFi hotspot s webovým rozhraním při nedostupné síti
- **Konfigurace přes RFID** — SET-příkazy pro nastavení IP, masky, SSID, timeoutů atd.
- **Denní automatický restart** — ochrana proti dlouhodobému běhu

## Architektura (FreeRTOS)

```
Core 1 (hlavní)                    Core 0 (pozadí)
├── loop()                         ├── tLAST_PING  — heartbeat/reconnect
│   ├── TCP()                      └── OTA Task    — kontrola aktualizací
│   ├── send_entire_file_buffer()
│   └── tDEMOscreen()
├── CommandProcessor (pri 2)
├── SerialC_Reader (pri 1)
└── SerialD_Reader (pri 1)
```

**Synchronizační primitiva:**
- `i2cMutex` — ochrana I2C sběrnice (LCD + RTC)
- `tcpMutex` — ochrana TCP klienta
- `serialDataQueue` — fronta pro data ze čteček → CommandProcessor

## Vývojové prostředí

- **IDE**: Visual Studio 2022/2026 + Visual Micro extension
- **Arduino Core**: ESP32 3.3.7 (ESP-IDF 5.x)
- **Board**: `esp32_esp32-poe`
- **Build cache**: `C:\Users\sramek\AppData\Local\Temp\VMBuilds\apt1220_visualmicro\`

## Struktura projektu

```
apt1220_visualmicro/
├── apt1220_visualmicro.ino      # Hlavní firmware (~3600 řádků, 127 kB)
├── Libraries/
│   ├── ESP_Async_WebServer/     # Shared project reference (patchovaná verze)
│   │   └── 3.10.0/
│   └── ESP32/
│       └── 3.3.7/
└── __vm/                        # Visual Micro konfigurace
```

## Klíčové knihovny

| Knihovna | Verze | Typ reference |
|---|---|---|
| ESP_Async_WebServer | 3.10.0 | Shared project (lokální, patchovaná) |
| AsyncTCP | — | Globální (`C:\Develop\arduino\libraries\`) |
| LCDI2C_Multilingual | — | Globální |
| I2C_RTC | — | Globální |
| RTClib | — | Globální |

## Známé patche (nutno znovu aplikovat po aktualizaci knihoven)

### ESP_Async_WebServer — BackPort_SHA1Builder

Visual Micro nedokáže najít systémový `SHA1Builder.h` z ESP32 core Hash knihovny.

**Soubory**: `Libraries/ESP_Async_WebServer/3.10.0/ESP_Async_WebServer/src/`
1. `AsyncWebSocket.cpp` — změněn include na `#include "BackPort_SHA1Builder.h"` (bezpodmínečný)
2. `BackPort_SHA1Builder.h` — odstraněny `#if ESP_IDF_VERSION_MAJOR < 5` guardy
3. `BackPort_SHA1Builder.cpp` — odstraněny `#if ESP_IDF_VERSION_MAJOR < 5` guardy

## Konfigurace terminálu (RFID příkazy)

| Příkaz | Popis |
|---|---|
| `SET-IP` | Nastavení IP adresy |
| `SET-SERV` | Nastavení adresy serveru |
| `SET-MASK` | Nastavení masky sítě |
| `SET-GATE` | Nastavení brány |
| `SET-DHCP` / `SET-STATIC` | Přepnutí DHCP/statická IP |
| `SET-ETH` / `SET-WIFI` | Přepnutí Ethernet/WiFi |
| `SET-SSID` / `SET-PASSW` | Nastavení WiFi credentials |
| `SET-SAVE` | Uložení konfigurace + restart |
| `SET-LOAD` | Načtení konfigurace |
| `CONFIG` | Zobrazení konfigurace na LCD |
| `STORNO` | Zrušení zadávání (bez restartu) |
| `BACKSPACE` | Smazání posledního znaku |
| `RESET-BUFFER` | Smazání offline bufferu |
| `DEFAULT` | Tovární nastavení |

## Licence

Interní projekt — AGERIT / eMISTR
