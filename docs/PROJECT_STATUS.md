# Project Status — APT1220 eMISTR Terminal

> Poslední aktualizace: 2025-06

## Aktuální verze

| Položka | Hodnota |
|---|---|
| Firmware verze | `1.0.2.1` |
| Protokol verze | `4.0.0` (VERZE_PRACANT) |
| ESP32 Core | 3.3.7 (ESP-IDF 5.x) |
| AsyncTCP | 3.4.10 (ESP32Async) |
| ESP_Async_WebServer | 3.10.0 (patchovaná) |

## Stav modulů

### ✅ Funkční
- **TCP komunikace** — binární protokol s PC serverem, ping/reconnect, mutex ochrana
- **Offline buffer** — zápis do LittleFS, automatické odesílání po reconnectu (`send_entire_file_buffer`)
- **Dual RFID čtečky** — SerialC + SerialD přes FreeRTOS queue
- **LCD displej** — hlavní obrazovka, konfigurace, stavové zprávy, mutex ochrana
- **Ethernet** — OLIMEX ESP32-POE, DHCP i statická IP, hostname z MAC
- **WiFi** — záložní připojení, konfigurovatelné přes RFID příkazy
- **AP režim** — konfigurační hotspot s async web serverem
- **Preferences** — konfigurace v NVS (IP, masky, SSID, čtečky, timeouty)
- **OTA aktualizace** — HTTPS i HTTP, kontrola verze na serveru, task na Core 0
- **RTC DS1307** — synchronizace času ze serveru, zobrazení na LCD
- **FreeRTOS synchronizace** — i2cMutex, tcpMutex, serialDataQueue
- **Konfigurační vstup** — podpora `DEFAULT` i `BACKSPACE` pro mazání znaků

### ⚠️ Vyžaduje pozornost
- **Velikost .ino** — 3600 řádků / 127 kB v jednom souboru, zvážit rozdělení do modulů
- **Nepoužívaný kód** — `send_buffer()`, `server_read()`, `tDEMOrun()`, `tDEMOcode()` jsou legacy funkce nahrazené novými verzemi, ale zůstávají v kódu
- **`processSerialData()`** — implementovaná ale nepoužívaná (zakomentovaná v `commandProcessorTask`)
- **Build warning** — Visual Micro vyžaduje ruční patching knihovny ESP_Async_WebServer po každé aktualizaci

### 🔴 Vypnuto / Zakomentováno
- **BLE scanning** — `USE_BLE` = 0, NimBLE kód je zakomentovaný
- **Heap/stack monitoring** — `printHeapStats()`, `printTaskWatermarks()` zakomentovány
- **Debug heartbeat** — `debugHeartbeatTick()` zakomentován
- **FTP klient** — ESP32FTPClient zakomentován
- **EEPROM** — nahrazeno Preferences (NVS)

## Stav knihoven

| Knihovna | Stav | Poznámka |
|---|---|---|
| AsyncTCP | ✅ Aktuální | ESP32Async/AsyncTCP v3.4.10, nahrazena stará verze |
| ESP_Async_WebServer | ⚠️ Patchovaná | BackPort_SHA1Builder guardy odstraněny pro Visual Micro |
| LCDI2C_Multilingual | ✅ OK | Globální knihovna |
| I2C_RTC | ✅ OK | DS1307 driver |
| RTClib | ✅ OK | DateTime helpers |

## Známé problémy Visual Micro

1. **`__has_include()` nespolehlivé** — preprocessor tvrdí, že soubor existuje, ale kompilátor ho nenajde
2. **Core bundled libraries** — `Hash`, `SHA1Builder.h` nejsou v include paths
3. **Build cache** — po úpravě shared project knihoven je nutné smazat `VMBuilds\apt1220_visualmicro\`
4. **Library discovery** — Visual Micro nerozpozná závislosti deklarované v `library.json` pro ESP32

## Architektonické poznámky

### Task alokace
| Task | Core | Stack | Priorita | Poznámka |
|---|---|---|---|---|
| loop() (main) | 1 | default | 1 | TCP, display, file buffer |
| tSEC_TIMER | 1 | 384 words | 0 | millis() → SEC_TIMER |
| tLAST_PING | 0 | 2048 words | 1 | TCP heartbeat + reconnect |
| CommandProcessor | 1 | 768 words | 2 | Zpracování RFID příkazů |
| SerialC_Reader | 1 | 512 words | 1 | Čtení RFID čtečky C |
| SerialD_Reader | 1 | 512 words | 1 | Čtení RFID čtečky D |
| OTATask | 0 | 1024 words | 1 | Kontrola + stahování FW |

### TCP protokol (binární)
- Příkazy: `\x01` echo, `\x02` set time, `\x03` get time, `\x04` worker, `\x05` order, `\x06` operation
- Odpovědi: `OK\n`, `~` potvrzení
- Oddělovač: `\n` (newline)
- Čas se přidává k příkazům D, E, J ve formátu `~YYYY-MM-DD HH:MM:SS`

### Offline buffer flow
```
RFID scan → serial() → off_buffer[] → appendFile(aptbuffer.txt)
                                              ↓
            net_on=1 → send_entire_file_buffer() → TCP → server OK → smazat řádek
```
