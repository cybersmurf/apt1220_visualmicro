# Copilot Instructions – apt1220_visualmicro

## Project Overview
- **Platform**: ESP32 (OLIMEX ESP32-POE), Arduino framework
- **IDE**: Visual Studio + Visual Micro extension
- **Language**: C/C++ (Arduino .ino)
- **Board**: `esp32_esp32-poe`
- **ESP32 Core**: 3.3.7 (ESP-IDF 5.x)
- **RTOS**: FreeRTOS (tasks, mutexes, queues)

## Architecture
- Dual-core ESP32: Core 0 = OTA/BLE, Core 1 = main app + serial readers
- TCP client communicates with a PC server (custom binary protocol)
- Offline buffer stored in LittleFS (`/apt1220/aptbuffer.txt`)
- LCD 20x4 I2C display, DS1307 RTC, dual RFID serial readers (SerialC, SerialD)
- AP mode for WiFi config via async web server

## Key Libraries (shared project references in Visual Micro)
- `ESP_Async_WebServer` 3.10.0 — referenced as shared project in `Libraries\ESP_Async_WebServer\3.10.0\`
- `AsyncTCP` — global library in `C:\Develop\arduino\libraries\AsyncTCP`
- `LCDI2C_Multilingual`, `I2C_RTC`, `RTClib`

## Visual Micro Specifics
- Visual Micro uses **shared project references** for libraries (`.vcxitems`), NOT the global Arduino library folder directly.
- Library source files are copied/referenced from `apt1220_visualmicro\Libraries\<name>\<version>\` — edits must go there, not to global `C:\Develop\arduino\libraries\`.
- Build cache is at `C:\Users\sramek\AppData\Local\Temp\VMBuilds\apt1220_visualmicro\` — delete it when library edits don't take effect.
- Visual Micro does NOT auto-discover ESP32 core "bundled" libraries (e.g., `Hash`). Their include paths are missing from the build.
- `__has_include()` may return inconsistent results in Visual Micro (reports file exists, but compiler can't find it). Avoid relying on it for conditional compilation.

## Known Patches (must reapply after library updates)

### ESP_Async_WebServer — BackPort_SHA1Builder
**Location**: `apt1220_visualmicro\Libraries\ESP_Async_WebServer\3.10.0\ESP_Async_WebServer\src\`

**Problem**: `AsyncWebSocket.cpp` tries to include system `<SHA1Builder.h>` on ESP-IDF 5+, but Visual Micro can't find it (Hash library not in include paths). The backport `BackPort_SHA1Builder.h/.cpp` originally had `#if ESP_IDF_VERSION_MAJOR < 5` guard, making the class invisible on ESP-IDF 5+.

**Fix applied**:
1. `AsyncWebSocket.cpp` line 10: Changed to `#include "BackPort_SHA1Builder.h"` (unconditional)
2. `BackPort_SHA1Builder.h`: Removed `#if ESP_IDF_VERSION_MAJOR < 5` / `#if !__has_include(<HashBuilder.h>)` guards — class is always defined
3. `BackPort_SHA1Builder.cpp`: Same — removed conditional guards

**When to reapply**: After updating ESP_Async_WebServer library version.

## Coding Conventions
- Czech comments are OK and common in this codebase
- Use `F()` macro for string literals in `Serial.print()` to save RAM
- Use `snprintf()` instead of `sprintf()` for buffer safety
- All I2C access (LCD, RTC) must be protected by `i2cMutex`
- All TCP client access must be protected by `tcpMutex`
- Use `STACK_WORDS(bytes)` macro (now returns bytes directly for ESP32)
- Prefer `vTaskDelay(pdMS_TO_TICKS(ms))` over `delay(ms)` in tasks
- LCD display functions: use `display_line_formated()` / `display_line_unformated()` (mutex-safe), NOT direct `lcd2.printf()`
- `_fast_clear_disp_unsafe()` = only inside existing mutex lock; `fast_clear_disp()` = safe wrapper with 500ms timeout

## FreeRTOS Task Architecture

| Task | Core | Stack | Priority | Function |
|---|---|---|---|---|
| loop() (Arduino main) | 1 | default | 1 | TCP(), tDEMOscreen(), send_entire_file_buffer() |
| tSEC_TIMER | 1 | STACK_WORDS(1536) | 0 | Updates SEC_TIMER from millis() |
| tLAST_PING | 0 | STACK_WORDS(8192) | 1 | TCP heartbeat ping + reconnect |
| CommandProcessor | 1 | STACK_WORDS(8192) | 2 | Reads serialDataQueue → calls serial() (FILE OPS!) |
| SerialC_Reader | 1 | STACK_WORDS(2048) | 1 | Reads HardwareSerial(1) → serialDataQueue |
| SerialD_Reader | 1 | STACK_WORDS(2048) | 1 | Reads HardwareSerial(2) → serialDataQueue |
| OTATask | 0 | STACK_WORDS(4096) | 1 | checkForUpdatesBackground() + performUpdate() |

### Synchronization Primitives
- `i2cMutex` — protects ALL I2C bus access (LCD writes + RTC reads)
- `tcpMutex` — protects ALL WiFiClient operations (connect/read/write/stop)
- `serialDataQueue` — FreeRTOS queue (10 items × SerialData_t) between reader tasks → CommandProcessor
- `demoMutex` — unused legacy (was for tDEMOcode task, now display runs from loop())

### Mutex Rules (CRITICAL — prevents deadlocks)
- Never nest mutexes (never hold i2cMutex while taking tcpMutex or vice versa)
- Always take-and-release sequentially, never hold across function boundaries
- Use timeouts (pdMS_TO_TICKS) instead of portMAX_DELAY where possible
- `lockI2C()` / `unlockI2C()` use portMAX_DELAY — only for short LCD batch writes (menuconfig, menu_dump)

## LCD Display Map (20×4)

### Main Screen (tDEMOscreen — when timeout1 expired)
```
Line 0: " eMISTR 2025   ETH "     or " eMISTR 2025 DVERE "
Line 1: "----*---------------"     bouncing asterisk effect
Line 2: "25.06.2025  14:30:00"     RTC time (DD.MM.YYYY  HH:MM:SS)
Line 3: "OFF-LINE  Volno: 97%"    or "      ON-LINE       "
```

### Offline RFID Data Display (in serial(), prefix → line)
| Prefix | Meaning | LCD Line |
|---|---|---|
| A | Operation | 0 |
| B | Order | 1 |
| D | Worker (triggers file save) | 3 |
| G | Worker alt | 0 |
| H | Quantity | 2 |
| default | Unknown | 3 |

### File Save Confirmation
```
*------------------*
|     ULOZENO      |
|   DO SOUBORU     |
*------------------*
```

### TCP Server Response Frame (command 0x0E)
```
*------------------*
| centered message |
*------------------*
```

## TCP Protocol (Binary)

### Outgoing Commands (terminal → server)
| Byte | Command | Payload |
|---|---|---|
| 0x01 | Echo/Ping | `\n` |
| 0x02 | Set time response | `~YYYY-MM-DD HH:MM:SS\n` |
| 0x04 | Worker | `D<rfid_code>~YYYY-MM-DD HH:MM:SS\n` |
| 0x05 | Order | `B<code>\n` |
| 0x06 | Operation | `A<code>\n` |
| 0x07 | (prefix F) | data |
| 0x08 | (prefix E) | data + time |

### Incoming Commands (server → terminal)
| Byte | Action |
|---|---|
| 0x01 | Respond with `OK\n` |
| 0x02 | Set RTC time from payload |
| 0x03 | Send current time back |
| 0x04 | Display worker name on line 3 |
| 0x05 | Display order on line 1 |
| 0x06 | Display operation on line 0 |
| 0x07 | Display on line 2 |
| 0x0B | Set timer1 value |
| 0x0C | Force demo screen refresh (timeout1=0) |
| 0x0E | Framed centered message on line 1 |
| 0x18 | Door unlock confirmation |

### Connection Flow
1. `connectToServer()` → TCP connect to `ip_server:54321`
2. `tLAST_PINGcode` sends `\x01\n` every 1.9s
3. Server responds `OK\n` → last_ping updated
4. If no response for `ping_timeout` (4s) → `setNetOn(0)` → reconnect

## Offline Buffer System

### Data Flow
```
RFID scan → serialReaderTask → serialDataQueue → commandProcessorTask
    → serial() → off_buffer[] accumulates commands
    → prefix D/0-9 triggers flush:
        → appendFile(aptbuffer.txt, off_buffer)
        → LCD shows "ULOZENO DO SOUBORU"
        → memset(off_buffer, 0)
```

### File Buffer Recovery (when back online)
```
loop() detects net_on + file has data
    → send_entire_file_buffer()
    → reads line by line from aptbuffer.txt
    → sends each line via TCP, waits for "OK" confirmation
    → failed lines copied to aptbuffer.tmp
    → rename tmp → original
```

### Limits
- `off_buffer`: 250 bytes (in-memory accumulator)
- `aptbuffer.txt`: max 128 kB (`MAX_BUFFER_FILE_SIZE`)
- File size checked before every append

## Configuration Commands (via RFID)

| Command | Action | Triggers |
|---|---|---|
| SET-IP | Set IP address | restartNetwork |
| SET-SERV | Set server IP | — |
| SET-MASK | Set subnet mask | restartNetwork |
| SET-GATE | Set gateway | restartNetwork |
| SET-DHCP | Enable DHCP | restartNetwork, saveConfig |
| SET-STATIC | Disable DHCP | restartNetwork, saveConfig |
| SET-ETH | Use Ethernet | restartNetwork, saveConfig |
| SET-WIFI | Use WiFi | saveConfig |
| SET-SSID | Set WiFi SSID | restartNetwork, saveConfig |
| SET-PASSW | Set WiFi password | restartNetwork, saveConfig |
| SET-TIM1 | Set display timeout (timer1) | — |
| SET-TIM2 | Set confirm timeout (timer2) | — |
| SET-OP-C | Set RFID C operation code | — |
| SET-OP-D | Set RFID D operation code | — |
| SET-KEY | Switch to door mode | — |
| SET-PRACANT | Switch to worker mode | — |
| SET-SAVE | Save config + restart | ESP.restart() |
| SET-LOAD | Reload config from NVS | — |
| CONFIG / SHOWCONFIG | Show config on LCD + reconnect | timeout1 = 30s guard |
| SHOW-NET | Show network config on LCD | — |
| SHOW-WIFI | Show WiFi config on LCD | — |
| DEFAULT | Factory reset | saveConfig |
| RESET-BUFFER | Delete offline buffer file | — |
| MEMINFO | Show FIFO debug info | — |
| LCDINIT | Reinitialize LCD | — |
| SET-APSSID | Set AP mode SSID | apSettingsChanged |
| SET-APPASS | Set AP mode password | apSettingsChanged |
| SET-AP-FIX | AP SSID without MAC | apSettingsChanged |
| SET-AP-MAC | AP SSID with MAC suffix | apSettingsChanged |

## Hardware Pinout

| Function | GPIO |
|---|---|
| SerialC RX | 36 |
| SerialC TX | 14 |
| SerialD RX | 32 |
| SerialD TX | 33 |
| I2C SDA | default (21) |
| I2C SCL | default (22) |
| Ethernet | LAN8720 (built-in OLIMEX ESP32-POE) |

## Current Version
- Firmware: `1.0.2.2`
- Protocol: `4.0.0` (VERZE_PRACANT)
- OTA check URL: `https://petrsramek.eu/emistr/apt1220/version.txt`
- OTA firmware URL: `https://petrsramek.eu/emistr/apt1220/firmware.bin`
