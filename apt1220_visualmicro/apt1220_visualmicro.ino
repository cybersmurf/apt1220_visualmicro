#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_wifi.h>
#include "string.h"
#include <ETH.h>
//#include <WebServer.h>
#include "time.h"
#include <Arduino.h>
#include <stdarg.h>  // Knihovna pro variabilní argumenty
#include <stdio.h>   // Knihovna pro vsnprintf
#include <LCDI2C_Multilingual.h>

#include <Wire.h>
#include <I2C_RTC.h>
#include <RTClib.h>
//#include <uRTClib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_task_wdt.h>

///vypnutí pouití EEPROM a EspConfigLib
//#include "uEEPROMLib.h"
//#include "uEspConfigLib.h"

#include <Preferences.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include <WiFiClientSecure.h>   // Needed for HTTPS

#define SPIFFS LittleFS
#define FORMAT_LITTLEFS_IF_FAILED true

#define DEBUG_MODE

//#include <ESP32FTPClient.h> // Include the FTP client library


// Caution: It need to be a global variable or a global pointer.
// if FS is a 'setup' variable it will lead to crashes
///vypnutí pouití EEPROM a EspConfigLib
//uEspConfigLibFSInterface* configFsC;
//uEspConfigLib* config;

// uEEPROMLib eeprom;
///vypnutí pouití EEPROM a EspConfigLib
//uEEPROMLib eeprom(0x50);

static DS1307 rtc2;
//uRTCLib rtc2;

LCDI2C_Latin_Symbols lcd2(0x27, 20, 4);    // I2C address = 0x27; LCD = Surenoo SLC1602A (European)
WiFiClient client;
//WebServer server(80);
AsyncWebServer server(80);

//ESP32FTPClient ftpClient;

// Inicializace sériovıch portù
HardwareSerial SerialC(1); // Pro SerialC
HardwareSerial SerialD(2); // Pro SerialD

#define TCPCONFIG 1
#define VERZE_PRACANT  "4.0.0"
#define ALOC_MEM 64000
#define SERB_USEPORTD
#define timeout_reset 86400;  // jak casto resetovat

// commands of terminal
#define C_ERROR_DATA     $00;  // nekorektni data
#define C_ECHO           $01;  // Pošli odezvu        PC <-> TERMINAL
#define C_SET_TIME       $02;  // Nastav èas          PC  -> TERMINAL
#define C_GET_TIME       $03;  // Pošli èas           PC <-> TERMINAL
#define C_DATA_WORKER    $04;  // Posilam zamestnance PC <-> TERMINAL

#define PORT 		     54321

#define IF_DOWN		     0
#define IF_COMING_UP	 1
#define IF_UP			 2
#define IF_COMING_DOWN	 3

#define RTC_YEAR_OFFSET  1900

//static int __attribute__((section(".noinit"))) loaded_default = 0;
static int loaded_default = 0;

static bool __attribute__((section(".noinit"))) ip_dhcp;
static char __attribute__((section(".noinit"))) ip_adr[16];
static char __attribute__((section(".noinit"))) ip_mask[16];
static char __attribute__((section(".noinit"))) ip_server[16];
static char __attribute__((section(".noinit"))) ip_gate[16];
static unsigned long last_ping;                                     // cas od posledni komunikace s PC
static int __attribute__((section(".noinit")))  ping_timeout;		// DISPLAY NAME timeout
static int __attribute__((section(".noinit")))  key_maker = 0;      // 0 - jedna se o pracanta
// 1 - jedna se o zamek
static long __attribute__((section(".noinit"))) timeout1;
static long __attribute__((section(".noinit"))) timeout2;

static int __attribute__((section(".noinit")))  rfid_reader_c;      // 0 - normalni èteèka
static int __attribute__((section(".noinit")))  id12_c;             // 0 - normalni èteèka
// 1 - rfid modul
static int __attribute__((section(".noinit")))  rfid_reader_d;      // 0 - normalni èteèka
static int __attribute__((section(".noinit")))  id12_d;             // 0 - normalni èteèka
// 1 - rfid modul

static long __attribute__((section(".noinit"))) timer1;		        // DISPLAY NAME timeout
static long __attribute__((section(".noinit"))) timer2;		        // CONFIRM TIMEOUT
static long __attribute__((section(".noinit"))) timer3;
static char __attribute__((section(".noinit"))) op_c[16];
static char __attribute__((section(".noinit"))) op_d[16];

static String activeMAC;

String buffer2StringTmp;

long timer4;
long timer_reset;

char rfid_c_last[15];
long rfid_c_last_time;
char rfid_d_last[15];
long rfid_d_last_time;

long r_operation;
long r_worker;
long r_order;
long r_service;
long r_time;

static int net_on;
int saved;

//static unsigned long __attribute__((section(".noinit"))) fifo_start;
//static unsigned long __attribute__((section(".noinit"))) fifo_end;
//static unsigned long __attribute__((section(".noinit"))) fifo_first;
//static unsigned long __attribute__((section(".noinit"))) fifo_last;

static unsigned long  fifo_start = 0;
static unsigned long  fifo_end = 64000;
static unsigned long  fifo_first = 0;
static unsigned long  fifo_last = 0;
static unsigned long  fifo_size = 0;

const size_t MAX_BUFFER_FILE_SIZE = 128 * 1024; // 128 kB
bool isSendingFileBuffer = false; // Globální flag pro signalizaci probíhajícího odesílání

static unsigned long physaddr;

char buffer[251] = { 0 };

char off_buffer[251] = { 0 };
//char __attribute__((section(".noinit"))) off_buffer[251];
#define off_buffer_size 250    // OFFLINE BUFFER SIZE - maximu
static unsigned long SEC_TIMER;

static bool eth_connected = false;
static int  runCounter = 0;

// Nastavení WiFi a TCP
static char ssid[32] = "AGERIT_AC 2GHz";
static char password[32] = "AGERITagerit512";
//static char* ssid = "blackies";
//static char* password = "Blackies105111";
const char* serverIP = "192.168.88.221";
//const char* serverIP = "192.168.225.221";
const int serverPort = 54321;

const bool vSerialDebug = true;
const int  vLogLevel = 0;

const char* bufferFilePath = "/apt1220/aptbuffer.txt"; // Definujeme cestu k souboru

static bool useWifi = false;
static bool useETH = true;

static bool useDHCP = true;

static int efect = 0;

// Add this global variable to track time for the asterisk movement
static unsigned long lastEffectChange = 0;

// Lokální verze firmware
const String localVersion = "1.0.1.5";

//String currentVersion = "1.0.1.2";
String newVersion = "";

// URL k souboru, kde je uvedena aktuální verze firmware na serveru
//const char* versionURL = "http://192.168.222.114:82/ota_update/version.txt";
const char* versionURL = "https://petrsramek.eu/emistr/apt1220/version.txt";
// URL k novému firmware (binární soubor)
//const char* firmwareURL = "http://192.168.222.114:82/ota_update/firmware.bin";
const char* firmwareURL = "https://petrsramek.eu/emistr/apt1220/firmware.bin";

// Define your server's Root CA certificate (same as before)
const char* rootCACertificate_OTA = \
"---- - BEGIN CERTIFICATE---- -\n" \
"MIIEGzCCAwOgAwIBAgIQCvUoYPNvi0rvR6nXFMo29zANBgkqhkiG9w0BAQsFADBI\n" \
"MRswGQYDVQQDDBJFU0VUIFNTTCBGaWx0ZXIgQ0ExHDAaBgNVBAoME0VTRVQsIHNw\n" \
"b2wuIHMgci4gby4xCzAJBgNVBAYTAlNLMB4XDTI1MDUxOTAxMDQ0N1oXDTI1MDgx\n" \
"NzAxMDQ0NlowGDEWMBQGA1UEAxMNcGV0cnNyYW1lay5ldTCCASIwDQYJKoZIhvcN\n" \
"AQEBBQADggEPADCCAQoCggEBANioEA3ZQhl17xZj7RUfFKKYctlCsHUFLSeDjKYw\n" \
"LYIgPaXIQW7uq6bkoZbiXU + 0MFNYENs2we1lSZbvTuSLV1n5edc4ZYUiHmoY1d9i\n" \
"qMy5WqbcOEDqzIbG9DPr580gqOI1PzqfZ6UYb5FTZyziA3mI86s4ZDPsVPK9PAdV\n" \
"hzdooV5vYqxmV6JpQLSkOIkmLvd + hBya + g9vTlRYW1rX / CnHxkznv6CM2 + Uh / A9R\n" \
"8vNAdL07zcYiu9Fyl6Qg3wTbMxFY9xmFrX3CcQCRsRJi1ONNUKRDWlRdytqoRsoC\n" \
"WPZbocYwVJlGkfbtdLUN + 7vQHvTmnfVeu7uNeN8b93zS9msCAwEAAaOCAS8wggEr\n" \
"MAsGA1UdDwQEAwIFoDAdBgNVHQ4EFgQUtzIVdxcqkyQXEAq1eXFpz + w9Ix4wHwYD\n" \
"VR0jBBgwFoAUqM1h + TUmOXLCXdRGzRli / gxXjaUwHQYDVR0lBBYwFAYIKwYBBQUH\n" \
"AwEGCCsGAQUFBwMCMAwGA1UdEwEB / wQCMAAwga4GA1UdEQSBpjCBo4ITYWRhbTMu\n" \
"cGV0cnNyYW1lay5ldYIXYWRhbWZpbmFsLnBldHJzcmFtZWsuZXWCC2RlbHBoaTR1\n" \
"LmV1gg9rb2xvcHJvYWRhbWEuY3qCDXBldHJzcmFtZWsuZXWCD3d3dy5kZWxwaGk0\n" \
"dS5ldYITd3d3LmtvbG9wcm9hZGFtYS5jeoIRd3d3LnBldHJzcmFtZWsuZXWCDXd3\n" \
"dy5zdWRhdGEuZXUwDQYJKoZIhvcNAQELBQADggEBAFr8bG2SZCg2GZL8jyCA / iu /\n" \
"77A8NBrSXVmKSM7bgzqQDQ / QxIYR1T7a / dyhG6ZX4XY8Owj30GXPuQQwbR + gTWoD\n" \
"QasEBwuNywEXVt9XFKkP4O / uwewCNRadLFAqu5GT0rTgMtECbtuJ1MiusumatLUu\n" \
"V / AFTVzxm6abtXAq8xUrR1248kOtXW9hgl174ENIg + xRr261bn8PBrorhxuK + btD\n" \
"kxoM0ROe5T0zjNawX3KzIMiuq / qf758abSTfRa2rKtzi63pVvHRbnDjXCziOzadP\n" \
"z1bfP984ztj80YymWtufoXRIpB7f / BjujEjzmmsdFlxg6Cp5FCmeKv5VUTxqY8Q =\n" \
"---- - END CERTIFICATE---- -\n";


const char* firmwareFTP = "update.emistr.cz";
const char* firmwareFTPPath = "/apt1220";
const char* firmwareFTPUser = "emistrUpdate";
const char* firmwareFTPPassword = "'X9SeQ$+#29d*b7#N'";

// Globální promìnné pro OTA aktualizaci
TaskHandle_t otaTaskHandle = NULL;
bool otaInProgress = false;
bool otaUpdateAvailable = false;

Preferences preferences;

static bool saveNewConfigData = false; // Flag pro uloení novıch dat do konfigurace
static bool restartNetwork    = false; // Flag pro restartování sítì

byte customChar[] = {
  B00010,
  B00100,
  B01100,
  B00100,
  B00100,
  B00100,
  B01110,
  B00000
};

TaskHandle_t tSEC_TIMER = NULL;
TaskHandle_t tLAST_PING = NULL;
TaskHandle_t tTCP = NULL;
TaskHandle_t tDEMO = NULL;

SemaphoreHandle_t demoMutex = NULL;
SemaphoreHandle_t tcpMutex = NULL;
SemaphoreHandle_t lastPingMutex = NULL;
SemaphoreHandle_t secTimerMutex = NULL;

char c_string2[16] = { 0 };

static char g_last_tcp_command_code = 0;

// Struktura pro pøenos dat z èteèek do fronty
struct SerialData_t {
    char data[100]; // Buffer pro data
    int port;       // Port, ze kterého data pøišla (1 pro C, 2 pro D)
};

// Handle pro frontu, která bude dret pøíchozí data
QueueHandle_t serialDataQueue;

SemaphoreHandle_t i2cMutex; // Mutex pro ochranu I2C sbìrnice (LCD a RTC)

///vypnutí pouití EEPROM a EspConfigLib
/*
void setup_inifile() {
    configFsC = new uEspConfigLibFSLittlefs("/config.ini", true);
    if (configFsC->status() == uEspConfigLibFS_STATUS_FATAL) {
        Serial.println("  * Error initializing FS LittleFS");
    }

    config = new uEspConfigLib(configFsC);

    config->addOption("wifi_ssid", "SSID of your WiFi", "Unconfigured_device");
    config->addOption("wifi_password", "Password of your WiFi", "wifi_password");
}
*/

void reset_buffer_file() {
    Serial.println("Create buffer file!");
    createDir(LittleFS, "/apt1220"); // Create a mydir folder
    //writeFile(LittleFS, "/apt1220/aptbuffer.txt", ""); // Create a hello1.txt file with the content "Hello1"
    writeFile(LittleFS, bufferFilePath, "");
}

void initializeHardware() {
    Wire.begin();
    delay(100);
    rtc2.begin();
    SerialC.begin(9600, SERIAL_8N1, 36, 14);
    SerialD.begin(9600, SERIAL_8N1, 32, 33);
}

void initializeFileSystem() {
    // Pokusíme se pøipojit LittleFS, formátovat pokud sele
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        Serial.println("LittleFS Mount Failed initially.");

        // --- Pøidáno: Explicitní pokus o formátování ---
        Serial.println("Attempting to format LittleFS...");
        if (LittleFS.format()) {
            Serial.println("LittleFS formatted successfully.");
            // Zkusíme pøipojit znovu po formátování
            if (!LittleFS.begin()) {
                Serial.println("LittleFS Mount Failed even after format!");
                return; // Ukonèíme, pokud ani po formátu nelze pøipojit
            }
            Serial.println("LittleFS Mounted successfully after format.");
        }
        else {
            Serial.println("LittleFS format failed.");
            return; // Ukonèíme, pokud formátování sele
        }
        // --- Konec pøidaného kódu ---

    }
    else {
        Serial.println("LittleFS Mount Successful");
    }


    if (!LittleFS.exists(bufferFilePath)) {
        reset_buffer_file();
    }
    else
    {
        Serial.print("Buffer file exists: ");
        Serial.println(bufferFilePath);

        File bufferFile = LittleFS.open(bufferFilePath, FILE_READ); // Otevøeme soubor pro ètení
        if (bufferFile) {
            size_t fileSize = bufferFile.size(); // Získáme velikost souboru
            Serial.print(" -> Size: ");
            Serial.print(fileSize);
            Serial.println(" bytes");
            bufferFile.close(); // Zavøeme soubor
        }
        else {
            Serial.println(" -> Error: Failed to open buffer file to get size.");
        }

    }
}

void initializeNetwork() {
    Serial.println("--- Initializing Network ---");
    WiFi.onEvent(WiFiEvent);
    bool wifiConnected = false;
    bool ethInitOk = false;

    if (useWifi) {
        connectToWiFi();
        wifiConnected = (WiFi.status() == WL_CONNECTED);
    }

    if (useETH) {
        if (restartNetwork) {
            ETH.end();
            delay(500);
        }

        if (ETH.begin()) {
            ethInitOk = true;

            if (!useDHCP) {
                Serial.println("Configuring static IP for ETH...");
                IPAddress local_IP, gateway, subnet, primaryDNS;

                local_IP.fromString(ip_adr);
                gateway.fromString(ip_gate);
                subnet.fromString(ip_mask);
                primaryDNS.fromString(ip_server);

                if (!ETH.config(local_IP, gateway, subnet, primaryDNS)) {
                    Serial.println("ETH: Failed to configure static IP!");
                }
            }

            // Logování stavu po inicializaci
            // Poèkáme chvilku, aby se stihly eventy zpracovat a DHCP priradit IP
            delay(1000);

            Serial.println("ETH Interface Status:");
            Serial.print("  MAC: ");
            Serial.println(ETH.macAddress());
            Serial.print("  IP Address: ");
            Serial.println(ETH.localIP());
            Serial.print("  Subnet Mask: ");
            Serial.println(ETH.subnetMask());
            Serial.print("  Gateway: ");
            Serial.println(ETH.gatewayIP());
            Serial.print("  DNS Server: ");
            Serial.println(ETH.dnsIP());
            Serial.print("  Link Status: ");
            Serial.println(ETH.linkUp() ? "UP" : "DOWN");
            Serial.print("  Link Speed: ");
            Serial.println(String(ETH.linkSpeed()) + " Mbps");

        }
        else {
            Serial.println("FATAL: ETH.begin() failed!");
        }
    }

    // eth_connected se nastavuje globálnì v eventu ARDUINO_EVENT_ETH_GOT_IP
    // Dáme mu chvilku, aby se to stihlo stát
    unsigned long startAttempt = millis();
    while (!eth_connected && useETH && (millis() - startAttempt < 5000)) {
        delay(100); // Poèkáme a 5 sekund na event, e jsme dostali IP
    }

    // AP Mode spustíme jen pokud selhala WiFi i Ethernet
    if (!wifiConnected && !eth_connected) {
        Serial.println("No network connection, starting AP Mode.");
        startAPMode();
    }
    else {
        Serial.println("Network connection established, connecting to server...");
        connectToServer();
    }
    Serial.println("--- Network Initialization Finished ---");
}

void initializeTasks() {
    xTaskCreatePinnedToCore(tSEC_TIMERcode, "tSEC_TIMER", 4096, NULL, 0, &tSEC_TIMER, 1);
    xTaskCreatePinnedToCore(tLAST_PINGcode, "tLAST_PING", 4096, NULL, 0, &tLAST_PING, 1);
}

void initializeDisplay() {
    lcd2.init();
    lcd2.backlight();
    lcd2.createChar(0, customChar);
    lcd2.setAutoNewLine(false); // Vypne automatické posouvání na novı øádek
    lcd2.clear();
    delay(200);
    lcd2.printf("       eMISTR       ");
    lcd2.setCursor(0, 2);
    lcd2.printf("Verze: %s", VERZE_PRACANT);
    lcd2.setCursor(0, 3);
    lcd2.printf("Verze FW: %s", localVersion);
    delay(200);
    //rtc2.setHourMode(CLOCK_H24);
    //rtc2.startClock();
}

void loadConfiguration() {
    if (!load_config()) {
        set_default();
        save_config();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // ================== ZMÌNA START ==================
    // KROK 1: INICIALIZACE SYNCHRONIZAÈNÍCH PRIMITIV
    // Zavoláme naši novou funkci hned na zaèátku. Tím zajistíme,
    // e všechny mutexy a fronty budou pøipravené pro tasky,
    // které se spustí pozdìji v setupu.
    initializeSyncPrimitives();
    // =================== ZMÌNA KONEC ===================

    //rtc2.begin();
    //rtc2.set_12hour_mode(false);
    rtc2.setHourMode(CLOCK_H24);
    //rtc2.setYear(2021);

    if (!rtc2.isRunning()) {
        Serial.println("RTC is NOT running, let's set the time!");
        //rtc2.startClock();
        //rtc2.begin(Wire); // adjust(DateTime(2014, 1, 21, 3, 0, 0));
        rtc2.startClock();
    }

    initializeHardware();
    initializeFileSystem();


    ///vypnutí pouití EEPROM a EspConfigLib
    //setup_inifile();

    ///vypnutí pouití EEPROM a EspConfigLib
    //eeprom.eeprom_read(240, &loaded_default);

    delay(100);

    loadConfiguration();

    initializeNetwork();

    // Kontrola aktualizací ji pøi spuštìní
    //checkForUpdates();

    initializeTasks();

    // Inicializace OTA
    setupOTA();

    initializeDisplay();


    // ================== ZMÌNA START ==================
    // Pùvodní volání pro vytváøení mutexù a fronty odsud maeme,
    // protoe jsme je pøesunuli do funkce initializeSyncPrimitives().

    /* PÙVODNÍ KÓD K ODSTRANÌNÍ:
    // Create mutex for safe LCD access
    demoMutex = xSemaphoreCreateMutex();

    // Vytvoøíme mutex pro ochranu sdílenıch I2C periferií
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("Chyba pri vytvareni I2C mutexu!");
        // Tady bychom mohli tøeba zastavit program, protoe bez toho to nebude fungovat
    }

    tcpMutex = xSemaphoreCreateMutex(); // <-- PØIDEJ TENTO ØÁDEK
    if (tcpMutex == NULL) {
        Serial.println("Chyba pri vytvareni TCP mutexu!");
    }
    */
    // =================== ZMÌNA KONEC ===================

    /*
        // Create the task that will handle the demo function
        xTaskCreate(
            tDEMOcode,          // Function to implement the task
            "DemoTask",        // Name of the task
            2048,              // Stack size in words
            NULL,              // Task input parameter
            1,                 // Priority of the task
            &tDEMO    // Task handle
        );
    */

    /*
          char c_string[128] = "abcdefghijklmnopqrstuvwxyz1234567890!@#$%^&*()ABCDEFGHIJKLMNOPQ";
          int string_length = strlen(c_string);

          if (!eeprom.eeprom_write(33, (byte *) c_string, strlen(c_string))) {
          Serial.println("Failed to store STRING");
          } else {
          Serial.println("STRING correctly stored");
          }

          eeprom.eeprom_read(40, (byte *) c_string2,16);
        */

        //Serial.println((String)loaded_default);
        //Serial.println(activeMAC);
        //Serial.println(useWifi ? "WIFI connect" : "ETH connect");

    rfid_c_last_time = 0;
    rfid_d_last_time = 0;
    rfid_c_last[0] = 0;
    rfid_d_last[0] = 0;

    net_on = 0;
    saved = 0;
    ping_timeout = 4;

    fifo_start = 0;
    fifo_end = fifo_start + off_buffer_size;
    fifo_first = fifo_start;
    fifo_last = fifo_start;

    if ((timer1 > 30) || (timer1 < 1)) timer1 = 15;
    if ((timer2 > 30) || (timer2 < 1)) timer2 = 3;

    //timeout1 = 999999999;
    timeout1 = 0;
    //timeout1 = millis();
    timer_reset = SEC_TIMER + 86400;
    // Denní reset 

    //rtc2.startClock();

    // ================== ZMÌNA START ==================
    // Pùvodní volání pro vytváøení fronty odsud maeme.

    /* PÙVODNÍ KÓD K ODSTRANÌNÍ:
    // Vytvoøíme frontu. Mùe pojmout a 10 zpráv typu SerialData_t.
    // Pokud by èteèky posílaly data rychleji, ne je stíháme zpracovat,
    // fronta se zaplní a další odeslání do ní poèká.
    serialDataQueue = xQueueCreate(10, sizeof(SerialData_t));

    // Zkontrolujeme, jestli se fronta úspìšnì vytvoøila
    if (serialDataQueue == NULL) {
        Serial.println("Chyba pri vytvareni fronty!");
    }
    */
    // =================== ZMÌNA KONEC ===================


    // Vytvoøíme task pro zpracování pøíkazù z fronty
    xTaskCreatePinnedToCore(
        commandProcessorTask,   // Funkce tasku
        "CommandProcessor",     // Jméno
        4096,                   // Stack
        NULL,                   // Parametry
        2,                      // Priorita (dáme jí vyšší ne ètecím taskùm)
        NULL,                   // Handle
        1                       // Spustíme na jádøe 1
    );

    // Vytvoøíme task pro ètení ze SerialC
    xTaskCreatePinnedToCore(
        serialReaderTask,       // Funkce tasku
        "SerialC_Reader",       // Jméno
        4096,                   // Stack
        (void*)&SerialC,        // Parametr - pøedáme ukazatel na SerialC
        1,                      // Priorita
        NULL,                   // Handle
        1                       // Spustíme také na jádøe 1
    );

    // Vytvoøíme task pro ètení ze SerialD
    xTaskCreatePinnedToCore(
        serialReaderTask,       // Funkce tasku
        "SerialD_Reader",       // Jméno
        4096,                   // Stack
        (void*)&SerialD,        // Parametr - pøedáme ukazatel na SerialD
        1,                      // Priorita
        NULL,                   // Handle
        1                       // Spustíme také na jádøe 1
    );

#ifdef DEBUG_MODE
    // Start the web server in debug mode
    server.on("/", HTTP_GET, handleRoot);
    server.on("/scan", HTTP_GET, handleScanNetworks);
    server.on("/save", HTTP_POST, handleSaveCredentials);
    server.begin();
    Serial.println("Web server started in debug mode.");
#endif
}

/**
 * @brief Task pro ètení dat ze sériového portu.
 * Tato funkce bìí v nekoneèné smyèce a je pouitelná pro oba sériové porty.
 * Jako parametr dostane ukazatel na HardwareSerial objekt.
 */
void serialReaderTask(void* parameter) {
    HardwareSerial* serialPort = (HardwareSerial*)parameter;
    int portNumber = (serialPort == &SerialC) ? 1 : 2;
    SerialData_t receivedData; // Vytvoøíme si strukturu pro data

    for (;;) { // Nekoneèná smyèka tasku
        if (serialPort->available()) {
            size_t len = serialPort->readBytesUntil('\n', receivedData.data, sizeof(receivedData.data) - 1);
            if (len > 0) {
                receivedData.data[len] = '\0'; // Ukonèíme øetìzec
                receivedData.port = portNumber;

                // Pošleme data do fronty. Pokud je fronta plná, poèkáme max 100ms.
                if (xQueueSend(serialDataQueue, &receivedData, pdMS_TO_TICKS(100)) != pdTRUE) {
                    Serial.printf("CHYBA: Fronta pro seriová data je plná, data z portu %d zahozena!\n", portNumber);
                }
            }
        }
        // Dáme plánovaèi šanci, aby se mohl vìnovat jinım úlohám.
        // Task se na 20ms uspí a neere zbyteènì CPU.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Task pro zpracování dat z fronty.
 * Èeká, a se ve frontì objeví data, a pak je zpracuje voláním funkce serial().
 */
void commandProcessorTask(void* parameter) {
    SerialData_t dataToProcess;

    for (;;) {
        // Èekáme na data ve frontì. Task je blokován (neere CPU), dokud nìco nepøijde.
        // portMAX_DELAY znamená, e bude èekat "vìènì".
        if (xQueueReceive(serialDataQueue, &dataToProcess, portMAX_DELAY) == pdTRUE) {
            // Máme nová data, zavoláme tvou pùvodní funkci pro zpracování
            Serial.printf("Data z fronty (port %d): %s\n", dataToProcess.port, dataToProcess.data);
            serial(dataToProcess.data, dataToProcess.port);
        }
    }
}

// "Hloupá" verze - jen èistí displej, nestará se o mutex
void _fast_clear_disp_unsafe() {
    lcd2.clear();
    lcd2.home();
}

// "Chytrá" verze - wrapper, kterı se stará o bezpeènost
void fast_clear_disp() {
    // Vezmeme klíè
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

        _fast_clear_disp_unsafe(); // Zavoláme "hloupou" funkci

        // Vrátíme klíè
        xSemaphoreGive(i2cMutex);
    }
}

void _blank_line_unsafe(int line) {
    lcd2.setCursor(0, line);                       // Umístí kurzor na zaèátek øádku
    lcd2.print("                    ");           // Vypíše prázdnı øetìzec o 20 mezerách
    lcd2.setCursor(0, line);                       // Umístí kurzor zpìt na zaèátek øádku
}

// "Chytrá" verze - teï se stará o zamykání
void blank_line(int line) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        _blank_line_unsafe(line); // Volá hloupou verzi
        xSemaphoreGive(i2cMutex);
    }
}

/**
 * @brief Bezpeènì zapíše data do TCP klienta. Stará se o zamèení a odemèení.
 * @param data Ukazatel na data k odeslání.
 * @return true pokud se odeslání podaøilo, jinak false.
 */
bool tcp_print_safe(const char* data) {
    bool success = false;
    // Poèkáme si na klíè (mutex)
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (client.connected()) {
            client.print(data);
            success = true;
        }
        // Vrátíme klíè
        xSemaphoreGive(tcpMutex);
    }
    else {
        Serial.println("CHYBA: tcp_print_safe nemohl ziskat TCP mutex!");
    }
    return success;
}

/**
 * @brief Bezpeènì se pøipojí k TCP serveru.
 */
void connectToServer_safe() {
    // Vezmeme si klíè
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Serial.println("Pokousim se pripojit k serveru...");
        if (client.connect(serverIP, serverPort)) {
            setNetOn(1, 12);
            SEC_TIMER = millis() / 1000;
            last_ping = SEC_TIMER;
            Serial.println("Pripojeno k serveru.");
        }
        else {
            setNetOn(0, 13);
            Serial.println("Pripojeni k serveru selhalo.");
        }
        // Vrátíme klíè
        xSemaphoreGive(tcpMutex);
    }
    else {
        Serial.println("CHYBA: connectToServer_safe nemohl ziskat TCP mutex!");
    }
}

boolean load_config() {
    //naètení konfigurace z Preferences
    preferences.begin("my-app", true);
    //sprintf(ip_adr, preferences.getString("ip_adr", "192.168.222.202").c_str());
    //sprintf(ip_mask, preferences.getString("ip_mask", "255.255.255.0").c_str());
    //sprintf(ip_server, preferences.getString("ip_server", "192.168.88.221").c_str());

	snprintf(ip_adr, sizeof(ip_adr), "%s", preferences.getString("ip_adr", "192.168.222.202").c_str());
	snprintf(ip_mask, sizeof(ip_mask), "%s", preferences.getString("ip_mask", "255.255.255.0").c_str());
	//snprintf(ip_server, sizeof(ip_server), "%s", preferences.getString("ip_server", "192.168.88.221").c_str());
    snprintf(ip_server, sizeof(ip_server), "%s", preferences.getString("ip_server", "192.168.225.221").c_str());
    snprintf(ip_gate, sizeof(ip_gate), "%s", preferences.getString("ip_gate", "192.168.222.222").c_str());


    //sprintf(ip_server, preferences.getString("ip_server", "192.168.225.221").c_str());
    //sprintf(ip_gate, preferences.getString("ip_gate", "192.168.222.222").c_str());

    timer1 = preferences.getInt("timer1", 5);
    timer2 = preferences.getInt("timer2", 3);
    key_maker = preferences.getInt("key_maker", 0);
    rfid_reader_c = preferences.getInt("rfid_reader_c", 1);
    rfid_reader_d = preferences.getInt("rfid_reader_d", 0);
    id12_c = preferences.getInt("id12_c", 1);
    id12_d = preferences.getInt("id12_d", 0);
    //sprintf(op_c, preferences.getString("op_c", "A3997").c_str());
    //sprintf(op_d, preferences.getString("op_d", "").c_str());

	snprintf(op_c, sizeof(op_c), "%s", preferences.getString("op_c", "A3997").c_str());
	snprintf(op_d, sizeof(op_d), "%s", preferences.getString("op_d", "").c_str());

    useWifi = preferences.getBool("useWifi", false);
    useETH = preferences.getBool("useETH", true);

	useDHCP = preferences.getBool("useDHCP", true);

    //sprintf(ssid, preferences.getString("SSID_NAME", "AGERIT_AC 2GHz").c_str());
    //sprintf(password, preferences.getString("SSID_PASS", "AGERITagerit512").c_str());

	snprintf(ssid, sizeof(ssid), "%s", preferences.getString("SSID_NAME", "AGERIT_AC 2GHz").c_str());
	snprintf(password, sizeof(password), "%s", preferences.getString("SSID_PASS", "AGERITagerit512").c_str());

    ping_timeout = 4;
    preferences.end();
    serverIP = ip_server;

    return true;
}

boolean save_config() {
    //uloení konfigurace do Preferences
    preferences.begin("my-app", false);
    preferences.putString("ip_adr", ip_adr);
    preferences.putString("ip_mask", ip_mask);
    preferences.putString("ip_server", ip_server);
    preferences.putString("ip_gate", ip_gate);
    preferences.putString("op_c", op_c);
    preferences.putString("op_d", op_d);
    //sprintf(ip_adr,   preferences.getString("ip_adr",   "192.168.222.202").c_str());
    //sprintf(ip_mask,  preferences.getString("ip_mask",  "255.255.255.0").c_str());
    //sprintf(ip_server,preferences.getString("ip_server","192.168.225.221").c_str());
    //sprintf(ip_gate,  preferences.getString("ip_gate",  "192.168.222.222").c_str());
    //sprintf(op_c,preferences.getString("op_c",  "A3997").c_str());
    //sprintf(op_d,preferences.getString("op_d",  "").c_str());  

    preferences.putInt("timer1", timer1);
    preferences.putInt("timer2", timer2);
    preferences.putInt("key_maker", key_maker);
    preferences.putInt("rfid_reader_c", rfid_reader_c);
    preferences.putInt("rfid_reader_d", rfid_reader_d);
    preferences.putInt("id12_c", id12_c);
    preferences.putInt("id12_d", id12_d);
    //timer1 = preferences.getInt("timer1",5);
    //timer2 = preferences.getInt("timer2",3);
    //key_maker = preferences.getInt("key_maker",0);
    //rfid_reader_c = preferences.getInt("rfid_reader_c",1);
    //rfid_reader_d = preferences.getInt("rfid_reader_d",0);
    //id12_c = preferences.getInt("id12_c",1);
    //id12_d = preferences.getInt("id12_d",0);

    preferences.putBool("useWifi", useWifi);
    preferences.putBool("useETH", useETH);
	preferences.putBool("useDHCP", useDHCP);

    preferences.putString("SSID_NAME", ssid);
    preferences.putString("SSID_PASS", password);

    preferences.end();
    return true;
}

void set_default() {
    //sprintf(ip_adr, "192.168.222.202");
    //sprintf(ip_mask, "255.255.255.0");
    //sprintf(ip_server, "192.168.225.221");
    //sprintf(ip_gate, "192.168.222.222");

    snprintf(ip_adr, sizeof(ip_adr), "192.168.222.202");
	snprintf(ip_mask, sizeof(ip_mask), "255.255.255.0");
	snprintf(ip_server, sizeof(ip_server), "192.168.225.221");
	snprintf(ip_gate, sizeof(ip_gate), "192.168.222.222");

    timer1 = 5;
    timer2 = 3;
    key_maker = 0;
    rfid_reader_c = 1;
    rfid_reader_d = 0;
    id12_c = 1;
    id12_d = 0;
    //sprintf(op_c, "A3997");
    //sprintf(op_d, "");

	snprintf(op_c, sizeof(op_c), "A3997");
	snprintf(op_d, sizeof(op_d), "");

    useDHCP = true;
    ping_timeout = 4;

    loaded_default = 1;
    ///vypnutí pouití EEPROM a EspConfigLib
    //eeprom.eeprom_write(240, &loaded_default);
    saveNewConfigData = true;
}

void init_lcd() {
    lcd2.init();
    lcd2.clear();                             // Initialize the LCD
    lcd2.backlight();                         // Turn on the LCD backlight
    lcd2.noAutoscroll();
}

void menu_dump() {
    char long_str[20];  // Vyrovnávací pamì pro pøevedená èísla
            
    lockI2C();  // Zámek I2C sbìrnice pro bezpeènı pøístup k LCD
	_fast_clear_disp_unsafe(); // Vyèištìní displeje

    // Zobrazení hodnoty fifo_start
    lcd2.setCursor(0, 0);
    lcd2.printf("START:");
    ltoa(fifo_start, long_str, 10);  // Pøevede `fifo_start` na øetìzec
    lcd2.printf(long_str);

    // Zobrazení hodnoty fifo_end
    lcd2.setCursor(0, 1);
    lcd2.printf("END:");
    ltoa(fifo_end, long_str, 10);
    lcd2.printf(long_str);

    // Zobrazení hodnoty fifo_first
    lcd2.setCursor(0, 2);
    lcd2.printf("FIRST:");
    ltoa(fifo_first, long_str, 10);
    lcd2.printf(long_str);

    // Zobrazení hodnoty fifo_last
    lcd2.setCursor(0, 3);
    lcd2.printf("LAST:");
    ltoa(fifo_last, long_str, 10);
    lcd2.printf(long_str);
	unlockI2C();  // Uvolnìní I2C zámku
}

void menuconfig() {

    lockI2C();  // Zámek I2C sbìrnice pro bezpeènı pøístup k LCD
    long timeout_up;
	_fast_clear_disp_unsafe(); // Vyèištìní displeje
    lcd2.setCursor(0, 0);
    lcd2.printf("IP:");
    //lcd2.printf(ip_adr);

    if (useDHCP) {
        char ip_adr2[16];
        if (useETH) { strcpy(ip_adr2, ETH.localIP().toString().c_str()); }
        else { strcpy(ip_adr2, WiFi.localIP().toString().c_str()); }
        lcd2.printf(ip_adr2);
		//delete ip_adr2; // Uvolnìní pamìti, pokud bylo pouito dynamické alokování
    }
    else {
        lcd2.printf(ip_adr);
    }

    lcd2.setCursor(0, 1);
    lcd2.printf("MASK:");
    lcd2.printf(ip_mask);
    lcd2.setCursor(0, 2);
    lcd2.printf("SRV:");
    lcd2.printf(ip_server);
    lcd2.setCursor(0, 3);
    lcd2.printf("VERZE: ");
    //lcd2.printf(" %s  ", VERZE_PRACANT);
    lcd2.printf(" %s  ", localVersion);
	unlockI2C();  // Uvolnìní I2C zámku
}

void menuconfigwifi() {
    lockI2C();  // Zámek I2C sbìrnice pro bezpeènı pøístup k LCD
    long timeout_up;
	_fast_clear_disp_unsafe(); // Vyèištìní displeje

    lcd2.setCursor(0, 0);
    lcd2.printf("SSID:");
    lcd2.printf(ssid);
    lcd2.setCursor(0, 3);
    lcd2.printf("VERZE: ");
    //lcd2.printf(" %s  ", VERZE_PRACANT);
    lcd2.printf(" %s  ", localVersion);
	unlockI2C();  // Uvolnìní I2C zámku
}

void resetConfig() {
    // Zavøete existující TCP pøipojení
    if (client.connected()) {
        client.stop();
        Serial.println("TCP client disconnected");
    }

    // Restart WiFi pøipojení
    WiFi.disconnect(true); // Odpojí a zapomene pøipojení
    delay(1000);           // Poèkejte na odpojení

    // Nastavení statické IP (pokud je poadována statická IP)
    if (!WiFi.config(IPAddress(ip_adr), IPAddress(ip_gate), IPAddress(ip_mask))) {
        Serial.println("STA Failed to configure");
    }

    Serial.println(useWifi ? "WIFI connect" : "ETH connect");

    // Pøipojte se znovu k WiFi síti
    if (useWifi) {
        connectToWiFi();
    }

    // Starth Ethernet (this does NOT start WiFi at the same time)
    if (useETH) {
        Serial.print("Starting ETH interface...");
        ETH.begin();
    }

    // Inicializace FIFO promìnnıch
      //fifo_start = malloc(ALOC_MEM);
      //fifo_end =fifo_start+ALOC_MEM;

    fifo_first = fifo_start;
    fifo_last = fifo_start;

    // Ovìøení a nastavení èasovaèù
    if (timer1 > 60 || timer1 < 1) timer1 = 10;
    if (timer2 > 60 || timer2 < 1) timer2 = 5;

    // Otevøete nové TCP pøipojení
    //connectToServer();
    connectToServer_safe();
	Serial.println("TCP client reinitialized");
    /*
      if (client.connect(ip_server, serverPort)) {
          Serial.println("Reconnected to TCP server");
      } else {
          Serial.println("Failed to reconnect to TCP server");
      }
    */
}

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer pro nová data
    char buffer2[100];       // Buffer pro sériovı vstup

    // ================== ZMÌNA START ==================
    // TOTO JE TEN TRIK:
    // Nastavíme timeout pro pøekreslení hlavní obrazovky na dalekou budoucnost (tøeba hodinu).
    // Tím zajistíme, e zatímco my tady èekáme na vstup, funkce loop() nezavolá tDEMOscreen()
    // a nepokusí se nám sáhnout na I2C sbìrnici.
    timeout1 = SEC_TIMER + 3600;
    // =================== ZMÌNA KONEC ===================

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        _fast_clear_disp_unsafe();
        delay(10);

        lcd2.setCursor(0, 0);
        lcd2.printf("%s", display_str);
        lcd2.setCursor(0, 1);
        lcd2.printf("%s", data_to_fill);
        lcd2.setCursor(0, 2);
        lcd2.printf("new:");

        delay(10);
        while (true) {
            bool input_processed = false;

            if (SerialC.available() > 0) {
                int len = SerialC.readBytesUntil('\n', buffer2, sizeof(buffer2) - 1);
                buffer2[len] = '\0';
                String buffer2String = buffer2;
                buffer2String.trim();

                if (buffer2String.equals("STORNO")) { break; }
                if (buffer2String.equals("OK")) {
                    strncpy(data_to_fill, new_data, sizeof(new_data) - 1);
                    data_to_fill[sizeof(new_data) - 1] = '\0';
                    saveNewConfigData = true;
                    break;
                }
                if (buffer2String.equals("DEFAULT")) {
                    if (strlen(new_data) > 0) {
                        new_data[strlen(new_data) - 1] = '\0';
                    }
                }
                else {
                    strncat(new_data, buffer2String.c_str(), sizeof(new_data) - strlen(new_data) - 1);
                }
                input_processed = true;
            }

            if (SerialD.available() > 0) {
                int len = SerialD.readBytesUntil('\n', buffer2, sizeof(buffer2) - 1);
                buffer2[len] = '\0';
                String buffer2String = buffer2;
                buffer2String.trim();

                if (buffer2String.equals("STORNO")) { break; }
                if (buffer2String.equals("OK")) {
                    strncpy(data_to_fill, new_data, sizeof(new_data) - 1);
                    data_to_fill[sizeof(new_data) - 1] = '\0';
                    saveNewConfigData = true;
                    break;
                }
                if (buffer2String.equals("DEFAULT")) {
                    if (strlen(new_data) > 0) {
                        new_data[strlen(new_data) - 1] = '\0';
                    }
                }
                else {
                    strncat(new_data, buffer2String.c_str(), sizeof(new_data) - strlen(new_data) - 1);
                }
                input_processed = true;
            }

            // Pøekreslíme zadanı text jen pokud pøišlo nìco nového
            if (input_processed) {
                lcd2.setCursor(4, 2);
                lcd2.print("                "); // Vymaeme starı text
                lcd2.setCursor(4, 2);
                lcd2.print(new_data);
            }

            // Dáme šanci ostatním taskùm, ale ne moc velkou, a je odezva sviná.
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        // Na konci vrátíme mutex, tak jak to bylo v pùvodní funkèní verzi.
        xSemaphoreGive(i2cMutex);
    }
    else {
        Serial.println("Chyba: reader_input nemohl ziskat I2C mutex!");
    }

    // ================== ZMÌNA START ==================
    // Po opuštìní funkce nastavíme timeout na nulu, abychom vynutili
    // okamité pøekreslení hlavní obrazovky s pøípadnımi novımi daty.
    timeout1 = 0;
    // =================== ZMÌNA KONEC ===================
}

/*

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer pro nová data
    String buffer2;          // Pomocnı buffer pro sériovı vstup

    fast_clear_disp();       // Vymazání LCD displeje

    delay(10);

    lcd2.setCursor(0, 0);
    lcd2.printf("%s", display_str);

    lcd2.setCursor(0, 1);
    lcd2.printf("%s", data_to_fill);

    lcd2.setCursor(0, 2);
    lcd2.printf("new:");

    delay(10);
    while (true) {
        if (SerialC.available() > 0) {
            //if (Serial.available() > 0) {
                //buffer2 = Serial.readStringUntil('\n');  // Ètení vstupu do bufferu do znaku nového øádku
            buffer2 = SerialC.readStringUntil('\n');  // Ètení vstupu do bufferu do znaku nového øádku

            buffer2.trim();  // Odstranìní bílıch znakù

            if (buffer2.equals("STORNO")) {
                break;  // Pøerušení pøi pøíkazu "STORNO"
            }
            else if (buffer2.equals("OK")) {
                strncpy(data_to_fill, new_data, sizeof(new_data));  // Uloení novıch dat
                break;
            }
            else if (buffer2.equals("DEFAULT")) {
                // Odstranìní posledního znaku
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                // Pøidání vstupu do `new_data` a zobrazení na LCD
                strncat(new_data, buffer2.c_str(), sizeof(new_data) - strlen(new_data) - 1);
                lcd2.printf("%s", buffer2.c_str());
            }
        }
    }
}

*/

void serial(char* buffer2, int port) {
    /* void serial(const char* buffer2, int port) {  */
    char buffer[100] = { 0 };
    char tmp[100] = { 0 };
    int counter1;
    char tmp2[100] = { 0 };

    char first_char_code[2] = { 0 }; // Pomocnı buffer pro první znak

    /*
        char buffer2_copy[100];
        strncpy(buffer2_copy, buffer2, sizeof(buffer2_copy) - 1);
        buffer2_copy[sizeof(buffer2_copy) - 1] = '\0';  // Zajištìní null-terminace

        // Nyní pracujte s `buffer2_copy` místo `buffer2`
        if (buffer2_copy[strlen(buffer2_copy) - 1] == '\n') {
            buffer2_copy[strlen(buffer2_copy) - 1] = '\0';
        }
    */
    // Odstranìní koncovıch znakù `\n`
    if (buffer2[strlen(buffer2) - 1] == '\n') {
        buffer2[strlen(buffer2) - 1] = '\0';
    }

    logToSerial(buffer2, 0);

    String buffer2String = buffer2;
    buffer2String.trim();

    if ((millis() / 1000) >= timeout1) { _fast_clear_disp_unsafe(); }

    // Zpracování rùznıch pøíkazù
    //if (strcmp(buffer2String, "SET-IP") == 0) {
    if (buffer2String.equals("SET-IP")) {
        if (useDHCP) {
            char ip_adr2[16];
            if (useETH) { strcpy(ip_adr2, ETH.localIP().toString().c_str()); }
            else { strcpy(ip_adr2, WiFi.localIP().toString().c_str()); }
          reader_input("Nastaveni IP DHCP", ip_adr2);
		  strcpy(ip_adr, ip_adr2);
		  restartNetwork = true; // Nastavíme flag pro restart sítì
          //snprintf(buffer, sizeof(buffer), "IP: %s\n", ip_adr2);
          //Serial.print(buffer);
		  //lcd2.printf("IP: %s\n", ip_adr2);
        }
        else {
          reader_input("Nastaveni IP", ip_adr);
          restartNetwork = true; // Nastavíme flag pro restart sítì
		}
        
		Serial.printf("IP: %s\n", ip_adr);
        //} else if (strcmp(buffer2String, "SET-SERV") == 0) {
    }
    else if (buffer2String.equals("SET-SERV")) {
        reader_input("Nastaveni SERVERU", ip_server);
        //} else if (strcmp(buffer2String, "SET-OP-C") == 0) {
    }
    else if (buffer2String.equals("SET-OP-C")) {
        reader_input("RFID C", op_c);
        //} else if (strcmp(buffer2, "SET-OP-D") == 0) {
    }
    else if (buffer2String.equals("SET-OP-D")) {
        reader_input("RFID D", op_d);
        //} else if (strcmp(buffer2, "SET-GATE") == 0) {
    }
    else if (buffer2String.equals("SET-GATE")) {
        reader_input("Nastaveni BRANY", ip_gate);
        restartNetwork = true; // Nastavíme flag pro restart sítì
        //} else if (strcmp(buffer2, "SET-MASK") == 0) {
    }
    else if (buffer2String.equals("SET-MASK")) {
        reader_input("Nastaveni Masky", ip_mask);
        restartNetwork = true; // Nastavíme flag pro restart sítì
        //} else if (strcmp(buffer2, "FILL") == 0) {
    }
    else if (buffer2String.equals("FILL")) {
        if (key_maker == 0) {
            for (counter1 = 0; counter1 <= 500; counter1++) {
                if (net_on == 0) {
                    snprintf(buffer, sizeof(buffer), "B%d\n", counter1);
                    serial(buffer, port);
                    snprintf(buffer, sizeof(buffer), "A%d\n", counter1);
                    serial(buffer, port);
                    snprintf(buffer, sizeof(buffer), "D%d\n", counter1);
                    serial(buffer, port);
                }
            }
        }
        else {
            /*
            blank_line(0);
            lcd2.printf("FUNKCE NEPODPOROVANA");
            */
			display_line_unformated("FUNKCE NEPODPOROVANA", 0);
            timeout1 = SEC_TIMER + timer2;
        }
        //} else if (strcmp(buffer2, "SET-CLOCK") == 0) {
    }
    else if (buffer2String.equals("SET-CLOCK")) {
        snprintf(buffer, sizeof(buffer), "\x03~");
        //client.print(buffer);
        tcp_print_safe(buffer);
        //} else if (strcmp(buffer2, "SET-TIM1") == 0) {
    }
    else if (buffer2String.equals("SET-TIM1")) {
        snprintf(buffer, sizeof(buffer), "%d", timer1);
        reader_input("Nastaveni TIMEOUT 1", buffer);
        timer1 = atoi(buffer);
        //} else if (strcmp(buffer2, "SET-TIM2") == 0) {
    }
    else if (buffer2String.equals("SET-TIM2")) {
        snprintf(buffer, sizeof(buffer), "%d", timer2);
        reader_input("Nastaveni TIMEOUT 2", buffer);
        timer2 = atoi(buffer);
        //} else if (strcmp(buffer2, "DEFAULT") == 0) {
    }
    else if (buffer2String.equals("DEFAULT")) {
        set_default();
        //} else if (strcmp(buffer2, "LCDINIT") == 0) {
    }
    else if (buffer2String.equals("LCDINIT")) {
        init_lcd();
        //} else if (strcmp(buffer2, "MEMINFO") == 0) {
    }
    else if (buffer2String.equals("MEMINFO")) {
        menu_dump();
        timeout1 = SEC_TIMER + timer2;
        //} else if (strcmp(buffer2, "CONFIG") == 0 || strcmp(buffer2, "SHOWCONFIG") == 0) {
    }
    else if (buffer2String.equals("CONFIG") || buffer2String.equals("SHOWCONFIG")) {
        menuconfig();
        resetConfig();
        timeout1 = SEC_TIMER + timer2;
        //} else if (strcmp(buffer2, "SET-KEY") == 0) {
    }
    else if (buffer2String.equals("SET-KEY")) {
        key_maker = 1;
        //} else if (strcmp(buffer2, "SET-RFIDC") == 0) {
    }
    else if (buffer2String.equals("SET-RFIDC")) {
        rfid_reader_c = 1;
        //} else if (strcmp(buffer2, "SET-RFIDD") == 0) {
    }
    else if (buffer2String.equals("SET-RFIDD")) {
        rfid_reader_d = 1;
        //} else if (strcmp(buffer2, "ID12-C") == 0) {
    }
    else if (buffer2String.equals("ID12-C")) {
        id12_c = 1;
        //} else if (strcmp(buffer2, "ID12-D") == 0) {
    }
    else if (buffer2String.equals("ID12-D")) {
        id12_d = 1;
        //} else if (strcmp(buffer2, "SET-PRACANT") == 0) {
    }
    else if (buffer2String.equals("SET-PRACANT")) {
        key_maker = 0;
    }
    else if (buffer2String.equals("SET-SAVE")) {
        boolean saved_config = save_config();
        delay(200);
        Serial.println(useWifi ? "WIFI connect" : "ETH connect");
        ESP.restart();
    }
    else if (buffer2String.equals("SET-LOAD")) {
        load_config();
    }
    else if (buffer2String.equals("SET-DHCP")) {
        useDHCP = true;
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítì
    }
    else if (buffer2String.equals("SET-STATIC")) {
        useDHCP = false;
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítì
        }
    else if (buffer2String.equals("SET-WIFI")) {
        useWifi = true;
        useETH = false;
        saveNewConfigData = true;
    }
    else if (buffer2String.equals("SET-ETH")) {
        useWifi = false;
        useETH = true;
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítì
    }
    else if (buffer2String.equals("SET-SSID")) {
        //SSID
        reader_input("Nastavení SSID", ssid);
    }
    else if (buffer2String.equals("SET-PASSW")) {
        //SSID PASSWORD 
        reader_input("Nastavení hesla", password);
    }
    else if (buffer2String.equals("SHOW-NET")) {
        menuconfig();
        timeout1 = SEC_TIMER + timer2;
    }
    else if (buffer2String.equals("SHOW-WIFI")) {
		menuconfigwifi();
        timeout1 = SEC_TIMER + timer2;
    }
    else if (buffer2String.equals("RESET-BUFFER")) {
        reset_buffer_file();
        /*
        blank_line(0);
        lcd2.printf("MAZU OFFLINE BUFFER");
        */
		display_line_unformated("MAZU OFFLINE BUFFER", 0);
        timeout1 = SEC_TIMER + timer2;
    }
    else {
        timeout1 = SEC_TIMER + timer1;
        if (saved) {
            _fast_clear_disp_unsafe();
            saved = 0;
        }

        char prefix = buffer2[0];
        const char* code_body = buffer2 + 1; // Zbytek øetìzce za prefixem

        // Nová logika pro zobrazení na displeji v OFFLINE reimu
        if (!net_on) {
            switch (prefix) {
            case 'A': // Operace
                display_line_formated(buffer2String.c_str(), 0);
                break;
            case 'B': // Zakázka
                display_line_formated(buffer2String.c_str(), 1);
                break;
            case 'D': // Pracovník
                display_line_formated(buffer2String.c_str(), 3);
                break;
            case 'G': // Pracovník
                display_line_formated(buffer2String.c_str(), 0);
                break;
            case 'H': // Mnoství
                display_line_formated(buffer2String.c_str(), 2);
                break;
                // Mùeme pøidat další, pokud je potøeba
            default:
                // Neznámı kód zobrazíme na posledním øádku jako døíve
                display_line_formated(buffer2, 3);
                break;
            }
        }

        // Zpracování na základì prvního znaku
        switch (buffer2[0]) {
        case 'A':
            //strcpy(tmp, "\x06");
			first_char_code[0] = '\x06'; // Uloení prvního znaku
            break;
        case 'B':
            //strcpy(tmp, "\x05");
			first_char_code[0] = '\x05'; // Uloení prvního znaku
            break;
        case 'D':
            if (key_maker == 1) {
                snprintf(tmp2, sizeof(tmp2), "\x06AOPEN\n");
                tcp_print_safe(tmp2);
            }
            first_char_code[0] = '\x04';
            break;
        case 'E':
            //strcpy(tmp, "\x08");
			first_char_code[0] = '\x08'; // Uloení prvního znaku
            break;
        case 'F':
            //strcpy(tmp, "\x07");
			first_char_code[0] = '\x07'; // Uloení prvního znaku
            break;
        case 'G':
            //strcpy(tmp, "\x10");
			first_char_code[0] = '\x10'; // Uloení prvního znaku
            break;
        case 'H':
            //strcpy(tmp, "\x11");
			first_char_code[0] = '\x11'; // Uloení prvního znaku
            break;
        case 'I':
            //strcpy(tmp, "\x12");
			first_char_code[0] = '\x12'; // Uloení prvního znaku
            break;
        case 'J':
            //strcpy(tmp, "\x13");
			first_char_code[0] = '\x13'; // Uloení prvního znaku
            break;
        case 'K':
            //strcpy(tmp, "\x19");
			first_char_code[0] = '\x19'; // Uloení prvního znaku
            break;
        case 'L':
            //strcpy(tmp, "\x20");
			first_char_code[0] = '\x20'; // Uloení prvního znaku
            break;
        default:
            //strcpy(tmp, "\x17");
			first_char_code[0] = '\x17'; // Uloení prvního znaku
            break;
        }

        //strcat(tmp, buffer2);

        //strcat(tmp, buffer2String.c_str());

        // Echo na LCD pøi offline stavu
        if (!net_on && ((long)(fifo_end - fifo_last) >= off_buffer_size)) {
            /*
            blank_line(2);
            //lcd2.printf("%s", buffer2);
            lcd2.printf("%s", buffer2String.c_str());
            */
			
			// Zobrazíme zprávu na LCD
			// ji není potøeba, protoe pouíváme display_line_formated podle prefixu 
            //display_line_formated(buffer2String.c_str(), 2);
        }

        // Pøidání èasu k pøíkazùm typu D, E, J atd.
        /*
        if (strchr("DEJ0123456789", buffer2[0])) {
            strcat(tmp, "~");
            //get_time(SEC_TIMER, buffer);
            get_time(buffer);
            strcat(tmp, buffer);
        }
        */
		// Pøidání èasu k pøíkazùm typu D, E, J atd.
        if (strchr("DEJ0123456789", buffer2[0])) {
            get_time(buffer, sizeof(buffer)); // buffer je pomocné pole, kam se uloí èas
            snprintf(tmp, sizeof(tmp), "%s%s~%s\n", first_char_code, buffer2String.c_str(), buffer);
        }
        else {
            snprintf(tmp, sizeof(tmp), "%s%s\n", first_char_code, buffer2String.c_str());
        }

        //strcat(tmp, "\n");
        
        if (net_on) {
            //client.print(tmp);
			tcp_print_safe(tmp); // Pouití bezpeèné funkce pro komunikaci pøes TCP
        }
        else {
            //strcat(off_buffer, tmp);
            // --- Kontrola velikosti PØED strcat (z minulé úpravy) ---
            if (strlen(off_buffer) + strlen(tmp) < sizeof(off_buffer)) {
                strcat(off_buffer, tmp); // Pøidat do off_buffer, jen pokud se vejde
            }
            else {
                Serial.println("CHYBA: off_buffer je plny, nelze pripojit zpravu:");
                Serial.print(tmp);
            }
            // --- Konec kontroly velikosti ---

            if (!net_on && strchr("D0123456789", buffer2[0])) {
                //&& ((long)(fifo_end - fifo_last) >= off_buffer_size))

                //lcd2.printf("%s", buffer2);

                //strncpy((char*)fifo_last, off_buffer, strlen(off_buffer));

                //appendFile(LittleFS, bufferFilePath, tmp);
                //appendFile(LittleFS, bufferFilePath, off_buffer);

                //fifo_last += strlen(off_buffer) + 1;

                // --- Pøidána kontrola velikosti souboru PØED zápisem ---
                bool can_write_to_file = false;
                size_t current_file_size = 0;
                size_t data_to_add_size = strlen(off_buffer); // Velikost dat, která chceme pøidat

                if (data_to_add_size > 0) { // Kontrolujeme jen pokud máme co zapisovat
                    File bufferFileRead = LittleFS.open(bufferFilePath, FILE_READ); // Otevøít jen pro ètení velikosti
                    if (bufferFileRead) {
                        current_file_size = bufferFileRead.size();
                        bufferFileRead.close(); // Hned zavøít

                        // Zkontrolujeme, zda se nová data vejdou pod limit
                        if (current_file_size + data_to_add_size < MAX_BUFFER_FILE_SIZE) {
                            can_write_to_file = true;
                        }
                        else {
                            // Pokud by pøidání dat pøekroèilo limit
                            Serial.printf("CHYBA: Soubor bufferu %s je plny (%u B). Nelze pridat %u B.\n",
                                bufferFilePath, (unsigned int)current_file_size, (unsigned int)data_to_add_size);

                            // Volitelnì: Zobrazit chybu i na LCD
                            /*
                            blank_line(0); // Vymaeme øádek 0
                            lcd2.setCursor(0, 0);
                            lcd2.print("CHYBA:SOUBOR PLNY!");
                            */

							display_line_formated("CHYBA: SOUBOR PLNY!", 0);
                            timeout1 = SEC_TIMER + timer2; // Necháme zprávu viditelnou
                        }
                    }
                    else {
                        Serial.printf("CHYBA: Nelze otevrit soubor %s pro kontrolu velikosti.\n", bufferFilePath);
                        // Pokud nelze zjistit velikost, radìji nezapisujeme
                        can_write_to_file = false;
                    }
                }
                else {
                    // Není co zapisovat (off_buffer je prázdnı)
                    Serial.println("DEBUG: off_buffer je prazdny, neni co zapisovat do souboru.");
                    can_write_to_file = false; // Není potøeba volat appendFile
                }
                // --- Konec kontroly velikosti souboru ---

                // --- Zápis do souboru POUZE pokud kontrola prošla ---
                if (can_write_to_file) {
                    appendFile(LittleFS, bufferFilePath, off_buffer); // Pøidá celı obsah off_buffer na konec souboru

                    // Zobrazit "ULOZENO DO SOUBORU" na LCD
                    lcd2.setCursor(0, 0);
                    lcd2.printf("*------------------*");
                    lcd2.setCursor(0, 1);
                    lcd2.printf("|     ULOZENO      |");
                    lcd2.setCursor(0, 2);
                    lcd2.printf("|   DO SOUBORU     |"); // Opravenı text
                    lcd2.setCursor(0, 3);
                    lcd2.printf("*------------------*");
                    saved = 1;
                    timeout1 = SEC_TIMER + timer2;

                    memset(off_buffer, 0x00, sizeof(off_buffer));
                }
                // --- Konec zápisu do souboru ---
            }
        }
    }

    if (saveNewConfigData) {
        save_config(); // Uloení novıch dat do konfigurace
        saveNewConfigData = false; // Reset flagu po uloení
	}

    if (restartNetwork) {
		//ESP.restart(); // Restart ESP32 pro aplikaci zmìn sítì
		/*
        blank_line(3); // Vymaeme øádek 3
		lcd2.printf("RESTART SITE...");
        */

		display_line_unformated("RESTART SITE...", 3);

		initializeNetwork(); // Volání funkce pro inicializaci sítì
		restartNetwork = false; // Reset flagu po restartu sítì
    }

}

void tDEMOscreen() {

    // Zkusíme si vzít klíè (mutex). Pokud je obsazenı, task tady poèká.
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    char buffer2[100] = { 0 };

    // Zobrazení informací na LCD
    if (key_maker == 0) {
        lcd2.setCursor(0, 0);
        //lcd2.printf("   eMISTR ESP32    ");
        String tmpConnType = " eMISTR 2025   ";
        tmpConnType += useWifi ? "WIFI" : " ETH ";
        lcd2.printf(tmpConnType.c_str());
    }
    if (key_maker == 1) {
        lcd2.setCursor(0, 0);
        lcd2.printf(" eMISTR 2025 DVERE ");
    }

    lcd2.setCursor(0, 1);
    lcd2.printf("--------------------");

    if (efect == 38) efect = 0;
    if (efect > 19) lcd2.setCursor(38 - efect, 1);
    else lcd2.setCursor(efect, 1);
    lcd2.printf("*");
    efect++;

    lcd2.setCursor(0, 2);
    //print_time(SEC_TIMER, buffer2);

	//DateTime now = rtc2.isConnected() ? rtc2.getDateTime() : DateTime(SEC_TIMER); // Pouijeme RTC pokud je pøipojeno, jinak pouijeme aktuální èas z millis()  
    //String nowStr = now.timestamp(DateTime::TIMESTAMP_FULL);

    //lcd2.print(nowStr);

	//lcd2.print(rtc2.getDateTimeString()); // Zobrazíme aktuální èas z RTC
    print_time(rtc2.getEpoch(), buffer2, sizeof(buffer2));

    //logToSerial(rtc2.getDateTimeString() , 1);
    //logToSerial(nowStr.c_str(), 1);
    //logToSerial(buffer2,1);

    //print_time(rtc2.now().unixtime(), buffer2);

    lcd2.printf(buffer2);
    lcd2.setCursor(0, 3);

    /*
        lcd2.printf(net_on ? "      ON-LINE       " : "OFF-LINE");

        if (!net_on) {
            if ((long)(fifo_end - fifo_last) < off_buffer_size) {
                lcd2.printf(" PLNA Pamet! ");
            }
            else {
                int volno = ((fifo_end - fifo_last) * 100) / off_buffer_size;
                lcd2.printf(" volno: %d%% ", volno);
            }
        }
    */

    if (net_on) {
        if (isSendingFileBuffer) {
            lcd2.print(" ODESILAM SOUBOR... "); // Indikace probíhajícího odesílání
        }
        else {
            lcd2.print("      ON-LINE       "); // Online a nic neodesíláme
        }
    }
    else { // Offline - zobrazíme vyuití souboru
        File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
        if (bufferFile) {
            size_t fileSize = bufferFile.size();
            bufferFile.close();

            // Vıpoèet procent (bezpeènı pro dìlení nulou, pokud by MAX_BUFFER_FILE_SIZE byla 0)
            int usage_percent = 0;
            if (MAX_BUFFER_FILE_SIZE > 0) {
                // Pouijeme 64bit int pro násobení, abychom pøedešli pøeteèení pøed dìlením
                usage_percent = (int)(((uint64_t)fileSize * 100) / MAX_BUFFER_FILE_SIZE);
            }
            if (usage_percent > 100) usage_percent = 100; // Omezení na max 100%

            // Formátování vıpisu, napø. "OFF-LINE Buf:  15% "
            char statusStr[21];
            snprintf(statusStr, sizeof(statusStr), "OFF-LINE  Volno:%3d%%", 100-usage_percent);
            // Doplnìní mezerami do konce øádku
            for (int i = strlen(statusStr); i < 20; ++i) statusStr[i] = ' ';
            statusStr[20] = '\0';
            lcd2.print(statusStr);

        }
        else {
            Serial.printf("tDEMOscreen: Failed to open buffer file '%s' for reading size.\n", bufferFilePath);

            // --- Pokus o zotavení: Zkontrolovat existenci a pøípadnì vytvoøit ---
            if (!LittleFS.exists(bufferFilePath)) {
                // Soubor skuteènì neexistuje
                Serial.println("tDEMOscreen: Buffer file does not exist. Attempting recreation...");
                lcd2.print("OFF-LINE (NEW FILE)"); // Indikace pokusu o vytvoøení na LCD
                reset_buffer_file(); // Zavoláme funkci, která vytvoøí adresáø a prázdnı soubor
            }
            else {
                // Soubor existuje, ale pøesto nešel otevøít -> Pravdìpodobnì chyba FS
                Serial.println("tDEMOscreen: Buffer file exists but cannot be opened (FS Error?). Displaying error.");
                lcd2.print("OFF-LINE (File ERR)"); // Zobrazíme pùvodní chybovou hlášku
            }
            // --- Konec pokusu o zotavení ---
        }
    }
    // Práce s LCD je hotová, vrátíme klíè, aby ho mohl pouít nìkdo jinı.
    xSemaphoreGive(i2cMutex);
    }
 else {
     // Nepodaøilo se získat mutex do 1 sekundy, nìco je špatnì.
     Serial.println("Chyba: tDEMOscreen nemohl ziskat I2C mutex!");
    }
}

void lockI2C() {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
}

void unlockI2C() {
    xSemaphoreGive(i2cMutex);
}

void loop() {
    // Pokud jsme online, zkusíme zpracovat TCP komunikaci a pøípadnì odeslat data z offline bufferu.
    if (net_on) {
        TCP(); // Zpracování pøíchozích TCP dat od serveru.

        // Pokud zrovna neprobíhá velké odesílání souboru, zkontrolujeme, jestli nìjakı soubor na odeslání nemáme.
        if (!isSendingFileBuffer) {
            File bufferFileCheck = LittleFS.open(bufferFilePath, FILE_READ);
            bool fileHasData = (bufferFileCheck && bufferFileCheck.size() > 0);
            if (bufferFileCheck) bufferFileCheck.close();

            if (fileHasData) {
                isSendingFileBuffer = true;  // Nastavíme flag, e se chystáme odesílat.
                send_entire_file_buffer();   // Pokusíme se odeslat celı buffer.
                isSendingFileBuffer = false; // Po dokonèení pokusu flag zase shodíme.
            }
        }
    }

    // Zobrazíme hlavní obrazovku, pokud vypršel její timeout.
    if ((millis() / 1000) >= timeout1) {
        tDEMOscreen();
    }

    // Malá pauza, aby loop() nebìela naprázdno a dala prostor ostatním taskùm.
    // V projektu s FreeRTOS je to dobrá praxe.
    delay(5);
}


/*
void tDEMOcode(void* parameter) {
    for (;;) {
        tDEMOrun();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
*/

void tDEMOcode(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100ms interval

    while (true) {
        // Acquire mutex before accessing the LCD
        if (xSemaphoreTake(demoMutex, portMAX_DELAY) == pdTRUE) {
            //tDEMOrun();
            if ((millis() / 1000) >= timeout1) { tDEMOscreen(); }
            // Release the mutex
            xSemaphoreGive(demoMutex);
        }

        // Wait for the next cycle (100ms)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void tDEMOrun() {
    //hlavní obrazovka
    // Buffer pro zobrazení èasu na LCD
    char buffer2[100] = { 0 };

    // Zobrazení informací na LCD
    if (key_maker == 0) {
        lcd2.setCursor(0, 0);
        lcd2.printf("   eMISTR 2025    ");
    }
    if (key_maker == 1) {
        lcd2.setCursor(0, 0);
        lcd2.printf(" eMISTR 2025 DVERE ");
    }

    lcd2.setCursor(0, 1);
    lcd2.printf("--------------------");

    if (efect == 38) efect = 0;
    if (efect > 19) lcd2.setCursor(38 - efect, 1);
    else lcd2.setCursor(efect, 1);
    lcd2.printf("*");
    efect++;

    lcd2.setCursor(0, 2);
    //print_time(SEC_TIMER, buffer2);
    //print_time(rtc.getEpoch(), buffer2);

    //print_time(rtc2.getEpoch(), buffer2);
	get_time(buffer2, sizeof(buffer2));

    // 
    //print_time(rtc2.now().unixtime(), buffer2);

    lcd2.print(buffer2);
    lcd2.setCursor(0, 3);
    lcd2.printf(net_on ? "      ON-LINE       " : "OFF-LINE");

    if (!net_on) {
        if ((long)(fifo_end - fifo_last) < off_buffer_size) {
            lcd2.printf(" PLNA Pamet! ");
        }
        else {
            int volno = ((fifo_end - fifo_last) * 100) / off_buffer_size;
            lcd2.printf(" volno: %d%% ", volno);
        }
    }
}

void logToSerial(const char* vMessage, int pLogLevel) {
    if ((vSerialDebug) && (pLogLevel >= vLogLevel)) { Serial.println(vMessage); }
}

void logToSerial(const String vMessage, int pLogLevel) {
    if ((vSerialDebug) && (pLogLevel >= vLogLevel)) { Serial.println(vMessage); }
}

void connectToWiFi() {
    setHostname(); // Set the hostname before connecting to WiFi
    WiFi.begin(ssid, password);

    int retryAttempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        ++retryAttempt;
        if (retryAttempt > 4) {
            Serial.println("Failed to connect to WiFi.");
            return;
        }
    }
    Serial.println("WiFi connected");
}

void connectToServer() {
    if (client.connect(serverIP, serverPort)) {
        setNetOn(1, 12);
        SEC_TIMER = millis() / 1000;
        last_ping = SEC_TIMER;
        Serial.println("Connected to server");
    }
    else {
        setNetOn(0, 13);
        Serial.println("Connection to server failed");
        //startAPMode(); // Start AP mode if server connection fails
    }
}

// React to Ethernet events:
void WiFiEvent(WiFiEvent_t event)
{
    switch (event) {

    case ARDUINO_EVENT_ETH_START:
        // This will happen during setup, when the Ethernet service starts
        Serial.println("ETH Started");
        //set eth hostname here
        //ETH.setHostname("esp32-ethernet");
        setHostname();
        //ETH.setHostname( ETH.macAddress().c_str() );
        break;

    case ARDUINO_EVENT_ETH_CONNECTED:
        // This will happen when the Ethernet cable is plugged 
        Serial.println("ETH Connected");
        break;

    case ARDUINO_EVENT_ETH_GOT_IP:
        // This will happen when we obtain an IP address through DHCP:
        Serial.print("Got an IP Address for ETH MAC: ");
        Serial.print(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.print(ETH.localIP());
        if (ETH.fullDuplex()) {
            Serial.print(", FULL_DUPLEX");
        }
        Serial.print(", ");
        Serial.print(ETH.linkSpeed());
        Serial.println("Mbps");
        eth_connected = true;

        activeMAC = ETH.macAddress();
        activeMAC.replace(":", "-");
        ETH.setHostname(activeMAC.c_str());
        //ETH.hostname = activeMAC;

        // Uncomment to automatically make a test connection to a server:
        // testClient( "192.168.0.1", 80 );

        break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
        // This will happen when the Ethernet cable is unplugged 
        Serial.println("ETH Disconnected");
        eth_connected = false;
        break;

    case ARDUINO_EVENT_ETH_STOP:
        // This will happen when the ETH interface is stopped but this never happens
        Serial.println("ETH Stopped");
        eth_connected = false;
        break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("WIFI IPv4: ");
        Serial.println(WiFi.localIP().toString());
        Serial.print("Got an IP Address for WIFI MAC: ");
        Serial.print(WiFi.macAddress());
        break;

        //    case ARDUINO_EVENT_WIFI_STA_START:
        //      WiFi.setHostname( strcat("APTW-",WiFi.macAddress().c_str()) );
        //      break;

    default:
        break;
    }
}

void get_time(unsigned long thetime, char* buff, size_t buff_size) {
    struct tm thetm;

    // Pouíváme gmtime_r pro bezpeèné konvertování epochálního èasu na strukturovanı èas
    gmtime_r((time_t*)&thetime, &thetm);

    // Vytvoøení formátovaného øetìzce s datem a èasem
    /*
    sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d",
        //1900 + thetm.tm_year,    // Rok zaèíná od 1900
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok zaèíná od 1900
        thetm.tm_mon + 1,        // Mìsíce jsou indexovány od 0
        thetm.tm_mday,           // Den v mìsíci
        thetm.tm_hour,           // Hodiny
        thetm.tm_min,            // Minuty
        thetm.tm_sec             // Sekundy
    );
    */

    snprintf(buff, buff_size, "%04d-%02d-%02d %02d:%02d:%02d",
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok zaèíná od 1900
        thetm.tm_mon + 1,        // Mìsíce jsou indexovány od 0
        thetm.tm_mday,           // Den v mìsíci
        thetm.tm_hour,           // Hodiny
        thetm.tm_min,            // Minuty
        thetm.tm_sec             // Sekundy
	);
}

void get_time(char* buff, size_t buff_size) {
    snprintf(buff, buff_size, "%04d-%02d-%02d %02d:%02d:%02d",
        rtc2.getYear(),
        rtc2.getMonth(),
        rtc2.getDay(),
        rtc2.getHours(),
        rtc2.getMinutes(),
        rtc2.getSeconds()
    );
}

// PÙVODNÍ, JEDNODUCHÁ VERZE BEZ MUTEXÙ
void print_time(unsigned long thetime, char* buff, size_t buff_size) {
    snprintf(buff, buff_size, "%02d.%02d.%04d  %02d:%02d:%02d",
        rtc2.getDay(),
        rtc2.getMonth(),
        rtc2.getYear(),
        rtc2.getHours(),
        rtc2.getMinutes(),
        rtc2.getSeconds()
    );
}



void setNetOn(int vNetOn, int vCallFromId) {
    //if (vSerialDebug) {Serial.println("net_on = "+String(vNetOn) + " from: "+String(vCallFromId));}
    if (vNetOn == 0) { logToSerial("net_on = " + String(vNetOn) + " from: " + String(vCallFromId), 1); }
    net_on = vNetOn;
}

void tLAST_PINGcode(void* parameter) {
    for (;;) {
        // Zkusíme odeslat ping pøes naši novou bezpeènou funkci.
        // Ta v sobì kontroluje, jestli je klient pøipojenı a stará se o mutex.
        if (tcp_print_safe("\x01\n")) {
            // Pokud funkce vrátila true, znamená to, e odeslání probìhlo.
            setNetOn(1, 15);
        }
        else {
            // Pokud vrátila false, odeslání se nepovedlo.
            // Nejspíš nejsme pøipojeni.
            setNetOn(0, 16);
            // Zkusíme se tedy znovu bezpeènì pøipojit.
            connectToServer_safe();
        }

        // Poèkáme a opakujeme.
        vTaskDelay(pdMS_TO_TICKS(1900));
    }
}

void tSEC_TIMERcode(void* parameter) {
    for (;;) {

        //SEC_TIMER = rtc2.getEpoch();
        //SEC_TIMER = rtc2.now().unixtime();

        SEC_TIMER = millis() / 1000;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




void TCP() {
    char buffer[80] = { 0 };
    char buffer2[80] = { 0 };
    struct tm thetm = {};

    // --- Kontrola stavu pøipojení ---
    SEC_TIMER = millis() / 1000;
    if (net_on == 1 && (long)(SEC_TIMER - last_ping) > ping_timeout) {
        if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            client.stop();
            xSemaphoreGive(tcpMutex);
        }
        Serial.println("TCP Timeout: " + String(SEC_TIMER) + "/" + String(last_ping));
        setNetOn(0, 7);
    }

    // --- Reconnect, pokud jsme "on", ale socket není pøipojen ---
    bool is_connected;
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        is_connected = client.connected();
        xSemaphoreGive(tcpMutex);
    }
    else { return; }

    if (!is_connected && net_on == 1) {
        connectToServer_safe();
    }

    if (!is_connected) return;

    // Pokud nejsou ádná data, nic nedìláme a resetujeme si pamì posledního pøíkazu
    if (client.available() == 0) {
        g_last_tcp_command_code = 0;
        return;
    }

    // --- Ètení dat z klienta ---
    while (client.available() > 0) {
        size_t bytes_read = 0;
        if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (client.connected() && client.available()) {
                bytes_read = client.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            }
            xSemaphoreGive(tcpMutex);
        }
        else {
            logToSerial("TCP: mutex timeout on read", 1);
            break;
        }

        if (bytes_read == 0) continue;
        buffer[bytes_read] = '\0';

        last_ping = millis() / 1000;
        logToSerial(buffer, 0);

        if ((buffer[0] == 'O') && (buffer[1] == 'K')) {
            setNetOn(1, 14);
            g_last_tcp_command_code = 0; // 'OK' není pøíkaz pro displej, resetujeme
            continue;
        }

        String buffer2String = buffer + 2;
        buffer2String.trim();

        switch (buffer[0]) {
        case 2: {
            int year, month, day, hour, minute, second;
            int items_parsed = sscanf(buffer + 2, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);
            if (items_parsed == 6) { rtc2.stopClock(); rtc2.setDate(day, month, year); rtc2.setTime(hour, minute, second); rtc2.startClock(); }
        } break;
        case 3: { get_time(SEC_TIMER, buffer2, sizeof(buffer2)); snprintf(buffer, sizeof(buffer), "\x02~%s\n", buffer2); tcp_print_safe(buffer); } break;
        case 1: { tcp_print_safe("OK\n"); } break;
        case 4: display_line_formated(buffer2String.c_str(), 3); break;
        case 5: display_line_formated(buffer2String.c_str(), 1); break;
        case 6: display_line_formated(buffer2String.c_str(), 0); break;
        case 7: display_line_formated(buffer2String.c_str(), 2); break;
        case 8: display_line_formated(buffer2String.c_str(), 0); break;
        case 0x10: display_line_formated(buffer2String.c_str(), 0); break;
        case 0x11: display_line_formated(buffer2String.c_str(), 2); break;
        case 18: display_line_formated(buffer2String.c_str(), 0); break;
        case 19: display_line_formated(buffer2String.c_str(), 1); break;
        case 0xB: { timer1 = atoi(buffer + 2); } break;
        case 0xC: { timeout1 = 0; } break;

        case 0xE: {
            char line_buffer[21];
            int msg_len = buffer2String.length();
            if (msg_len > 18) { msg_len = 18; }
            int padding_left = (18 - msg_len) / 2;
            int padding_right = 18 - msg_len - padding_left;
            snprintf(line_buffer, sizeof(line_buffer), "|%*s%.*s%*s|", padding_left, "", msg_len, buffer2String.c_str(), padding_right, "");

            if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
                // ============== ZDE JE TA MAGIE ==============
                // Pokud pøedchozí pøíkaz nebyl také rámeèek, vykreslíme ho celı.
                if (g_last_tcp_command_code != 0xE) {
                    lcd2.setCursor(0, 0); lcd2.printf("*------------------*");
                    lcd2.setCursor(0, 2); lcd2.printf("*------------------*");
                }
                // Ale text na prostøedním øádku aktualizujeme VDY.
                lcd2.setCursor(0, 1);
                lcd2.print(line_buffer);
                xSemaphoreGive(i2cMutex);
            }

            saved = 1;
            timeout1 = SEC_TIMER + timer2;
        } break;
        case 0x18: { display_line_unformated("Zamek odjisten", 0); saved = 1; timeout1 = SEC_TIMER + timer2; } break;
        default: break;
        }

        // Na konci zpracování si zapamatujeme kód aktuálního pøíkazu pro pøíští smyèku.
        g_last_tcp_command_code = buffer[0];
    }
}


void display_line_formated(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se dìje uvnitø jednoho zamèení
        _blank_line_unsafe(linenumber);
        //lcd2.setCursor(0, linenumber);
        lcd2.printf("%s",message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu
    }
}

void display_line_unformated(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se dìje uvnitø jednoho zamèení
        _blank_line_unsafe(linenumber);
        //lcd2.setCursor(0, linenumber);
        lcd2.printf(message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu
    }
}

void display_temporary_status(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se dìje uvnitø jednoho zamèení
        _blank_line_unsafe(linenumber);
        lcd2.setCursor(0, linenumber);
        lcd2.printf(message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu

        vTaskDelay(pdMS_TO_TICKS(2000)); // Poèkám

        // Znovu si zamknu, abych po sobì uklidil
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            _blank_line_unsafe(linenumber);
            xSemaphoreGive(i2cMutex);
        }
    }
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);

    if (!file) { file = fs.open(path, FILE_WRITE, true); }

    if (!file) {
        Serial.println("- failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("- file written");
    }
    else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS& fs, const char* path, const char* message) {
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("- failed to open file for appending");
        return;
    }
    if (file.print(message)) {
        Serial.println("- message appended");
    }
    else {
        Serial.println("- append failed");
    }
    file.close();
}


//************************************************************************
//************************************************************************
// ANALYZE RECIVED DATA ..................
//************************************************************************
//************************************************************************
void send_buffer() {
    char tmp[100];
    char tmp_send_data[50];
    char j[10];
    long i, k;
    int respons;
    int set_first;
    int efect;
    float proc;

    efect = 1;

    // Zobrazení stavu pøenosu na LCD
    lcd2.setCursor(0, 0);
    lcd2.printf("*------------------*");
    lcd2.setCursor(0, 1);
    lcd2.printf("|  PRENASIM DATA   |");
    lcd2.setCursor(0, 2);
    lcd2.printf("| DO PC - CEKEJTE  |");
    lcd2.setCursor(0, 3);
    lcd2.printf("*------------------*");

    // Pøíprava pro odeslání dat
    if (!client.connected() || client.write("\x06A-100\n") != 7) {
        //net_on = 0;
        setNetOn(0, 3);
        return;
    }
    client.flush();
    delay(1000);

    timeout2 = SEC_TIMER + 2;  // Timeout na odpovìï
    tmp[0] = '\0';

    // Èekání na odpovìï od serveru
    while (((long)(SEC_TIMER - timeout2) <= 0) && (tmp[3] != '?')) {
        tmp[0] = '\0';
        server_read(tmp);
    }

    if (tmp[3] == '?') {
        timeout2 = SEC_TIMER + 30;  // Zakázání pøepisu displeje
        last_ping = SEC_TIMER;
        long l = fifo_first;

        // Odeslání dat, dokud je sí pøipojena a existují data
        while ((fifo_first + 3 < fifo_last) && net_on) {
            memset(off_buffer, 0x00, off_buffer_size);
            // Simulace naètení dat do `off_buffer`
            // xmem2root(off_buffer, fifo_first, off_buffer_size);

            tmp_send_data[0] = '\0';
            i = k = 0;
            set_first = 0;

            // Vıbìr a formátování dat pro odeslání
            while ((off_buffer[i] != 0x0A) && (i <= 45)) {
                j[0] = off_buffer[i];
                j[1] = '\0';

                if (j[0] == 0x04) {
                    set_first = 1;
                }
                strcat(tmp_send_data, j);
                i++;
                k++;
            }

            strcat(tmp_send_data, "\n");
            k++;

            // Kontrola pøipojení a odeslání dat
            if ((long)(SEC_TIMER - last_ping) > ping_timeout) setNetOn(0, 4);//net_on = 0;

            if (net_on == 1) {
                if (client.connected() && client.write(tmp_send_data, strlen(tmp_send_data))) {
                    client.flush();
                    delay(100);

                    respons = 0;
                    while (respons != 1 && net_on) {
                        if ((long)(SEC_TIMER - last_ping) > ping_timeout) {
                            //net_on = 0;
                            setNetOn(0, 5);
                        }

                        if (server_read(tmp) > 0) {
                            tmp[strlen(tmp) - 1] = '\0';  // Odstranìní znaku '\n'
                            if (tmp[1] == 0x7E) {
                                respons = 1;
                                last_ping = SEC_TIMER;
                            }
                        }
                    }

                    // Aktualizace FIFO
                    if ((respons && net_on) || (i > 45)) {
                        if (set_first) {
                            l = fifo_first + k;
                        }
                        fifo_first = fifo_first + k;
                    }
                    else {
                        fifo_first = l;
                    }
                }
                else {
                    //net_on = 0;
                    setNetOn(0, 6);
                }
            }

            // Zobrazení pokroku na LCD2
            lcd2.setCursor(0, 0);
            lcd2.printf("*------------------*");
            lcd2.setCursor(15, 0);
            int percentage2 = ((fifo_last - fifo_start) != 0) ?
                ((fifo_first - fifo_start) * 100) / (fifo_last - fifo_start) : 100;
            lcd2.printf("%d%%", percentage2);

            lcd2.setCursor(0, 3);
            lcd2.printf("*------------------*");
            lcd2.setCursor(efect > 18 ? 37 - efect : efect, 3);
            lcd2.printf("*");

            efect = (efect == 37) ? 1 : efect + 1;
        }

        // Uvolnìní pamìti FIFO
        if ((long)(fifo_first + 3 >= fifo_last)) {
            fifo_last = fifo_start;
            fifo_first = fifo_start;
        }
    }
    else {
        // Reset spojení pøi neúspìchu
        client.stop();
        net_on = 0;
        delay(200);
    }
}

int server_read(char* buffer) {
    int len = 0;
    char tmp[2] = { 0 };  // Vyrovnávací pamì pro jednotlivé znaky
    unsigned long timeout = SEC_TIMER + 2;
    bool ok = false;
    *buffer = '\0';  // Inicializace bufferu na prázdnı øetìzec

    while (!ok && ((long)(SEC_TIMER - timeout) <= 0)) {
        // Èekejte na data
        if (client.available()) {
            len = client.readBytes(tmp, 1);  // Pøeètìte jeden znak
            tmp[1] = '\0';  // Ukonèovací znak pro správné spojení s øetìzcem
            if (len > 0) {
                strcat(buffer, tmp);
                // Pokud narazíme na novı øádek, ukonèujeme ètení
                if (tmp[0] == '\n') {
                    ok = true;
                }
            }
        }

        // Pokud ádná data nepøichází, ukonèíme smyèku
        if (strlen(buffer) == 0) {
            ok = true;
        }
    }

    return strlen(buffer);  // Vrací délku naètenıch dat
}

void createDir(fs::FS& fs, const char* path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path)) {
        Serial.println("Dir created");
    }
    else {
        Serial.println("mkdir failed");
    }
}

void startAPMode() {
    // Check if SoftAP is already running
    if (WiFi.softAPgetStationNum() > 0) {
        Serial.println("SoftAP is already running.");
        return;
    }

    // Get the MAC address
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", ""); // Remove colons from MAC address

    // Create the SSID using the MAC address
    String ssidAP = "APT1220_WIFI_" + macAddress;

    // Start the SoftAP with the generated SSID
    if (WiFi.softAP(ssidAP.c_str(), "12345678")) {
        Serial.println("AP mode started. SSID: " + ssidAP);
        Serial.println("IP address: " + WiFi.softAPIP().toString());

        // Start the web server
        server.on("/", HTTP_GET, handleRoot);
        server.on("/scan", HTTP_GET, handleScanNetworks);
        server.on("/save", HTTP_POST, handleSaveCredentials);
        server.begin();
        Serial.println("Web server started.");
    }
    else {
        Serial.println("Failed to start AP mode.");
    }
}


void handleRoot(AsyncWebServerRequest* request) {
    String html = "<html><body>";
    html += "<h1>Available Networks</h1>";
    html += "<form action='/save' method='POST'>";
    html += "<label for='ssid'>SSID:</label><br>";
    html += "<input type='text' id='ssid' name='ssid'><br>";
    html += "<label for='password'>Password:</label><br>";
    html += "<input type='password' id='password' name='password'><br><br>";
    html += "<input type='checkbox' id='useWifi' name='useWifi' value='true'>";
    html += "<label for='useWifi'> Use WiFi</label><br><br>";
    html += "<input type='submit' value='Save'>";
    html += "</form>";
    html += "</body></html>";
    request->send(200, "text/html", html);
}


void handleScanNetworks(AsyncWebServerRequest* request) {
    int n = WiFi.scanNetworks();
    String html = "<html><body>";
    html += "<h1>Available Networks</h1>";
    html += "<ul>";
    for (int i = 0; i < n; ++i) {
        html += "<li>" + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + "dBm)</li>";
    }
    html += "</ul>";
    html += "<a href='/'>Back</a>";
    html += "</body></html>";
    request->send(200, "text/html", html);
}

void handleSaveCredentials(AsyncWebServerRequest* request) {
    String newSSID = request->arg("ssid");
    String newPassword = request->arg("password");
    bool useWifi = request->hasArg("useWifi");

    if (newSSID.length() > 0 && newPassword.length() > 0) {
        preferences.begin("my-app", false);
        preferences.putString("SSID_NAME", newSSID);
        preferences.putString("SSID_PASS", newPassword);
        preferences.putBool("useWifi", useWifi);
        preferences.end();

        request->send(200, "text/html", "<html><body><h1>Credentials Saved. Restarting...</h1></body></html>");
        delay(2000);
        ESP.restart();
    }
    else {
        request->send(400, "text/html", "<html><body><h1>Invalid Input</h1></body></html>");
    }
}

void setHostname() {
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", ""); // Remove colons from MAC address

    if (macAddress.startsWith("00000000")) {
        macAddress = WiFi.macAddress();
        macAddress.replace(":", ""); // Remove colons from MAC address      
    }

    String hostname = "APT_" + macAddress;
    WiFi.setHostname(hostname.c_str());
    ETH.setHostname(hostname.c_str());
}

void checkForUpdates() {
    Serial.println("Kontrola aktualizací...");

    // Staení souboru s verzí z serveru
    HTTPClient http;
    // Explicitnì zaèínáme HTTP spojení se specifikací portu a timeoutu
    http.begin(versionURL);
    http.setTimeout(10000); // Nastavení delšího timeoutu (10 sekund)

    // Pøidání hlavièek pro simulaci bìného browseru
    http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
    http.addHeader("Accept", "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8");
    http.addHeader("Accept-Language", "cs,en-US;q=0.7,en;q=0.3");
    http.addHeader("Connection", "keep-alive");

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String serverVersion = http.getString();
        serverVersion.trim(); // odstraní bílé znaky
        Serial.print("Verze na serveru: ");
        Serial.println(serverVersion);
        Serial.print("Verze na lokální: ");
        Serial.println(localVersion);

        String localVersionTmp = localVersion;
        localVersionTmp.replace(".", "");
        String serverVersionTmp = serverVersion;
        serverVersionTmp.replace(".", "");

        int localVersionInt = localVersionTmp.toInt();
        int serverVersionInt = serverVersionTmp.toInt();

        if (serverVersionInt > localVersionInt) {
            Serial.println("Nová verze nalezena, zahajuji aktualizaci...");

            // Vytvoøíme novı HTTPClient pro staení aktualizace
            HTTPClient updateClient;

            // Pøidání hlavièek pro simulaci bìného browseru i pro update request
            updateClient.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
            updateClient.addHeader("Accept", "*/*");
            updateClient.addHeader("Accept-Language", "cs,en-US;q=0.7,en;q=0.3");
            updateClient.addHeader("Connection", "keep-alive");
            updateClient.setTimeout(30000); // Delší timeout pro stahování firmware (30 sekund)

            Serial.print("Pøipojuji se k: ");
            Serial.println(firmwareURL);

            // Zkusíme nejprve pøímou metodu s pouitím WiFiClient místo HTTPClient
            WiFiClient client;

            Serial.println("Pokouším se o aktualizaci pøes WiFiClient...");
            t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

            // Pokud první metoda sele, zkusíme pùvodní metodu s HTTPClient
            if (ret == HTTP_UPDATE_FAILED) {
                Serial.println("První metoda selhala, zkouším alternativní metodu...");
                ret = httpUpdate.update(updateClient, String(firmwareURL));
            }

            switch (ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("Aktualizace selhala: %d - %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                // Dodateèné debugování
                Serial.print("URL firmware: ");
                Serial.println(firmwareURL);
                break;
            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("Není k dispozici ádná nová aktualizace.");
                break;
            case HTTP_UPDATE_OK:
                Serial.println("Aktualizace probìhla úspìšnì.");
                break;
            }
        }
        else {
            Serial.println("Firmware je aktuální.");
        }
    }
    else {
        Serial.printf("Nelze získat verzi, HTTP kód: %d\n", httpCode);
    }
    http.end();
}



// Funkce, která bìí jako samostatné vlákno
void otaUpdateTask(void* parameter) {
    Serial.println("OTA task started");

    while (true) {
        // Kontrola dostupnosti aktualizace
        if (!otaInProgress) {
            checkForUpdatesBackground();
        }

        // Pokud je aktualizace dostupná, proveï ji podle schématu URL
        if (otaUpdateAvailable && !otaInProgress) {
            Serial.print("Firmware URL pro aktualizaci: ");
            Serial.println(firmwareURL);

            if (firmwareURL != NULL && strlen(firmwareURL) > 7) { // Základní kontrola platnosti URL
                if (strncmp(firmwareURL, "https://", 8) == 0) {
                    Serial.println("Detekováno HTTPS, volám performUpdateHTTPS().");
                    performUpdateHTTPS();
                }
                else if (strncmp(firmwareURL, "http://", 7) == 0) {
                    Serial.println("Detekováno HTTP, volám performUpdate().");
                    performUpdate(); // Vaše existující funkce pro HTTP
                }
                else {
                    Serial.println("CHYBA: firmwareURL má neznámé schéma nebo je neplatná!");
                    otaUpdateAvailable = false; // Resetuj pøíznak, aby se to neopakovalo hned
                    // otaInProgress zùstává false
                }
            }
            else {
                Serial.println("CHYBA: firmwareURL je NULL nebo pøíliš krátká!");
                otaUpdateAvailable = false; // Resetuj pøíznak
            }
        }

        // Poèkej na další kontrolu.
        // POZNÁMKA: 60000 ms je 1 MINUTA, ne 1 hodina. Pro 1 hodinu pouijte 3600000 ms.
        vTaskDelay(60000 / portTICK_PERIOD_MS); // Aktuálnì 1 minuta
    }
}

// Kontroluje, zda je dostupná nová verze
void checkForUpdatesBackground() {
    Serial.println("Kontrola aktualizací na pozadí...");

    // Staení souboru s verzí z serveru
    HTTPClient http;
    http.begin(versionURL);
    http.setTimeout(10000);

    // Pøidání hlavièek pro simulaci bìného browseru
    http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
    http.addHeader("Accept", "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8");

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String serverVersion = http.getString();
        serverVersion.trim(); // odstraní bílé znaky

        Serial.print("Verze na serveru: ");
        Serial.println(serverVersion);
        Serial.print("Aktuální verze: ");
        //Serial.println(currentVersion);
        Serial.println(localVersion);

        //String localVersionTmp = currentVersion;
        String localVersionTmp = localVersion;
        localVersionTmp.replace(".", "");
        String serverVersionTmp = serverVersion;
        serverVersionTmp.replace(".", "");

        int localVersionInt = localVersionTmp.toInt();
        int serverVersionInt = serverVersionTmp.toInt();

        if (serverVersionInt > localVersionInt) {
            Serial.println("Nová verze nalezena!");
            otaUpdateAvailable = true;
            newVersion = serverVersion;
        }
        else {
            Serial.println("Firmware je aktuální.");
            otaUpdateAvailable = false;
        }
    }
    else {
        Serial.printf("Nelze získat verzi, HTTP kód: %d\n", httpCode);
    }
    http.end();
}

// Provede samotnou aktualizaci
void performUpdate() {
    Serial.println("Zahajuji aktualizaci na pozadí...");
    otaInProgress = true;

    // Vytvoøení indikace na displeji (pokud je potøeba)
    // displayUpdateStatus("Aktualizace...");

    // Pouijeme pøímou metodu s WiFiClient, která fungovala
    WiFiClient client;

    t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

    switch (ret) {
    case HTTP_UPDATE_FAILED:
        Serial.printf("Aktualizace selhala: %d - %s\n",
            httpUpdate.getLastError(),
            httpUpdate.getLastErrorString().c_str());
        // Resetujeme pøíznaky pro pøípadnı další pokus
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_NO_UPDATES:
        Serial.println("Není k dispozici ádná nová aktualizace.");
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_OK:
        Serial.println("Aktualizace probìhla úspìšnì, restartuju...");
        // Pøi úspìšné aktualizaci dojde k restartu ESP32
        break;
    }
}

// Performs the actual update (using HTTPS, following official example style)
void performUpdateHTTPS() {
    Serial.println("Zahajuji aktualizaci na pozadí (HTTPS)...");
    otaInProgress = true;
    otaUpdateAvailable = false; // Consume the flag

    Serial.print("Stahování z (HTTPS): ");
    Serial.println(firmwareURL);

    WiFiClientSecure secureClientForUpdate;

    // Configure the WiFiClientSecure object directly:
    // 1. Set the Root CA Certificate
    secureClientForUpdate.setInsecure();
    //secureClientForUpdate.setCACert(rootCACertificate_OTA);
    // 2. Set a timeout for the connection (e.g., 30 seconds for firmware download)
    // The official example uses 12 seconds (12000), adjust as needed for your server/file size.
    secureClientForUpdate.setTimeout(30000); // 30 seconds

    t_httpUpdate_return ret = httpUpdate.update(secureClientForUpdate, firmwareURL);

    switch (ret) {
    case HTTP_UPDATE_FAILED:
        Serial.printf("Aktualizace selhala: %d - %s\n",
            httpUpdate.getLastError(),
            httpUpdate.getLastErrorString().c_str());
        otaInProgress = false;
        break;

    case HTTP_UPDATE_NO_UPDATES:
        Serial.println("Není k dispozici ádná nová aktualizace (dle httpUpdate).");
        otaInProgress = false;
        break;

    case HTTP_UPDATE_OK:
        Serial.println("Aktualizace probìhla úspìšnì, restartuju...");
        // ESP32 will restart automatically.
        break;
    }
}

// Funkce pro inicializaci OTA úlohy - volejte v setup()
void setupOTA() {
    // Nastavíme aktuální verzi
    //currentVersion = "1.0.0.0";  // Pro testování, v reálu by se èetla z konstanty

    // Vytvoøíme úlohu na jádøe 0 (hlavní aplikace bìí typicky na jádøe 1)
    xTaskCreatePinnedToCore(
        otaUpdateTask,      // Funkce, která implementuje úlohu
        "OTATask",          // Název úlohy
        8192,               // Velikost zásobníku (v slovech)
        NULL,               // Parametr úlohy
        1,                  // Priorita úlohy (niší èíslo = niší priorita)
        &otaTaskHandle,     // Handle úlohy
        0                   // Jádro CPU, na kterém má úloha bìet (0 nebo 1)
    );
}

//************************************************************************
// Nová funkce pro odeslání bufferu ze souboru aptbuffer.txt
//************************************************************************
void send_file_buffer() {
    // 1. Zkontrolujeme, zda jsme online a pøipojeni
    if (!net_on || !client.connected()) {
        return; // Nejsme online nebo pøipojeni, nemá smysl pokraèovat
    }

    // 2. Zkusíme otevøít soubor bufferu pro ètení
    File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
    if (!bufferFile || bufferFile.size() == 0) {
        if (bufferFile) bufferFile.close();
        return; // Soubor neexistuje nebo je prázdnı
    }

    // 3. Pøeèteme první øádek (zprávu) ze souboru
    String lineToSend = "";
    if (bufferFile.available()) {
        lineToSend = bufferFile.readStringUntil('\n');
    }

    if (lineToSend.length() == 0) {
        bufferFile.close(); // Nic jsme nepøeèetli (moná chyba nebo jen prázdnı øádek na zaèátku)
        // Mùeme zkusit soubor promazat, pokud obsahuje jen bílé znaky? Prozatím ne.
        return;
    }

    // Pøidáme zpìt znak nového øádku, pokud ho readStringUntil odstranil a server ho vyaduje
    lineToSend += "\n";

    Serial.print("Attempting to send from file buffer: ");
    Serial.print(lineToSend);

    // 4. Pokusíme se odeslat první øádek
    bool send_success = false;
    if (client.print(lineToSend)) {
        client.flush(); // Poèkáme na odeslání
        Serial.println(" - Sent, waiting for confirmation...");

        // 5. Èekáme na potvrzení od serveru (podobnì jako v pùvodním send_buffer)
        //    Pouijeme jednoduchı mechanismus ètení s timeoutem
        unsigned long confirm_timeout = millis() + 3000; // Timeout 3 sekundy na odpovìï
        String response = "";
        while (millis() < confirm_timeout) {
            if (client.available()) {
                response = client.readStringUntil('\n');
                response.trim(); // Odstraníme bílé znaky
                Serial.print(" - Received response: ");
                Serial.println(response);
                // Zde pøedpokládáme, e server pošle nìco specifického pro potvrzení
                // Pùvodní kód èekal na odpovìï zaèínající na 0x7E (~)
                // Upravte podmínku podle vaší serverové logiky
                if (response.length() > 0 && response.startsWith("~")) { // Pøíklad: odpovìï zaèíná '~'
                    send_success = true;
                    last_ping = SEC_TIMER; // Aktualizujeme last_ping
                    break;
                }
            }
            delay(10); // Krátká pauza, aby nebìela smyèka naplno
        }
        if (!send_success) {
            Serial.println(" - Confirmation failed or timed out.");
        }

    }
    else {
        Serial.println(" - Client print failed.");
        setNetOn(0, 20); // Pravdìpodobnì problém se spojením
        bufferFile.close();
        return;
    }

    // 6. Pokud odeslání a potvrzení probìhlo úspìšnì, pøepíšeme soubor bez odeslaného øádku
    if (send_success) {
        Serial.println(" - Send successful. Removing line from buffer file.");

        // Otevøeme doèasnı soubor pro zápis
        File tempFile = LittleFS.open("/apt1220/aptbuffer.tmp", FILE_WRITE);
        if (!tempFile) {
            Serial.println("Error: Failed to open temporary buffer file for writing!");
            bufferFile.close();
            return;
        }

        // Zkopírujeme zbytek pùvodního souboru (od druhého øádku dál) do doèasného
        char copyBuf[128]; // Buffer pro kopírování
        while (bufferFile.available()) {
            int bytesRead = bufferFile.readBytes(copyBuf, sizeof(copyBuf));
            if (bytesRead > 0) {
                tempFile.write((uint8_t*)copyBuf, bytesRead);
            }
        }

        // Zavøeme oba soubory
        bufferFile.close();
        tempFile.close();

        // Smaeme pùvodní soubor a pøejmenujeme doèasnı
        if (LittleFS.remove(bufferFilePath)) {
            if (LittleFS.rename("/apt1220/aptbuffer.tmp", bufferFilePath)) {
                Serial.println(" - Buffer file updated successfully.");
            }
            else {
                Serial.println("Error: Failed to rename temporary buffer file!");
                // Pokusíme se smazat i doèasnı soubor, pokud pøejmenování selhalo
                LittleFS.remove("/apt1220/aptbuffer.tmp");
            }
        }
        else {
            Serial.println("Error: Failed to remove original buffer file!");
            // Pokusíme se smazat i doèasnı soubor
            LittleFS.remove("/apt1220/aptbuffer.tmp");
        }

    }
    else {
        // Pokud odeslání selhalo, soubor nemìníme, zavøeme ho a zkusíme to znovu pozdìji
        Serial.println(" - Send failed. Buffer file remains unchanged.");
        bufferFile.close();
    }
}

void initializeSyncPrimitives() {
    // Create mutex for safe I2C access (LCD and RTC)
    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        Serial.println("Chyba pri vytvareni I2C mutexu!");
    }

    // Create mutex for safe TCP client access
    tcpMutex = xSemaphoreCreateMutex();
    if (tcpMutex == NULL) {
        Serial.println("Chyba pri vytvareni TCP mutexu!");
    }

    // Create mutex for the demo screen task
    demoMutex = xSemaphoreCreateMutex();
    if (demoMutex == NULL) {
        Serial.println("Chyba pri vytvareni Demo mutexu!");
    }

    // Create the queue for serial data
    serialDataQueue = xQueueCreate(10, sizeof(SerialData_t));
    if (serialDataQueue == NULL) {
        Serial.println("Chyba pri vytvareni fronty!");
    }
}

//************************************************************************
// OPRAVENÁ funkce pro odeslání CELÉHO bufferu ze souboru aptbuffer.txt
// Verze 4: Opraveno nebezpeèné volání isspace().
//************************************************************************
void send_entire_file_buffer() {
    // 1. Kontrola sítì - pokud nejsme online, nemá smysl pokraèovat.
    if (!net_on) return;

    // Rychlá kontrola pøipojení klienta pomocí mutexu
    bool is_connected = false;
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        is_connected = client.connected();
        xSemaphoreGive(tcpMutex);
    }
    if (!is_connected) return;

    // 2. Kontrola existence a velikosti souboru
    File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
    if (!bufferFile || bufferFile.size() == 0) {
        if (bufferFile) bufferFile.close();
        return; // Soubor neexistuje nebo je prázdnı, není co dìlat.
    }

    Serial.println(">>> Zahajuji odesilani offline bufferu ze souboru... <<<");

    // Pøipravíme si doèasnı soubor, kam budeme ukládat øádky, které se nepodaøilo odeslat.
    String tempFileName = "/apt1220/aptbuffer.tmp";
    File tempFile = LittleFS.open(tempFileName, FILE_WRITE);
    if (!tempFile) {
        Serial.println("CHYBA: Nelze otevrit docasny soubor pro zapis!");
        bufferFile.close();
        return;
    }

    bool error_occurred = false; // Flag, kterı nám øekne, jestli se má pùvodní soubor smazat, nebo nahradit doèasnım.

    // Indikace na LCD displeji
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        _fast_clear_disp_unsafe();
        lcd2.setCursor(0, 0); lcd2.print(F("*------------------*"));
        lcd2.setCursor(0, 1); lcd2.print(F("|  ODESILAM BUFFER |"));
        lcd2.setCursor(0, 2); lcd2.print(F("|   ZE SOUBORU...  |"));
        lcd2.setCursor(0, 3); lcd2.print(F("*------------------*"));
        xSemaphoreGive(i2cMutex);
    }

    // 3. Smyèka pro ètení a odeslání všech øádkù ze souboru
    while (bufferFile.available()) {
        String lineToSend = bufferFile.readStringUntil('\n');
        if (lineToSend.length() == 0) continue;
        lineToSend += "\n";

        Serial.print("Odesilam: "); Serial.print(lineToSend);

        bool send_success = tcp_print_safe(lineToSend.c_str());
        bool confirmed = false;

        if (send_success) {
            uint32_t deadline = millis() + 3000;
            char responseBuffer[64];

            while (millis() < deadline) {
                size_t bytesRead = 0;
                if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (client.available()) {
                        bytesRead = client.readBytesUntil('\n', responseBuffer, sizeof(responseBuffer) - 1);
                    }
                    xSemaphoreGive(tcpMutex);
                }

                if (bytesRead > 0) {
                    responseBuffer[bytesRead] = '\0';
                    size_t n = strlen(responseBuffer);
                    // Pøetypujeme responseBuffer[n-1] na (unsigned char), abychom pøedešli pádu.
                    while (n > 0 && isspace((unsigned char)responseBuffer[n - 1])) {
                        responseBuffer[--n] = '\0';
                    }

                    Serial.print("  Odpoved: '"); Serial.print(responseBuffer); Serial.println("'");

                    if (strcmp(responseBuffer, "OK") == 0) {
                        confirmed = true;
                        last_ping = SEC_TIMER;
                        break;
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        if (confirmed) {
            Serial.println(" -> USPECH");
        }
        else {
            Serial.println(" -> SELHANI");
            error_occurred = true;
            tempFile.print(lineToSend);

            if (!send_success) {
                setNetOn(0, 22);
                char copyBuf[128];
                while (size_t bytesRead = bufferFile.readBytes(copyBuf, sizeof(copyBuf))) {
                    tempFile.write((uint8_t*)copyBuf, bytesRead);
                }
                break;
            }
        }
    }

    bufferFile.close();
    tempFile.close();

    LittleFS.remove(bufferFilePath);

    if (LittleFS.rename(tempFileName, bufferFilePath)) {
        if (error_occurred) {
            Serial.println(">>> Odesilani bufferu dokonceno s chybami. Neodeslana data zachovana. <<<");
        }
        else {
            Serial.println(">>> Offline buffer uspesne odeslan a smazan. <<<");
        }
    }
    else {
        Serial.println("CHYBA: Kriticka chyba pri prejmenovani docasneho souboru!");
    }

    timeout1 = 0;
}