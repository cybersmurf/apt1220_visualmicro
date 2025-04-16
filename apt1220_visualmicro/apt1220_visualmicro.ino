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
#include <stdarg.h>  // Knihovna pro variabiln� argumenty
#include <stdio.h>   // Knihovna pro vsnprintf
#include <LCDI2C_Multilingual.h>

#include <Wire.h>
#include <I2C_RTC.h>
#include <RTClib.h>
//#include <uRTClib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

///vypnut� pou�it� EEPROM a EspConfigLib
//#include "uEEPROMLib.h"
//#include "uEspConfigLib.h"

#include <Preferences.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>

#define SPIFFS LittleFS
#define FORMAT_LITTLEFS_IF_FAILED true

#define DEBUG_MODE

// Caution: It need to be a global variable or a global pointer.
// if FS is a 'setup' variable it will lead to crashes
///vypnut� pou�it� EEPROM a EspConfigLib
//uEspConfigLibFSInterface* configFsC;
//uEspConfigLib* config;

// uEEPROMLib eeprom;
///vypnut� pou�it� EEPROM a EspConfigLib
//uEEPROMLib eeprom(0x50);

static DS1307 rtc2;
//uRTCLib rtc2;

LCDI2C_Latin_Symbols lcd2(0x27, 20, 4);    // I2C address = 0x27; LCD = Surenoo SLC1602A (European)
WiFiClient client;
//WebServer server(80);
AsyncWebServer server(80);


// Inicializace s�riov�ch port�
HardwareSerial SerialC(1); // Pro SerialC
HardwareSerial SerialD(2); // Pro SerialD

#define TCPCONFIG 1
#define VERZE_PRACANT  "4.0.0"
#define ALOC_MEM 64000
#define SERB_USEPORTD
#define timeout_reset 86400;  // jak casto resetovat

// commands of terminal
#define C_ERROR_DATA     $00;  // nekorektni data
#define C_ECHO           $01;  // Po�li odezvu        PC <-> TERMINAL
#define C_SET_TIME       $02;  // Nastav �as          PC  -> TERMINAL
#define C_GET_TIME       $03;  // Po�li �as           PC <-> TERMINAL
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

static int __attribute__((section(".noinit")))  rfid_reader_c;      // 0 - normalni �te�ka
static int __attribute__((section(".noinit")))  id12_c;             // 0 - normalni �te�ka
                                                                    // 1 - rfid modul
static int __attribute__((section(".noinit")))  rfid_reader_d;      // 0 - normalni �te�ka
static int __attribute__((section(".noinit")))  id12_d;             // 0 - normalni �te�ka
                                                                    // 1 - rfid modul

static long __attribute__((section(".noinit"))) timer1;		        // DISPLAY NAME timeout
static long __attribute__((section(".noinit"))) timer2;		        // CONFIRM TIMEOUT
static long __attribute__((section(".noinit"))) timer3;
static char __attribute__((section(".noinit"))) op_c[16];
static char __attribute__((section(".noinit"))) op_d[16];

static String activeMAC;

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

static unsigned long physaddr;

char buffer[251] = { 0 };

char off_buffer[251] = { 0 };
//char __attribute__((section(".noinit"))) off_buffer[251];
#define off_buffer_size 250    // OFFLINE BUFFER SIZE - maximu
static unsigned long SEC_TIMER;

static bool eth_connected = false;
static int  runCounter = 0;

// Nastaven� WiFi a TCP
static char ssid[32] = "AGERIT_AC 2GHz";
static char password[32] = "AGERITagerit512";
//static char* ssid = "blackies";
//static char* password = "Blackies105111";
const char* serverIP = "192.168.225.221";
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

// Lok�ln� verze firmware
const String localVersion = "1.0.0.2";
// URL k souboru, kde je uvedena aktu�ln� verze firmware na serveru
const char* versionURL = "http://192.168.222.114:82/ota_update/version.txt";
// URL k nov�mu firmware (bin�rn� soubor)
const char* firmwareURL = "http://192.168.222.114:82/ota_update/firmware.bin";

// Glob�ln� prom�nn� pro OTA aktualizaci
TaskHandle_t otaTaskHandle = NULL;
bool otaInProgress = false;
bool otaUpdateAvailable = false;
String currentVersion = "1.0.0.2";
String newVersion = "";

Preferences preferences;

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

///vypnut� pou�it� EEPROM a EspConfigLib
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
    SerialC.begin(9600, SERIAL_8N1, 36, 14);
    SerialD.begin(9600, SERIAL_8N1, 32, 33);
}

void initializeFileSystem() {
    // Pokus�me se p�ipojit LittleFS, form�tovat pokud sel�e
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        Serial.println("LittleFS Mount Failed initially.");

        // --- P�id�no: Explicitn� pokus o form�tov�n� ---
        Serial.println("Attempting to format LittleFS...");
        if (LittleFS.format()) {
            Serial.println("LittleFS formatted successfully.");
            // Zkus�me p�ipojit znovu po form�tov�n�
            if (!LittleFS.begin()) {
                Serial.println("LittleFS Mount Failed even after format!");
                return; // Ukon��me, pokud ani po form�tu nelze p�ipojit
            }
            Serial.println("LittleFS Mounted successfully after format.");
        }
        else {
            Serial.println("LittleFS format failed.");
            return; // Ukon��me, pokud form�tov�n� sel�e
        }
        // --- Konec p�idan�ho k�du ---

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

        File bufferFile = LittleFS.open(bufferFilePath, FILE_READ); // Otev�eme soubor pro �ten�
        if (bufferFile) {
            size_t fileSize = bufferFile.size(); // Z�sk�me velikost souboru
            Serial.print(" -> Size: ");
            Serial.print(fileSize);
            Serial.println(" bytes");
            bufferFile.close(); // Zav�eme soubor
        }
        else {
            Serial.println(" -> Error: Failed to open buffer file to get size.");
        }

    }
}

void initializeNetwork() {
    Serial.print("Registering event handler for ETH events...");
    WiFi.onEvent(WiFiEvent);
    bool wifiConnected = false;
    bool ethConnected = false;

    if (useWifi) {
        connectToWiFi();
        wifiConnected = (WiFi.status() == WL_CONNECTED);
    }
    if (useETH) {
        ETH.begin();
        ethConnected = eth_connected;
    }

    if (!wifiConnected && !ethConnected) {
        startAPMode();
    }
    else {
        connectToServer();
    }
}

void initializeTasks() {
    xTaskCreatePinnedToCore(tSEC_TIMERcode, "tSEC_TIMER", 4096, NULL, 0, &tSEC_TIMER, 1);
    xTaskCreatePinnedToCore(tLAST_PINGcode, "tLAST_PING", 4096, NULL, 0, &tLAST_PING, 1);
}

void initializeDisplay() {
    lcd2.init();
    lcd2.backlight();
    lcd2.createChar(0, customChar);
    lcd2.clear();
    delay(200);
    lcd2.printf("       eMISTR       ");
    lcd2.setCursor(0, 2);
	lcd2.printf("  Verze: %s", VERZE_PRACANT);
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


    ///vypnut� pou�it� EEPROM a EspConfigLib
    //setup_inifile();

    ///vypnut� pou�it� EEPROM a EspConfigLib
    //eeprom.eeprom_read(240, &loaded_default);

    delay(100);

	loadConfiguration();

	initializeNetwork();

    // Kontrola aktualizac� ji� p�i spu�t�n�
    //checkForUpdates();

	initializeTasks();

    // Inicializace OTA
    setupOTA();

	initializeDisplay();


    // Create mutex for safe LCD access
    demoMutex = xSemaphoreCreateMutex();

/*
    // Create the task that will handle the demo function
    xTaskCreate(
        tDEMOcode,          // Function to implement the task
        "DemoTask",        // Name of the task
        2048,              // Stack size in words
        NULL,              // Task input parameter
        1,                 // Priority of the task
        &tDEMO    // Task handle
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
    timer_reset = SEC_TIMER + 86400;  // Denn� reset 

    //rtc2.startClock();

#ifdef DEBUG_MODE
    // Start the web server in debug mode
    server.on("/", HTTP_GET, handleRoot);
    server.on("/scan", HTTP_GET, handleScanNetworks);
    server.on("/save", HTTP_POST, handleSaveCredentials);
    server.begin();
    Serial.println("Web server started in debug mode.");
#endif
}

void fast_clear_disp() {
    lcd2.clear();
    lcd2.home();
};

void blank_line(int line) {
    lcd2.setCursor(0, line);                       // Um�st� kurzor na za��tek ��dku
    lcd2.print("                    ");           // Vyp�e pr�zdn� �et�zec o 20 mezer�ch
    lcd2.setCursor(0, line);                       // Um�st� kurzor zp�t na za��tek ��dku
}

boolean load_config() {
    //na�ten� konfigurace z Preferences
    preferences.begin("my-app", true);
    sprintf(ip_adr, preferences.getString("ip_adr", "192.168.222.202").c_str());
    sprintf(ip_mask, preferences.getString("ip_mask", "255.255.255.0").c_str());
    sprintf(ip_server, preferences.getString("ip_server", "192.168.225.221").c_str());
    sprintf(ip_gate, preferences.getString("ip_gate", "192.168.222.222").c_str());
    timer1 = preferences.getInt("timer1", 5);
    timer2 = preferences.getInt("timer2", 3);
    key_maker = preferences.getInt("key_maker", 0);
    rfid_reader_c = preferences.getInt("rfid_reader_c", 1);
    rfid_reader_d = preferences.getInt("rfid_reader_d", 0);
    id12_c = preferences.getInt("id12_c", 1);
    id12_d = preferences.getInt("id12_d", 0);
    sprintf(op_c, preferences.getString("op_c", "A3997").c_str());
    sprintf(op_d, preferences.getString("op_d", "").c_str());

    useWifi = preferences.getBool("useWifi", false);
    useETH = preferences.getBool("useETH", true);

    sprintf(ssid, preferences.getString("SSID_NAME", "AGERIT_AC 2GHz").c_str());
    sprintf(password, preferences.getString("SSID_PASS", "AGERITagerit512").c_str());

    ping_timeout = 4;
    preferences.end();
    return true;
}

boolean save_config() {
    //ulo�en� konfigurace do Preferences
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

    preferences.putString("SSID_NAME", ssid);
    preferences.putString("SSID_PASS", password);

    preferences.end();
    return true;
}

void set_default() {
    sprintf(ip_adr, "192.168.222.202");
    sprintf(ip_mask, "255.255.255.0");
    sprintf(ip_server, "192.168.225.221");
    sprintf(ip_gate, "192.168.222.222");
    timer1 = 5;
    timer2 = 3;
    key_maker = 0;
    rfid_reader_c = 1;
    rfid_reader_d = 0;
    id12_c = 1;
    id12_d = 0;
    sprintf(op_c, "A3997");
    sprintf(op_d, "");
    useDHCP = true;
    ping_timeout = 4;

    loaded_default = 1;
    ///vypnut� pou�it� EEPROM a EspConfigLib
    //eeprom.eeprom_write(240, &loaded_default);
}

void init_lcd() {
    lcd2.init();
    lcd2.clear();                             // Initialize the LCD
    lcd2.backlight();                         // Turn on the LCD backlight
    lcd2.noAutoscroll();
}

void menu_dump() {
    char long_str[20];  // Vyrovn�vac� pam� pro p�eveden� ��sla

    fast_clear_disp();  // Vy�i�t�n� displeje

    // Zobrazen� hodnoty fifo_start
    lcd2.setCursor(0, 0);
    lcd2.printf("START:");
    ltoa(fifo_start, long_str, 10);  // P�evede `fifo_start` na �et�zec
    lcd2.printf(long_str);

    // Zobrazen� hodnoty fifo_end
    lcd2.setCursor(0, 1);
    lcd2.printf("END:");
    ltoa(fifo_end, long_str, 10);
    lcd2.printf(long_str);

    // Zobrazen� hodnoty fifo_first
    lcd2.setCursor(0, 2);
    lcd2.printf("FIRST:");
    ltoa(fifo_first, long_str, 10);
    lcd2.printf(long_str);

    // Zobrazen� hodnoty fifo_last
    lcd2.setCursor(0, 3);
    lcd2.printf("LAST:");
    ltoa(fifo_last, long_str, 10);
    lcd2.printf(long_str);
}

void menuconfig() {
    long timeout_up;
    fast_clear_disp();
    lcd2.setCursor(0, 0);
    lcd2.printf("IP:");
    lcd2.printf(ip_adr);
    lcd2.setCursor(0, 1);
    lcd2.printf("MASK:");
    lcd2.printf(ip_mask);
    lcd2.setCursor(0, 2);
    lcd2.printf("SRV:");
    lcd2.printf(ip_server);
    lcd2.setCursor(0, 3);
    lcd2.printf("VERZE: ");
    lcd2.printf(" %s  ", VERZE_PRACANT);
}

void resetConfig() {
    // Zav�ete existuj�c� TCP p�ipojen�
    if (client.connected()) {
        client.stop();
        Serial.println("TCP client disconnected");
    }

    // Restart WiFi p�ipojen�
    WiFi.disconnect(true); // Odpoj� a zapomene p�ipojen�
    delay(1000);           // Po�kejte na odpojen�

    // Nastaven� statick� IP (pokud je po�adov�na statick� IP)
    if (!WiFi.config(IPAddress(ip_adr), IPAddress(ip_gate), IPAddress(ip_mask))) {
        Serial.println("STA Failed to configure");
    }

    Serial.println(useWifi ? "WIFI connect" : "ETH connect");

    // P�ipojte se znovu k WiFi s�ti
    if (useWifi) {
        connectToWiFi();
    }

    // Starth Ethernet (this does NOT start WiFi at the same time)
    if (useETH) {
        Serial.print("Starting ETH interface...");
        ETH.begin();
    }

    // Inicializace FIFO prom�nn�ch
      //fifo_start = malloc(ALOC_MEM);
      //fifo_end =fifo_start+ALOC_MEM;

    fifo_first = fifo_start;
    fifo_last = fifo_start;

    // Ov��en� a nastaven� �asova��
    if (timer1 > 60 || timer1 < 1) timer1 = 10;
    if (timer2 > 60 || timer2 < 1) timer2 = 5;

    // Otev�ete nov� TCP p�ipojen�
    connectToServer();
    /*
      if (client.connect(ip_server, serverPort)) {
          Serial.println("Reconnected to TCP server");
      } else {
          Serial.println("Failed to reconnect to TCP server");
      }
    */
}

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer for new data
    char buffer2[100];       // Buffer for serial input

    fast_clear_disp();
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
            int len = SerialC.readBytesUntil('\n', buffer2, sizeof(buffer2) - 1);
            buffer2[len] = '\0';  // Null-terminate the string

            if (strcmp(buffer2, "STORNO") == 0) {
                break;
            }
            else if (strcmp(buffer2, "OK") == 0) {
                strncpy(data_to_fill, new_data, sizeof(new_data));
                break;
            }
            else if (strcmp(buffer2, "DEFAULT") == 0) {
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                strncat(new_data, buffer2, sizeof(new_data) - strlen(new_data) - 1);
                lcd2.printf("%s", buffer2);
            }
        }

        if (SerialD.available() > 0) {
            int len = SerialD.readBytesUntil('\n', buffer2, sizeof(buffer2) - 1);
            buffer2[len] = '\0';  // Null-terminate the string

            if (strcmp(buffer2, "STORNO") == 0) {
                break;
            }
            else if (strcmp(buffer2, "OK") == 0) {
                strncpy(data_to_fill, new_data, sizeof(new_data));
                break;
            }
            else if (strcmp(buffer2, "DEFAULT") == 0) {
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                strncat(new_data, buffer2, sizeof(new_data) - strlen(new_data) - 1);
                lcd2.printf("%s", buffer2);
            }
        }

    }
}

/*

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer pro nov� data
    String buffer2;          // Pomocn� buffer pro s�riov� vstup

    fast_clear_disp();       // Vymaz�n� LCD displeje

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
                //buffer2 = Serial.readStringUntil('\n');  // �ten� vstupu do bufferu do znaku nov�ho ��dku
            buffer2 = SerialC.readStringUntil('\n');  // �ten� vstupu do bufferu do znaku nov�ho ��dku

            buffer2.trim();  // Odstran�n� b�l�ch znak�

            if (buffer2.equals("STORNO")) {
                break;  // P�eru�en� p�i p��kazu "STORNO"
            }
            else if (buffer2.equals("OK")) {
                strncpy(data_to_fill, new_data, sizeof(new_data));  // Ulo�en� nov�ch dat
                break;
            }
            else if (buffer2.equals("DEFAULT")) {
                // Odstran�n� posledn�ho znaku
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                // P�id�n� vstupu do `new_data` a zobrazen� na LCD
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
    /*
        char buffer2_copy[100];
        strncpy(buffer2_copy, buffer2, sizeof(buffer2_copy) - 1);
        buffer2_copy[sizeof(buffer2_copy) - 1] = '\0';  // Zaji�t�n� null-terminace

        // Nyn� pracujte s `buffer2_copy` m�sto `buffer2`
        if (buffer2_copy[strlen(buffer2_copy) - 1] == '\n') {
            buffer2_copy[strlen(buffer2_copy) - 1] = '\0';
        }
    */
    // Odstran�n� koncov�ch znak� `\n`
    if (buffer2[strlen(buffer2) - 1] == '\n') {
        buffer2[strlen(buffer2) - 1] = '\0';
    }

    logToSerial(buffer2, 0);

    String buffer2String = buffer2;
    buffer2String.trim();

    if ((millis() / 1000) >= timeout1) { fast_clear_disp(); }

    // Zpracov�n� r�zn�ch p��kaz�
    //if (strcmp(buffer2String, "SET-IP") == 0) {
    if (buffer2String.equals("SET-IP")) {
        reader_input("Nastaveni IP", ip_adr);
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
        //} else if (strcmp(buffer2, "SET-MASK") == 0) {
    }
    else if (buffer2String.equals("SET-MASK")) {
        reader_input("Nastaveni Masky", ip_mask);
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
            blank_line(0);
            lcd2.printf("FUNKCE NEPODPOROVANA");
            timeout1 = SEC_TIMER + timer2;
        }
        //} else if (strcmp(buffer2, "SET-CLOCK") == 0) {
    }
    else if (buffer2String.equals("SET-CLOCK")) {
        snprintf(buffer, sizeof(buffer), "\x03~");
        client.print(buffer);
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
    }
    else if (buffer2String.equals("SET-WIFI")) {
        useWifi = true;
        useETH = false;
    }
    else if (buffer2String.equals("SET-ETH")) {
        useWifi = false;
        useETH = true;
    }
    else if (buffer2String.equals("SET-SSID")) {
        //SSID
        reader_input("Nastaven� SSID", ssid);
    }
    else if (buffer2String.equals("SET-PASSW")) {
        //SSID PASSWORD 
        reader_input("Nastaven� hesla", password);                
    }
    else if (buffer2String.equals("SHOW-NET")) {
        //SSID PASSWORD 
        reader_input("Nastaven� hesla", password);
    }
    else if (buffer2String.equals("SHOW-WIFI")) {
        //SSID PASSWORD 
        reader_input("Nastaven� hesla", password);
    }
    else if (buffer2String.equals("RESET-BUFFER")) {
        reset_buffer_file();
    }
    else {
        timeout1 = SEC_TIMER + timer1;
        if (saved) {
            fast_clear_disp();
            saved = 0;
        }

        // Zpracov�n� na z�klad� prvn�ho znaku
        switch (buffer2[0]) {
        case 'A':
            strcpy(tmp, "\x06");
            break;
        case 'B':
            strcpy(tmp, "\x05");
            break;
        case 'D':
            if (key_maker == 1) {
                snprintf(tmp2, sizeof(tmp2), "\x06AOPEN\n");
                if (net_on) {
                    client.print(tmp2);
                    strcpy(tmp, "\x04");
                }
                else {
                    strcpy(tmp, "\x06AOPEN\n");
                    lcd2.printf("Zamek odjisten");

                    timeout1 = SEC_TIMER + timer2;
                }
            }
            else {
                strcpy(tmp, "\x04");
            }
            break;
        case 'E':
            strcpy(tmp, "\x08");
            break;
        case 'F':
            strcpy(tmp, "\x07");
            break;
        case 'G':
            strcpy(tmp, "\x10");
            break;
        case 'H':
            strcpy(tmp, "\x11");
            break;
        case 'I':
            strcpy(tmp, "\x12");
            break;
        case 'J':
            strcpy(tmp, "\x13");
            break;
        case 'K':
            strcpy(tmp, "\x19");
            break;
        case 'L':
            strcpy(tmp, "\x20");
            break;
        default:
            strcpy(tmp, "\x17");
            break;
        }

        //strcat(tmp, buffer2);
		strcat(tmp, buffer2String.c_str());

        // Echo na LCD p�i offline stavu
        if (!net_on && ((long)(fifo_end - fifo_last) >= off_buffer_size)) {
            blank_line(2);
            //lcd2.printf("%s", buffer2);
			lcd2.printf("%s", buffer2String.c_str());
        }

        // P�id�n� �asu k p��kaz�m typu D, E, J atd.
        if (strchr("DEJ0123456789", buffer2[0])) {
            strcat(tmp, "~");
            //get_time(SEC_TIMER, buffer);
            get_time(buffer);
            strcat(tmp, buffer);
        }

        strcat(tmp, "\n");
        if (net_on) {
            client.print(tmp);
        }
        else {
            //strcat(off_buffer, tmp);
            // --- Kontrola velikosti P�ED strcat (z minul� �pravy) ---
            if (strlen(off_buffer) + strlen(tmp) < sizeof(off_buffer)) {
                strcat(off_buffer, tmp); // P�idat do off_buffer, jen pokud se vejde
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
                appendFile(LittleFS, bufferFilePath, off_buffer);

                //fifo_last += strlen(off_buffer) + 1;

                lcd2.setCursor(0, 0);
                lcd2.printf("*------------------*");
                lcd2.setCursor(0, 1);
                lcd2.printf("|      ULOZENO     |");
                lcd2.setCursor(0, 2);
                lcd2.printf("|     DO PAMETI    |");
                lcd2.setCursor(0, 3);
                lcd2.printf("*------------------*");

                saved = 1;
                timeout1 = SEC_TIMER + timer2;
                //memset(off_buffer, 0x00, off_buffer_size);
                memset(off_buffer, 0x00, sizeof(off_buffer));
            }
        }
    }
}

void tDEMOscreen() {
    char buffer2[100] = { 0 };

    // Zobrazen� informac� na LCD
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

    //DateTime now = rtc2.now();
	//String nowStr = now.timestamp(DateTime::TIMESTAMP_FULL);

    //lcd2.print(nowStr);
    
    print_time(rtc2.getEpoch() , buffer2);

    //logToSerial(rtc2.getDateTimeString() , 1);
	//logToSerial(nowStr.c_str(), 1);
    //logToSerial(buffer2,1);

    //print_time(rtc2.now().unixtime(), buffer2);

    lcd2.printf(buffer2);
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

void loop() {
    //hlavn� obrazovka
    // Buffer pro zobrazen� �asu na LCD
    if (net_on) {
        TCP();
        if ((long)(fifo_last - fifo_start) > 0) {
            //send_buffer();
            send_file_buffer();
        }
    }

    //loop_bbbb();
	//tDEMOscreen();
    if ((millis() / 1000) >= timeout1) { tDEMOscreen(); }

    // Periodick� kontrola, nap��klad ka�dou minutu (pro demonstraci)
    //static unsigned long lastCheck = 0;
    //if (millis() - lastCheck > 60000) { // 60 sekund
    //    checkForUpdates();
    //    lastCheck = millis();
    //}

    // �ten� z r�zn�ch s�riov�ch port� (SerialC, SerialD)
    if (SerialC.available()) {
        char buffer2[100] = { 0 };  // Deklarace lok�ln� prom�nn� pro �ten� ze SerialC
        SerialC.readBytesUntil('\n', buffer2, sizeof(buffer2));
        logToSerial(buffer2, 1);
        if (strlen(buffer2) > 0) {
            serial(buffer2, 1);
        }
    }

    if (SerialD.available()) {
        char buffer2[100] = { 0 };  // Deklarace lok�ln� prom�nn� pro �ten� ze SerialD
        SerialD.readBytesUntil('\n', buffer2, sizeof(buffer2));
        logToSerial(buffer2, 1);
        if (strlen(buffer2) > 0) {
            serial(buffer2, 2);
        }
    }

    // Automatick� reset ka�d�ch 24 hodin
    /*
    if ((long)(SEC_TIMER - timer_reset) >= 0) {
        Serial.println("REINICIALIZACE = " + String(SEC_TIMER) + " / " + String(timer_reset));
        lcd2.setCursor(0, 0);
        lcd2.printf(" REINICIALIZACE ... ");

        delay(2000);
        ESP.restart();
    }
    */

    //delay(100);
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
            if ((millis()/1000)>=timeout1) { tDEMOscreen(); }
            // Release the mutex
            xSemaphoreGive(demoMutex);
        }

        // Wait for the next cycle (100ms)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void tDEMOrun() {
    //hlavn� obrazovka
    // Buffer pro zobrazen� �asu na LCD
    char buffer2[100] = { 0 };

    // Zobrazen� informac� na LCD
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

    print_time(rtc2.getEpoch(), buffer2);
    //print_time(rtc2.now().unixtime(), buffer2);

    lcd2.printf(buffer2);
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
    if ((vSerialDebug) && (pLogLevel > vLogLevel)) { Serial.println(vMessage); }
}

void logToSerial(const String vMessage, int pLogLevel) {
    if ((vSerialDebug) && (pLogLevel > vLogLevel)) { Serial.println(vMessage); }
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

void get_time(unsigned long thetime, char* buff) {
    struct tm thetm;

    // Pou��v�me gmtime_r pro bezpe�n� konvertov�n� epoch�ln�ho �asu na strukturovan� �as
    gmtime_r((time_t*)&thetime, &thetm);

    // Vytvo�en� form�tovan�ho �et�zce s datem a �asem
    sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d",
        //1900 + thetm.tm_year,    // Rok za��n� od 1900
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok za��n� od 1900
        thetm.tm_mon + 1,        // M�s�ce jsou indexov�ny od 0
        thetm.tm_mday,           // Den v m�s�ci
        thetm.tm_hour,           // Hodiny
        thetm.tm_min,            // Minuty
        thetm.tm_sec             // Sekundy
    );
}

void get_time(char* buff) {
    //DateTime now;

    // Z�sk�n� aktu�ln�ho �asu
    //now = rtc2.getDateTime();

    // Vytvo�en� form�tovan�ho �et�zce s datem a �asem
    sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d",
        rtc2.getYear(),          // Rok (pln�, nap�. 2024)
        rtc2.getMonth(),         // M�s�c (1-12)
        rtc2.getDay(),           // Den (1-31)
        rtc2.getHours(),          // Hodiny (0-23)
        rtc2.getMinutes(),        // Minuty (0-59)
        rtc2.getSeconds()         // Sekundy (0-59)
    );
}

void print_time(unsigned long thetime, char* buff)
{
    struct tm	thetm;

    gmtime_r((time_t*)&thetime, &thetm);
    /*
        sprintf(buff,"%02d.%02d.%04d  %02d:%02d:%02d",
                thetm.tm_mday , thetm.tm_mon, 1900+thetm.tm_year,
                thetm.tm_hour, thetm.tm_min, thetm.tm_sec);
    */
    sprintf(buff, "%02d.%02d.%04d  %02d:%02d:%02d",
        //thetm.tm_mday, thetm.tm_mon + 1, 1900 + thetm.tm_year,
        thetm.tm_mday, thetm.tm_mon + 1, RTC_YEAR_OFFSET + thetm.tm_year,
        thetm.tm_hour, thetm.tm_min, thetm.tm_sec);

};

void setNetOn(int vNetOn, int vCallFromId) {
    //if (vSerialDebug) {Serial.println("net_on = "+String(vNetOn) + " from: "+String(vCallFromId));}
    if (vNetOn == 0) { logToSerial("net_on = " + String(vNetOn) + " from: " + String(vCallFromId), 1); }
    net_on = vNetOn;
}

void tLAST_PINGcode(void* parameter) {
    for (;;) {
        // Ode�leme p��kaz ping ve form�tu "\01\n" (bin�rn� 0x01 n�sledovan� '\n')
        if (client.connected()) {
            if (client.write("\x01\n", 2) != 2) {
                setNetOn(0, 15);
            }
            else
            {
                setNetOn(1, 15);
            }
            client.flush();  // Vypr�zdn� v�stupn� buffer
        }
        else {
            setNetOn(0, 16); //net_on = 0;  // Pokud nen� p�ipojen, nastav� `net_on` na 0
            connectToServer();
        }
        vTaskDelay(1900 / portTICK_PERIOD_MS);
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
    char buffer[80] = { 0 };     // Buffer pro p�ijat� data
    char buffer2[80] = { 0 };    // Dal�� pomocn� buffer
    int counter1;
    struct tm thetm;

    // Kontrola stavu p�ipojen�
    if (net_on == 1 && ((long)(SEC_TIMER - last_ping) > ping_timeout)) {
        client.stop();  // Zav��t spojen�, pokud do�lo k timeoutu
        //net_on = 0;
        Serial.println("values: " + String(SEC_TIMER) + "/" + String(last_ping) + "/" + String(ping_timeout) + "/");
        setNetOn(0, 7);
        //delay(200);
        delay(5);
    }
    /*
        if (!client.connected() && !inSetup) {
              if (WiFi.status() == WL_CONNECTED) {
                if (client.connect(serverIP, serverPort)) {  // Zm��te na IP serveru a port
                    SEC_TIMER = rtc.getEpoch();
                    last_ping = SEC_TIMER;
                    net_on = 1;
                    //setNetOn(1,8);
                }
              } else {
                  net_on = 0;
                  //setNetOn(0,9);
              }
        }
    */
    if (!client.connected() && net_on == 1) {
        // Restart spojen�, pokud nen� p�ipojen� aktivn�
        client.stop();
        //delay(200);
        delay(5);
        if (client.connect(serverIP, serverPort)) {  // Zm��te na IP serveru a port

            //SEC_TIMER = rtc2.getEpoch();
            //SEC_TIMER = rtc2.now().unixtime();

            SEC_TIMER = millis() / 1000;

            last_ping = SEC_TIMER;
            //net_on = 1;
            setNetOn(1, 8);
        }
        else {
            //net_on = 0;
            setNetOn(0, 9);
        }
        //connectToServer();
    }

    if (client.connected()) {
        // �ten� dat z klienta
        while (client.available() > 0) {
            int bytes_read = client.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            buffer[bytes_read] = '\0';  // Ukon�ovac� znak pro �et�zec
            //SEC_TIMER = rtc2.getEpoch();
            SEC_TIMER = millis() / 1000;
            last_ping = SEC_TIMER;
            //last_ping = rtc2.getEpoch();
            //last_ping = rtc2.now().unixtime();

            logToSerial(buffer, 0);
            logToSerial("s:" + String(SEC_TIMER), 0);

            if ((buffer[0] == 'O') && (buffer[1] == 'K')) {
                last_ping = SEC_TIMER;

                //last_ping = rtc2.getEpoch();
                //last_ping = rtc2.now().unixtime();

                setNetOn(1, 14);
            };

            logToSerial(buffer, 0);

            String buffer2String = buffer + 2;
            buffer2String.trim();

            // Ov��en� p��choz�ho p��kazu
            switch (buffer[0]) {
            case 2:  // Nastaven� �asu
                logToSerial(buffer, 1);
                thetm.tm_sec = atoi(buffer + 19);
                buffer[18] = 0;
                thetm.tm_min = atoi(buffer + 16);
                buffer[15] = 0;
                thetm.tm_hour = atoi(buffer + 13);
                buffer[12] = 0;
                thetm.tm_mday = atoi(buffer + 10);
                buffer[9] = 0;
                //thetm.tm_mon = atoi(buffer + 7) - 1;
                thetm.tm_mon = atoi(buffer + 7);
                buffer[6] = 0;
                //thetm.tm_year = atoi(buffer + 2) - 1900;
                thetm.tm_year = atoi(buffer + 2);
                logToSerial(String(thetm.tm_year), 0);

                //ESP32 realtime clock
                //rtc.setTimeStruct(thetm);

                //rtc2.setEpoch(rtc.getEpoch());

               
                rtc2.stopClock();
                rtc2.setDate(thetm.tm_mday, thetm.tm_mon, atoi(buffer + 2));

                //rtc2.setDate(thetm.tm_mday, thetm.tm_mon, thetm.tm_year);
                rtc2.setTime(thetm.tm_hour, thetm.tm_min, thetm.tm_sec);

                rtc2.startClock();


                //rtc2.adjust( mktime(&thetm) );

                //SEC_TIMER = mktime(&thetm);

                thetm.tm_mon = thetm.tm_mon;
                thetm.tm_year = atoi(buffer + 2);

                SEC_TIMER = mktime(&thetm);
                //SEC_TIMER = millis() / 1000;
                timer_reset = SEC_TIMER + 86400;
                last_ping = SEC_TIMER;
                //logToSerial(String(thetm.tm_year) + "-" + String(thetm.tm_mon) + "-" + String(thetm.tm_mday), 1);
                //logToSerial(String(timer_reset) + " - " + String(SEC_TIMER),1);
                break;

            case 3:  // Z�sk�n� �asu
                strcpy(buffer, "\x02~");
                get_time(SEC_TIMER, buffer2);
                strcat(buffer, buffer2);
                strcat(buffer, "\n");
                client.print(buffer);
                break;

            case 1:  // Echo p��kaz
                client.print("OK\n");
                break;

            case 4:
                blank_line(3);
                //lcd2.printf("%s", buffer + 2);
				lcd2.printf("%s", buffer2String);
                break;
            case 5:
                blank_line(1);
                //lcd2.printf("%s", buffer + 2);
                lcd2.printf("%s", buffer2String);
                break;
            case 6:
                blank_line(0);
                //lcd2.printf("%s", buffer + 2);
				//lcd2.print(buffer + 2);

				//lcd2.printf("%s", buffer + 2);
                lcd2.printf("%s", buffer2String);
                break;
            case 7:
                blank_line(2);
                //lcd2.printf("%s", buffer + 2);
                lcd2.printf("%s", buffer2String);
                break;
            case 0xB:  // Nastaven� timeoutu
                timer1 = atoi(buffer + 2);
                break;
            case 0xC:
                timeout1 = 0;
                break;
            case 0xE:  // Zobrazen� zpr�vy na LCD
                lcd2.setCursor(0, 0);
                lcd2.printf("*------------------*");
                lcd2.setCursor(0, 1);
                lcd2.printf("|                  |");
                //lcd2.setCursor(((20 - strlen(buffer)) / 2) + 1, 1);
				logToSerial(buffer, 1);
                lcd2.setCursor(((20 - buffer2String.length()) / 2) + 0, 1);
                //lcd2.printf("%s", buffer + 2);
                lcd2.printf("%s", buffer2String);
                lcd2.setCursor(0, 2);
                lcd2.printf("*------------------*");

                saved = 1;
                timeout1 = SEC_TIMER + timer2;
                break;

            case 0x16:  // Vymaz�n� displeje
                saved = 1;
                timeout1 = SEC_TIMER + timer2;
                break;

            case 0x22:  // P��kaz k signalizaci
                strcpy(buffer, "\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07");
                last_ping = SEC_TIMER;
                for (counter1 = 0; counter1 <= 5; counter1++) {
                    Serial.write(buffer, 10);  // Signalizace pomoc� s�riov� komunikace
                    delay(20);  // 20 ms zpo�d�n�
                }
                break;

            default:
                break;
            }
            logToSerial("e:" + String(SEC_TIMER), 0);
            //delay(timeout1);
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

    // Zobrazen� stavu p�enosu na LCD
    lcd2.setCursor(0, 0);
    lcd2.printf("*------------------*");
    lcd2.setCursor(0, 1);
    lcd2.printf("|  PRENASIM DATA   |");
    lcd2.setCursor(0, 2);
    lcd2.printf("| DO PC - CEKEJTE  |");
    lcd2.setCursor(0, 3);
    lcd2.printf("*------------------*");

    // P��prava pro odesl�n� dat
    if (!client.connected() || client.write("\x06A-100\n") != 7) {
        //net_on = 0;
        setNetOn(0, 3);
        return;
    }
    client.flush();
    delay(1000);

    timeout2 = SEC_TIMER + 2;  // Timeout na odpov��
    tmp[0] = '\0';

    // �ek�n� na odpov�� od serveru
    while (((long)(SEC_TIMER - timeout2) <= 0) && (tmp[3] != '?')) {
        tmp[0] = '\0';
        server_read(tmp);
    }

    if (tmp[3] == '?') {
        timeout2 = SEC_TIMER + 30;  // Zak�z�n� p�episu displeje
        last_ping = SEC_TIMER;
        long l = fifo_first;

        // Odesl�n� dat, dokud je s� p�ipojena a existuj� data
        while ((fifo_first + 3 < fifo_last) && net_on) {
            memset(off_buffer, 0x00, off_buffer_size);
            // Simulace na�ten� dat do `off_buffer`
            // xmem2root(off_buffer, fifo_first, off_buffer_size);

            tmp_send_data[0] = '\0';
            i = k = 0;
            set_first = 0;

            // V�b�r a form�tov�n� dat pro odesl�n�
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

            // Kontrola p�ipojen� a odesl�n� dat
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
                            tmp[strlen(tmp) - 1] = '\0';  // Odstran�n� znaku '\n'
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

            // Zobrazen� pokroku na LCD2
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

        // Uvoln�n� pam�ti FIFO
        if ((long)(fifo_first + 3 >= fifo_last)) {
            fifo_last = fifo_start;
            fifo_first = fifo_start;
        }
    }
    else {
        // Reset spojen� p�i ne�sp�chu
        client.stop();
        net_on = 0;
        delay(200);
    }
}

int server_read(char* buffer) {
    int len = 0;
    char tmp[2] = { 0 };  // Vyrovn�vac� pam� pro jednotliv� znaky
    unsigned long timeout = SEC_TIMER + 2;
    bool ok = false;
    *buffer = '\0';  // Inicializace bufferu na pr�zdn� �et�zec

    while (!ok && ((long)(SEC_TIMER - timeout) <= 0)) {
        // �ekejte na data
        if (client.available()) {
            len = client.readBytes(tmp, 1);  // P�e�t�te jeden znak
            tmp[1] = '\0';  // Ukon�ovac� znak pro spr�vn� spojen� s �et�zcem
            if (len > 0) {
                strcat(buffer, tmp);
                // Pokud naraz�me na nov� ��dek, ukon�ujeme �ten�
                if (tmp[0] == '\n') {
                    ok = true;
                }
            }
        }

        // Pokud ��dn� data nep�ich�z�, ukon��me smy�ku
        if (strlen(buffer) == 0) {
            ok = true;
        }
    }

    return strlen(buffer);  // Vrac� d�lku na�ten�ch dat
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
    Serial.println("Kontrola aktualizac�...");

    // Sta�en� souboru s verz� z serveru
    HTTPClient http;
    // Explicitn� za��n�me HTTP spojen� se specifikac� portu a timeoutu
    http.begin(versionURL);
    http.setTimeout(10000); // Nastaven� del��ho timeoutu (10 sekund)

    // P�id�n� hlavi�ek pro simulaci b�n�ho browseru
    http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
    http.addHeader("Accept", "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8");
    http.addHeader("Accept-Language", "cs,en-US;q=0.7,en;q=0.3");
    http.addHeader("Connection", "keep-alive");

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String serverVersion = http.getString();
        serverVersion.trim(); // odstran� b�l� znaky
        Serial.print("Verze na serveru: ");
        Serial.println(serverVersion);
        Serial.print("Verze na lok�ln�: ");
        Serial.println(localVersion);

        String localVersionTmp = localVersion;
        localVersionTmp.replace(".", "");
        String serverVersionTmp = serverVersion;
        serverVersionTmp.replace(".", "");

        int localVersionInt = localVersionTmp.toInt();
        int serverVersionInt = serverVersionTmp.toInt();

        if (serverVersionInt > localVersionInt) {
            Serial.println("Nov� verze nalezena, zahajuji aktualizaci...");

            // Vytvo��me nov� HTTPClient pro sta�en� aktualizace
            HTTPClient updateClient;

            // P�id�n� hlavi�ek pro simulaci b�n�ho browseru i pro update request
            updateClient.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
            updateClient.addHeader("Accept", "*/*");
            updateClient.addHeader("Accept-Language", "cs,en-US;q=0.7,en;q=0.3");
            updateClient.addHeader("Connection", "keep-alive");
            updateClient.setTimeout(30000); // Del�� timeout pro stahov�n� firmware (30 sekund)

            Serial.print("P�ipojuji se k: ");
            Serial.println(firmwareURL);

            // Zkus�me nejprve p��mou metodu s pou�it�m WiFiClient m�sto HTTPClient
            WiFiClient client;

            Serial.println("Pokou��m se o aktualizaci p�es WiFiClient...");
            t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

            // Pokud prvn� metoda sel�e, zkus�me p�vodn� metodu s HTTPClient
            if (ret == HTTP_UPDATE_FAILED) {
                Serial.println("Prvn� metoda selhala, zkou��m alternativn� metodu...");
                ret = httpUpdate.update(updateClient, String(firmwareURL));
            }

            switch (ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("Aktualizace selhala: %d - %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                // Dodate�n� debugov�n�
                Serial.print("URL firmware: ");
                Serial.println(firmwareURL);
                break;
            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("Nen� k dispozici ��dn� nov� aktualizace.");
                break;
            case HTTP_UPDATE_OK:
                Serial.println("Aktualizace prob�hla �sp�n�.");
                break;
            }
        }
        else {
            Serial.println("Firmware je aktu�ln�.");
        }
    }
    else {
        Serial.printf("Nelze z�skat verzi, HTTP k�d: %d\n", httpCode);
    }
    http.end();
}

// Funkce, kter� b�� jako samostatn� vl�kno
void otaUpdateTask(void* parameter) {
    Serial.println("OTA task started");

    while (true) {
        // Kontrola dostupnosti aktualizace
        if (!otaInProgress) {
            checkForUpdatesBackground();
        }

        // Pokud je aktualizace dostupn�, prove� ji
        if (otaUpdateAvailable && !otaInProgress) {
            performUpdate();
        }

        // Po�kej na dal�� kontrolu (nap�. 1 hodina)
        vTaskDelay(60000 / portTICK_PERIOD_MS); // 1 hodina
    }
}

// Kontroluje, zda je dostupn� nov� verze
void checkForUpdatesBackground() {
    Serial.println("Kontrola aktualizac� na pozad�...");

    // Sta�en� souboru s verz� z serveru
    HTTPClient http;
    http.begin(versionURL);
    http.setTimeout(10000);

    // P�id�n� hlavi�ek pro simulaci b�n�ho browseru
    http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
    http.addHeader("Accept", "text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8");

    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        String serverVersion = http.getString();
        serverVersion.trim(); // odstran� b�l� znaky

        Serial.print("Verze na serveru: ");
        Serial.println(serverVersion);
        Serial.print("Aktu�ln� verze: ");
        Serial.println(currentVersion);

        String localVersionTmp = currentVersion;
        localVersionTmp.replace(".", "");
        String serverVersionTmp = serverVersion;
        serverVersionTmp.replace(".", "");

        int localVersionInt = localVersionTmp.toInt();
        int serverVersionInt = serverVersionTmp.toInt();

        if (serverVersionInt > localVersionInt) {
            Serial.println("Nov� verze nalezena!");
            otaUpdateAvailable = true;
            newVersion = serverVersion;
        }
        else {
            Serial.println("Firmware je aktu�ln�.");
            otaUpdateAvailable = false;
        }
    }
    else {
        Serial.printf("Nelze z�skat verzi, HTTP k�d: %d\n", httpCode);
    }
    http.end();
}

// Provede samotnou aktualizaci
void performUpdate() {
    Serial.println("Zahajuji aktualizaci na pozad�...");
    otaInProgress = true;

    // Vytvo�en� indikace na displeji (pokud je pot�eba)
    // displayUpdateStatus("Aktualizace...");

    // Pou�ijeme p��mou metodu s WiFiClient, kter� fungovala
    WiFiClient client;

    t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

    switch (ret) {
    case HTTP_UPDATE_FAILED:
        Serial.printf("Aktualizace selhala: %d - %s\n",
            httpUpdate.getLastError(),
            httpUpdate.getLastErrorString().c_str());
        // Resetujeme p��znaky pro p��padn� dal�� pokus
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_NO_UPDATES:
        Serial.println("Nen� k dispozici ��dn� nov� aktualizace.");
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_OK:
        Serial.println("Aktualizace prob�hla �sp�n�, restartuju...");
        // P�i �sp�n� aktualizaci dojde k restartu ESP32
        break;
    }
}

// Funkce pro inicializaci OTA �lohy - volejte v setup()
void setupOTA() {
    // Nastav�me aktu�ln� verzi
    //currentVersion = "1.0.0.0";  // Pro testov�n�, v re�lu by se �etla z konstanty

    // Vytvo��me �lohu na j�d�e 0 (hlavn� aplikace b�� typicky na j�d�e 1)
    xTaskCreatePinnedToCore(
        otaUpdateTask,      // Funkce, kter� implementuje �lohu
        "OTATask",          // N�zev �lohy
        8192,               // Velikost z�sobn�ku (v slovech)
        NULL,               // Parametr �lohy
        1,                  // Priorita �lohy (ni��� ��slo = ni��� priorita)
        &otaTaskHandle,     // Handle �lohy
        0                   // J�dro CPU, na kter�m m� �loha b�et (0 nebo 1)
    );
}

//************************************************************************
// Nov� funkce pro odesl�n� bufferu ze souboru aptbuffer.txt
//************************************************************************
void send_file_buffer() {
    // 1. Zkontrolujeme, zda jsme online a p�ipojeni
    if (!net_on || !client.connected()) {
        return; // Nejsme online nebo p�ipojeni, nem� smysl pokra�ovat
    }

    // 2. Zkus�me otev��t soubor bufferu pro �ten�
    File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
    if (!bufferFile || bufferFile.size() == 0) {
        if (bufferFile) bufferFile.close();
        return; // Soubor neexistuje nebo je pr�zdn�
    }

    // 3. P�e�teme prvn� ��dek (zpr�vu) ze souboru
    String lineToSend = "";
    if (bufferFile.available()) {
        lineToSend = bufferFile.readStringUntil('\n');
    }

    if (lineToSend.length() == 0) {
        bufferFile.close(); // Nic jsme nep�e�etli (mo�n� chyba nebo jen pr�zdn� ��dek na za��tku)
        // M��eme zkusit soubor promazat, pokud obsahuje jen b�l� znaky? Prozat�m ne.
        return;
    }

    // P�id�me zp�t znak nov�ho ��dku, pokud ho readStringUntil odstranil a server ho vy�aduje
    lineToSend += "\n";

    Serial.print("Attempting to send from file buffer: ");
    Serial.print(lineToSend);

    // 4. Pokus�me se odeslat prvn� ��dek
    bool send_success = false;
    if (client.print(lineToSend)) {
        client.flush(); // Po�k�me na odesl�n�
        Serial.println(" - Sent, waiting for confirmation...");

        // 5. �ek�me na potvrzen� od serveru (podobn� jako v p�vodn�m send_buffer)
        //    Pou�ijeme jednoduch� mechanismus �ten� s timeoutem
        unsigned long confirm_timeout = millis() + 3000; // Timeout 3 sekundy na odpov��
        String response = "";
        while (millis() < confirm_timeout) {
            if (client.available()) {
                response = client.readStringUntil('\n');
                response.trim(); // Odstran�me b�l� znaky
                Serial.print(" - Received response: ");
                Serial.println(response);
                // Zde p�edpokl�d�me, �e server po�le n�co specifick�ho pro potvrzen�
                // P�vodn� k�d �ekal na odpov�� za��naj�c� na 0x7E (~)
                // Upravte podm�nku podle va�� serverov� logiky
                if (response.length() > 0 && response.startsWith("~")) { // P��klad: odpov�� za��n� '~'
                    send_success = true;
                    last_ping = SEC_TIMER; // Aktualizujeme last_ping
                    break;
                }
            }
            delay(10); // Kr�tk� pauza, aby neb�ela smy�ka naplno
        }
        if (!send_success) {
            Serial.println(" - Confirmation failed or timed out.");
        }

    }
    else {
        Serial.println(" - Client print failed.");
        setNetOn(0, 20); // Pravd�podobn� probl�m se spojen�m
        bufferFile.close();
        return;
    }

    // 6. Pokud odesl�n� a potvrzen� prob�hlo �sp�n�, p�ep�eme soubor bez odeslan�ho ��dku
    if (send_success) {
        Serial.println(" - Send successful. Removing line from buffer file.");

        // Otev�eme do�asn� soubor pro z�pis
        File tempFile = LittleFS.open("/apt1220/aptbuffer.tmp", FILE_WRITE);
        if (!tempFile) {
            Serial.println("Error: Failed to open temporary buffer file for writing!");
            bufferFile.close();
            return;
        }

        // Zkop�rujeme zbytek p�vodn�ho souboru (od druh�ho ��dku d�l) do do�asn�ho
        char copyBuf[128]; // Buffer pro kop�rov�n�
        while (bufferFile.available()) {
            int bytesRead = bufferFile.readBytes(copyBuf, sizeof(copyBuf));
            if (bytesRead > 0) {
                tempFile.write((uint8_t*)copyBuf, bytesRead);
            }
        }

        // Zav�eme oba soubory
        bufferFile.close();
        tempFile.close();

        // Sma�eme p�vodn� soubor a p�ejmenujeme do�asn�
        if (LittleFS.remove(bufferFilePath)) {
            if (LittleFS.rename("/apt1220/aptbuffer.tmp", bufferFilePath)) {
                Serial.println(" - Buffer file updated successfully.");
            }
            else {
                Serial.println("Error: Failed to rename temporary buffer file!");
                // Pokus�me se smazat i do�asn� soubor, pokud p�ejmenov�n� selhalo
                LittleFS.remove("/apt1220/aptbuffer.tmp");
            }
        }
        else {
            Serial.println("Error: Failed to remove original buffer file!");
            // Pokus�me se smazat i do�asn� soubor
            LittleFS.remove("/apt1220/aptbuffer.tmp");
        }

    }
    else {
        // Pokud odesl�n� selhalo, soubor nem�n�me, zav�eme ho a zkus�me to znovu pozd�ji
        Serial.println(" - Send failed. Buffer file remains unchanged.");
        bufferFile.close();
    }
}