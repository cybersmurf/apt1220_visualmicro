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

///vypnutí použití EEPROM a EspConfigLib
//#include "uEEPROMLib.h"
//#include "uEspConfigLib.h"

#include <Preferences.h>

#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include <WiFiClientSecure.h>   // Needed for HTTPS

#include "esp_system.h"
#include "esp_mac.h"

#include "esp_heap_caps.h"   // (nové) na měření HEAPu/DMA

#define SPIFFS LittleFS
#define FORMAT_LITTLEFS_IF_FAILED true

#define DEBUG_MODE

//#include <ESP32FTPClient.h> // Include the FTP client library


// Caution: It need to be a global variable or a global pointer.
// if FS is a 'setup' variable it will lead to crashes
///vypnutí použití EEPROM a EspConfigLib
//uEspConfigLibFSInterface* configFsC;
//uEspConfigLib* config;

// uEEPROMLib eeprom;
///vypnutí použití EEPROM a EspConfigLib
//uEEPROMLib eeprom(0x50);

static DS1307 rtc2;
//uRTCLib rtc2;

LCDI2C_Latin_Symbols lcd2(0x27, 20, 4);    // I2C address = 0x27; LCD = Surenoo SLC1602A (European)
WiFiClient client;
//WebServer server(80);
AsyncWebServer server(80);

//ESP32FTPClient ftpClient;

// Inicializace sériových portů
HardwareSerial SerialC(1); // Pro SerialC
HardwareSerial SerialD(2); // Pro SerialD

#define TCPCONFIG 1
#define VERZE_PRACANT  "4.0.0"
#define ALOC_MEM 64000
#define SERB_USEPORTD
#define timeout_reset 86400  // jak casto resetovat

// commands of terminal
#define C_ERROR_DATA     $00  // nekorektni data
#define C_ECHO           $01  // Pošli odezvu        PC <-> TERMINAL
#define C_SET_TIME       $02  // Nastav čas          PC  -> TERMINAL
#define C_GET_TIME       $03  // Pošli čas           PC <-> TERMINAL
#define C_DATA_WORKER    $04  // Posilam zamestnance PC <-> TERMINAL

#define PORT 		     54321

#define IF_DOWN		     0
#define IF_COMING_UP	 1
#define IF_UP			 2
#define IF_COMING_DOWN	 3

#define RTC_YEAR_OFFSET  1900

#define STACK_WORDS(bytes) ((bytes) / sizeof(StackType_t))

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

static int __attribute__((section(".noinit")))  rfid_reader_c;      // 0 - normalni čtečka
static int __attribute__((section(".noinit")))  id12_c;             // 0 - normalni čtečka
// 1 - rfid modul
static int __attribute__((section(".noinit")))  rfid_reader_d;      // 0 - normalni čtečka
static int __attribute__((section(".noinit")))  id12_d;             // 0 - normalni čtečka
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

static char ap_ssid[32] = "APT1220_AP";   // fixní část SSID
static char ap_pass[64] = "12345678";     // heslo AP
static bool ap_ssid_use_mac = true;           // true = přidá MAC, false = použije fixní ap_ssid


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
static String localVersion = "1.0.1.6";

String newVersion = "";

// URL k souboru, kde je uvedena aktuální verze firmware na serveru
const char* versionURL = "https://petrsramek.eu/emistr/apt1220/version.txt";
// URL k novému firmware (binární soubor)
const char* firmwareURL = "https://petrsramek.eu/emistr/apt1220/firmware.bin";

const char* firmwareFTP = "update.emistr.cz";
const char* firmwareFTPPath = "/apt1220";
const char* firmwareFTPUser = "emistrUpdate";
const char* firmwareFTPPassword = "'X9SeQ$+#29d*b7#N'";

// Globální proměnné pro OTA aktualizaci
TaskHandle_t otaTaskHandle = NULL;
bool otaInProgress = false;
bool otaUpdateAvailable = false;

Preferences preferences;

static bool saveNewConfigData = false; // Flag pro uložení nových dat do konfigurace
static bool restartNetwork    = false; // Flag pro restartování sítě

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

// Struktura pro přenos dat z čteček do fronty
struct SerialData_t {
    char data[100]; // Buffer pro data
    int port;       // Port, ze kterého data přišla (1 pro C, 2 pro D)
};

// Handle pro frontu, která bude držet příchozí data
QueueHandle_t serialDataQueue;

SemaphoreHandle_t i2cMutex; // Mutex pro ochranu I2C sběrnice (LCD a RTC)

static bool apSettingsChanged = false;   // nastavíš na true při změně SSID/hesla/flagu
static bool pendingApRestart = false;   // nastavíš na true, když je potřeba restartovat AP

TaskHandle_t hCommandProcessor = nullptr;
TaskHandle_t hSerialCReader = nullptr;
TaskHandle_t hSerialDReader = nullptr;


static inline bool isApRunning() {
    return (WiFi.getMode() & WIFI_MODE_AP) != 0;   // AP nebo AP+STA
}

void restartAPIfRunning() {
    if (isApRunning()) {
        WiFi.softAPdisconnect(true);  // zastaví AP (a odpojí klienty)
        delay(200);
        startAPMode();                // znovu spustí AP s novým SSID/heslem
    }
}


// ===== Hostname & MAC helpers =====
static String macToStr(const uint8_t mac[6], bool withDelims, char delim = '-') {
    char buf[18];
    if (withDelims) {
        snprintf(buf, sizeof(buf), "%02X%c%02X%c%02X%c%02X%c%02X%c%02X",
            mac[0], delim, mac[1], delim, mac[2], delim, mac[3], delim, mac[4], delim, mac[5]);
    }
    else {
        snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return String(buf);
}

static String readStaMacStr(bool delims = false) {
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return macToStr(mac, delims);
}

static String readApMacStr(bool delims = false) {
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    return macToStr(mac, delims);
}

static String readEthMacStr(bool delims = false) {
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_ETH);
    return macToStr(mac, delims);
}

static String macNoDelims(String mac) {
    mac.toUpperCase();
    mac.replace(":", "");
    mac.replace("-", "");
    return mac;
}

static String makeHost(const char* prefix, const String& mac) {
    return String(prefix) + macNoDelims(mac);
}

// Wi-Fi: nastav hostname z Wi-Fi STA MAC (volat PŘED WiFi.begin)
static void setWiFiHostnameFromMac() {
    String host = makeHost("APT1220-", WiFi.macAddress());
    WiFi.setHostname(host.c_str());
    // Pro zobrazení/logy si držíme i "aktivní MAC" v hezkém formátu
    activeMAC = WiFi.macAddress();
    activeMAC.replace(":", "-");
}

// Ethernet: nastav hostname z ETH MAC (ideálně při ETH_START/GOT_IP)
static void setEthHostnameFromMac() {
    String host = makeHost("APT1220E-", ETH.macAddress());
    ETH.setHostname(host.c_str());
    activeMAC = ETH.macAddress();
    activeMAC.replace(":", "-");
}


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
    // Pokusíme se připojit LittleFS, formátovat pokud selže
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        Serial.println("LittleFS Mount Failed initially.");

        // --- Přidáno: Explicitní pokus o formátování ---
        Serial.println("Attempting to format LittleFS...");
        if (LittleFS.format()) {
            Serial.println("LittleFS formatted successfully.");
            // Zkusíme připojit znovu po formátování
            if (!LittleFS.begin()) {
                Serial.println("LittleFS Mount Failed even after format!");
                return; // Ukončíme, pokud ani po formátu nelze připojit
            }
            Serial.println("LittleFS Mounted successfully after format.");
        }
        else {
            Serial.println("LittleFS format failed.");
            return; // Ukončíme, pokud formátování selže
        }
        // --- Konec přidaného kódu ---

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

        File bufferFile = LittleFS.open(bufferFilePath, FILE_READ); // Otevřeme soubor pro čtení
        if (bufferFile) {
            size_t fileSize = bufferFile.size(); // Získáme velikost souboru
            Serial.print(" -> Size: ");
            Serial.print(fileSize);
            Serial.println(" bytes");
            bufferFile.close(); // Zavřeme soubor
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

            // Nastav hostname z ETH MAC co nejdříve před DHCP
            String hostE = "APT1220E-" + readEthMacStr(false);
            ETH.setHostname(hostE.c_str());

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
            // Počkáme chvilku, aby se stihly eventy zpracovat a DHCP priradit IP
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

    // eth_connected se nastavuje globálně v eventu ARDUINO_EVENT_ETH_GOT_IP
    // Dáme mu chvilku, aby se to stihlo stát
    unsigned long startAttempt = millis();
    while (!eth_connected && useETH && (millis() - startAttempt < 5000)) {
        delay(100); // Počkáme až 5 sekund na event, že jsme dostali IP
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
    //xTaskCreatePinnedToCore(tSEC_TIMERcode, "tSEC_TIMER", 4096, NULL, 0, &tSEC_TIMER, 1);
    //xTaskCreatePinnedToCore(tLAST_PINGcode, "tLAST_PING", 4096, NULL, 0, &tLAST_PING, 1);

    xTaskCreatePinnedToCore(tSEC_TIMERcode, "tSEC_TIMER", STACK_WORDS(1536), NULL, 0, &tSEC_TIMER, 1);
    xTaskCreatePinnedToCore(tLAST_PINGcode, "tLAST_PING", STACK_WORDS(1536), NULL, 0, &tLAST_PING, 1);
}

void initializeDisplay() {
    lcd2.init();
    lcd2.backlight();
    lcd2.createChar(0, customChar);
    lcd2.setAutoNewLine(false); // Vypne automatické posouvání na nový řádek
    lcd2.clear();
    delay(200);
    lcd2.printf("       eMISTR       ");
    lcd2.setCursor(0, 2);
    lcd2.printf("Verze: %s", VERZE_PRACANT);
    lcd2.setCursor(0, 3);
    lcd2.printf("Verze FW: %s", localVersion);
    delay(200);
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

    localVersion = "1.0.1.6";

    // KROK 1: INICIALIZACE SYNCHRONIZAČNÍCH PRIMITIV
    // Zavoláme naši novou funkci hned na začátku. Tím zajistíme,
    // že všechny mutexy a fronty budou připravené pro tasky,
    // které se spustí později v setupu.
    initializeSyncPrimitives();

    rtc2.setHourMode(CLOCK_H24);

    if (!rtc2.isRunning()) {
        Serial.println("RTC is NOT running, let's set the time!");
        rtc2.startClock();
    }

    initializeHardware();
    initializeFileSystem();

    delay(100);

    loadConfiguration();

    initializeNetwork();

    initializeTasks();

    // Inicializace OTA
    setupOTA();

    initializeDisplay();

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

    timeout1 = 0;
    timer_reset = SEC_TIMER + 86400;
    // Denní reset 


    // Vytvoříme task pro zpracování příkazů z fronty
    /*
    xTaskCreatePinnedToCore(
        commandProcessorTask,   // Funkce tasku
        "CommandProcessor",     // Jméno
        4096,                   // Stack
        NULL,                   // Parametry
        2,                      // Priorita (dáme jí vyšší než čtecím taskům)
        NULL,                   // Handle
        1                       // Spustíme na jádře 1
    );

    // Vytvoříme task pro čtení ze SerialC
    xTaskCreatePinnedToCore(
        serialReaderTask,       // Funkce tasku
        "SerialC_Reader",       // Jméno
        4096,                   // Stack
        (void*)&SerialC,        // Parametr - předáme ukazatel na SerialC
        1,                      // Priorita
        NULL,                   // Handle
        1                       // Spustíme také na jádře 1
    );

    // Vytvoříme task pro čtení ze SerialD
    xTaskCreatePinnedToCore(
        serialReaderTask,       // Funkce tasku
        "SerialD_Reader",       // Jméno
        4096,                   // Stack
        (void*)&SerialD,        // Parametr - předáme ukazatel na SerialD
        1,                      // Priorita
        NULL,                   // Handle
        1                       // Spustíme také na jádře 1
    );
    */

    xTaskCreatePinnedToCore(commandProcessorTask, "CommandProcessor", STACK_WORDS(3072), NULL, 2, &hCommandProcessor, 1);
    xTaskCreatePinnedToCore(serialReaderTask, "SerialC_Reader", STACK_WORDS(2048), (void*)&SerialC, 1, &hSerialCReader, 1);
    xTaskCreatePinnedToCore(serialReaderTask, "SerialD_Reader", STACK_WORDS(2048), (void*)&SerialD, 1, &hSerialDReader, 1);

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
 * @brief Task pro čtení dat ze sériového portu.
 * Tato funkce běží v nekonečné smyčce a je použitelná pro oba sériové porty.
 * Jako parametr dostane ukazatel na HardwareSerial objekt.
 */
void serialReaderTask(void* parameter) {
    HardwareSerial* serialPort = (HardwareSerial*)parameter;
    int portNumber = (serialPort == &SerialC) ? 1 : 2;
    SerialData_t receivedData; // Vytvoříme si strukturu pro data

    for (;;) { // Nekonečná smyčka tasku
        if (serialPort->available()) {
            size_t len = serialPort->readBytesUntil('\n', receivedData.data, sizeof(receivedData.data) - 1);
            if (len > 0) {
                receivedData.data[len] = '\0'; // Ukončíme řetězec
                receivedData.port = portNumber;

                // Pošleme data do fronty. Pokud je fronta plná, počkáme max 100ms.
                if (xQueueSend(serialDataQueue, &receivedData, pdMS_TO_TICKS(100)) != pdTRUE) {
                    Serial.printf("CHYBA: Fronta pro seriová data je plná, data z portu %d zahozena!\n", portNumber);
                }
            }
        }
        // Dáme plánovači šanci, aby se mohl věnovat jiným úlohám.
        // Task se na 20ms uspí a nežere zbytečně CPU.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Task pro zpracování dat z fronty.
 * Čeká, až se ve frontě objeví data, a pak je zpracuje voláním funkce serial().
 */
void commandProcessorTask(void* parameter) {
    SerialData_t dataToProcess;

    for (;;) {
        // Čekáme na data ve frontě. Task je blokován (nežere CPU), dokud něco nepřijde.
        // portMAX_DELAY znamená, že bude čekat "věčně".
        if (xQueueReceive(serialDataQueue, &dataToProcess, portMAX_DELAY) == pdTRUE) {
            // Máme nová data, zavoláme tvou původní funkci pro zpracování
            Serial.printf("Data z fronty (port %d): %s\n", dataToProcess.port, dataToProcess.data);
            serial(dataToProcess.data, dataToProcess.port);
        }
    }
}

// "Hloupá" verze - jen čistí displej, nestará se o mutex
void _fast_clear_disp_unsafe() {
    lcd2.clear();
    lcd2.home();
}

// "Chytrá" verze - wrapper, který se stará o bezpečnost
void fast_clear_disp() {
    // Vezmeme klíč
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {

        _fast_clear_disp_unsafe(); // Zavoláme "hloupou" funkci

        // Vrátíme klíč
        xSemaphoreGive(i2cMutex);
    }
}

void _blank_line_unsafe(int line) {
    lcd2.setCursor(0, line);                       // Umístí kurzor na začátek řádku
    lcd2.print("                    ");           // Vypíše prázdný řetězec o 20 mezerách
    lcd2.setCursor(0, line);                       // Umístí kurzor zpět na začátek řádku
}

// "Chytrá" verze - teď se stará o zamykání
void blank_line(int line) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        _blank_line_unsafe(line); // Volá hloupou verzi
        xSemaphoreGive(i2cMutex);
    }
}

/**
 * @brief Bezpečně zapíše data do TCP klienta. Stará se o zamčení a odemčení.
 * @param data Ukazatel na data k odeslání.
 * @return true pokud se odeslání podařilo, jinak false.
 */
bool tcp_print_safe(const char* data) {
    bool success = false;
    // Počkáme si na klíč (mutex)
    if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (client.connected()) {
            client.print(data);
            success = true;
        }
        // Vrátíme klíč
        xSemaphoreGive(tcpMutex);
    }
    else {
        Serial.println("CHYBA: tcp_print_safe nemohl ziskat TCP mutex!");
    }
    return success;
}

/**
 * @brief Bezpečně se připojí k TCP serveru.
 */
void connectToServer_safe() {
    // Vezmeme si klíč
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
        // Vrátíme klíč
        xSemaphoreGive(tcpMutex);
    }
    else {
        Serial.println("CHYBA: connectToServer_safe nemohl ziskat TCP mutex!");
    }
}

boolean load_config() {
    //načtení konfigurace z Preferences
    preferences.begin("my-app", true);

	snprintf(ip_adr, sizeof(ip_adr), "%s", preferences.getString("ip_adr", "192.168.222.202").c_str());
	snprintf(ip_mask, sizeof(ip_mask), "%s", preferences.getString("ip_mask", "255.255.255.0").c_str());
    snprintf(ip_server, sizeof(ip_server), "%s", preferences.getString("ip_server", "192.168.225.221").c_str());
    snprintf(ip_gate, sizeof(ip_gate), "%s", preferences.getString("ip_gate", "192.168.222.222").c_str());

    timer1 = preferences.getInt("timer1", 5);
    timer2 = preferences.getInt("timer2", 3);
    key_maker = preferences.getInt("key_maker", 0);
    rfid_reader_c = preferences.getInt("rfid_reader_c", 1);
    rfid_reader_d = preferences.getInt("rfid_reader_d", 0);
    id12_c = preferences.getInt("id12_c", 1);
    id12_d = preferences.getInt("id12_d", 0);

    snprintf(op_c, sizeof(op_c), "%s", preferences.getString("op_c", "A3997").c_str());
	snprintf(op_d, sizeof(op_d), "%s", preferences.getString("op_d", "").c_str());

    useWifi = preferences.getBool("useWifi", false);
    useETH = preferences.getBool("useETH", true);

	useDHCP = preferences.getBool("useDHCP", true);

    snprintf(ssid, sizeof(ssid), "%s", preferences.getString("SSID_NAME", "AGERIT_AC 2GHz").c_str());
	snprintf(password, sizeof(password), "%s", preferences.getString("SSID_PASS", "AGERITagerit512").c_str());

    snprintf(ap_ssid, sizeof(ap_ssid), "%s", preferences.getString("AP_SSID", "APT1220_AP").c_str());
    snprintf(ap_pass, sizeof(ap_pass), "%s", preferences.getString("AP_PASS", "12345678").c_str());
    ap_ssid_use_mac = preferences.getBool("AP_USE_MAC", true);

    ping_timeout = 4;
    preferences.end();
    serverIP = ip_server;

    return true;
}

boolean save_config() {
    //uložení konfigurace do Preferences
    preferences.begin("my-app", false);
    preferences.putString("ip_adr", ip_adr);
    preferences.putString("ip_mask", ip_mask);
    preferences.putString("ip_server", ip_server);
    preferences.putString("ip_gate", ip_gate);
    preferences.putString("op_c", op_c);
    preferences.putString("op_d", op_d);

    preferences.putInt("timer1", timer1);
    preferences.putInt("timer2", timer2);
    preferences.putInt("key_maker", key_maker);
    preferences.putInt("rfid_reader_c", rfid_reader_c);
    preferences.putInt("rfid_reader_d", rfid_reader_d);
    preferences.putInt("id12_c", id12_c);
    preferences.putInt("id12_d", id12_d);

    preferences.putBool("useWifi", useWifi);
    preferences.putBool("useETH", useETH);
	preferences.putBool("useDHCP", useDHCP);

    preferences.putString("SSID_NAME", ssid);
    preferences.putString("SSID_PASS", password);

    preferences.putString("AP_SSID", ap_ssid);
    preferences.putString("AP_PASS", ap_pass);
    preferences.putBool("AP_USE_MAC", ap_ssid_use_mac);

    preferences.end();
    return true;
}

void set_default() {
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

    snprintf(op_c, sizeof(op_c), "A3997");
	snprintf(op_d, sizeof(op_d), "");

    useDHCP = true;
    ping_timeout = 4;

    loaded_default = 1;

    snprintf(ap_ssid, sizeof(ap_ssid), "APT1220_AP");
    snprintf(ap_pass, sizeof(ap_pass), "12345678");
    ap_ssid_use_mac = true;

    saveNewConfigData = true;
}

void init_lcd() {
    lcd2.init();
    lcd2.clear();                             // Initialize the LCD
    lcd2.backlight();                         // Turn on the LCD backlight
    lcd2.noAutoscroll();
}

void menu_dump() {
    char long_str[20];  // Vyrovnávací paměť pro převedená čísla
            
    lockI2C();  // Zámek I2C sběrnice pro bezpečný přístup k LCD
	_fast_clear_disp_unsafe(); // Vyčištění displeje

    // Zobrazení hodnoty fifo_start
    lcd2.setCursor(0, 0);
    lcd2.printf("START:");
    ltoa(fifo_start, long_str, 10);  // Převede `fifo_start` na řetězec
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
	unlockI2C();  // Uvolnění I2C zámku
}

void menuconfig() {

    lockI2C();  // Zámek I2C sběrnice pro bezpečný přístup k LCD
    long timeout_up;
	_fast_clear_disp_unsafe(); // Vyčištění displeje
    lcd2.setCursor(0, 0);
    lcd2.printf("IP:");
    //lcd2.printf(ip_adr);

    if (useDHCP) {
        char ip_adr2[16];
        if (useETH) { strcpy(ip_adr2, ETH.localIP().toString().c_str()); }
        else { strcpy(ip_adr2, WiFi.localIP().toString().c_str()); }
        lcd2.printf(ip_adr2);
		//delete ip_adr2; // Uvolnění paměti, pokud bylo použito dynamické alokování
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
	unlockI2C();  // Uvolnění I2C zámku
}

void menuconfigwifi() {
    lockI2C();  // Zámek I2C sběrnice pro bezpečný přístup k LCD
    long timeout_up;
	_fast_clear_disp_unsafe(); // Vyčištění displeje

    lcd2.setCursor(0, 0);
    lcd2.printf("SSID:");
    lcd2.printf(ssid);
    lcd2.setCursor(0, 3);
    lcd2.printf("VERZE: ");
    //lcd2.printf(" %s  ", VERZE_PRACANT);
    lcd2.printf(" %s  ", localVersion);
	unlockI2C();  // Uvolnění I2C zámku
}

void resetConfig() {
    // Zavřete existující TCP připojení
    if (client.connected()) {
        client.stop();
        Serial.println("TCP client disconnected");
    }

    // Restart WiFi připojení
    WiFi.disconnect(true); // Odpojí a zapomene připojení
    delay(1000);           // Počkejte na odpojení

    // Nastavení statické IP (pokud je požadována statická IP)
    if (!WiFi.config(IPAddress(ip_adr), IPAddress(ip_gate), IPAddress(ip_mask))) {
        Serial.println("STA Failed to configure");
    }

    Serial.println(useWifi ? "WIFI connect" : "ETH connect");

    // Připojte se znovu k WiFi síti
    if (useWifi) {
        connectToWiFi();
    }

    // Starth Ethernet (this does NOT start WiFi at the same time)
    if (useETH) {
        Serial.print("Starting ETH interface...");
        ETH.begin();
    }

    // Inicializace FIFO proměnných
      //fifo_start = malloc(ALOC_MEM);
      //fifo_end =fifo_start+ALOC_MEM;

    fifo_first = fifo_start;
    fifo_last = fifo_start;

    // Ověření a nastavení časovačů
    if (timer1 > 60 || timer1 < 1) timer1 = 10;
    if (timer2 > 60 || timer2 < 1) timer2 = 5;

    // Otevřete nové TCP připojení
    connectToServer_safe();
	Serial.println("TCP client reinitialized");
}

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer pro nová data
    char buffer2[100];       // Buffer pro sériový vstup

    // TOTO JE TEN TRIK:
    // Nastavíme timeout pro překreslení hlavní obrazovky na dalekou budoucnost (třeba hodinu).
    // Tím zajistíme, že zatímco my tady čekáme na vstup, funkce loop() nezavolá tDEMOscreen()
    // a nepokusí se nám sáhnout na I2C sběrnici.
    timeout1 = SEC_TIMER + 3600;

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

            // Překreslíme zadaný text jen pokud přišlo něco nového
            if (input_processed) {
                lcd2.setCursor(4, 2);
                lcd2.print("                "); // Vymažeme starý text
                lcd2.setCursor(4, 2);
                lcd2.print(new_data);
            }

            // Dáme šanci ostatním taskům, ale ne moc velkou, ať je odezva svižná.
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        // Na konci vrátíme mutex, tak jak to bylo v původní funkční verzi.
        xSemaphoreGive(i2cMutex);
    }
    else {
        Serial.println("Chyba: reader_input nemohl ziskat I2C mutex!");
    }

    // Po opuštění funkce nastavíme timeout na nulu, abychom vynutili
    // okamžité překreslení hlavní obrazovky s případnými novými daty.
    timeout1 = 0;
}

/*

void reader_input(const char* display_str, char* data_to_fill) {
    char new_data[16] = "";  // Buffer pro nová data
    String buffer2;          // Pomocný buffer pro sériový vstup

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
                //buffer2 = Serial.readStringUntil('\n');  // Čtení vstupu do bufferu do znaku nového řádku
            buffer2 = SerialC.readStringUntil('\n');  // Čtení vstupu do bufferu do znaku nového řádku

            buffer2.trim();  // Odstranění bílých znaků

            if (buffer2.equals("STORNO")) {
                break;  // Přerušení při příkazu "STORNO"
            }
            else if (buffer2.equals("OK")) {
                strncpy(data_to_fill, new_data, sizeof(new_data));  // Uložení nových dat
                break;
            }
            else if (buffer2.equals("DEFAULT")) {
                // Odstranění posledního znaku
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                // Přidání vstupu do `new_data` a zobrazení na LCD
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

    char first_char_code[2] = { 0 }; // Pomocný buffer pro první znak

    /*
        char buffer2_copy[100];
        strncpy(buffer2_copy, buffer2, sizeof(buffer2_copy) - 1);
        buffer2_copy[sizeof(buffer2_copy) - 1] = '\0';  // Zajištění null-terminace

        // Nyní pracujte s `buffer2_copy` místo `buffer2`
        if (buffer2_copy[strlen(buffer2_copy) - 1] == '\n') {
            buffer2_copy[strlen(buffer2_copy) - 1] = '\0';
        }
    */
    // Odstranění koncových znaků `\n`
    if (buffer2[strlen(buffer2) - 1] == '\n') {
        buffer2[strlen(buffer2) - 1] = '\0';
    }

    logToSerial(buffer2, 0);

    String buffer2String = buffer2;
    buffer2String.trim();

    if ((millis() / 1000) >= timeout1) { _fast_clear_disp_unsafe(); }

    // Zpracování různých příkazů
    //if (strcmp(buffer2String, "SET-IP") == 0) {
    if (buffer2String.equals("SET-IP")) {
        if (useDHCP) {
            char ip_adr2[16];
            if (useETH) { strcpy(ip_adr2, ETH.localIP().toString().c_str()); }
            else { strcpy(ip_adr2, WiFi.localIP().toString().c_str()); }
          reader_input("Nastaveni IP DHCP", ip_adr2);
		  strcpy(ip_adr, ip_adr2);
		  restartNetwork = true; // Nastavíme flag pro restart sítě
          //snprintf(buffer, sizeof(buffer), "IP: %s\n", ip_adr2);
          //Serial.print(buffer);
		  //lcd2.printf("IP: %s\n", ip_adr2);
        }
        else {
          reader_input("Nastaveni IP", ip_adr);
          restartNetwork = true; // Nastavíme flag pro restart sítě
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
        restartNetwork = true; // Nastavíme flag pro restart sítě
        //} else if (strcmp(buffer2, "SET-MASK") == 0) {
    }
    else if (buffer2String.equals("SET-MASK")) {
        reader_input("Nastaveni Masky", ip_mask);
        restartNetwork = true; // Nastavíme flag pro restart sítě
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
        restartNetwork = true; // Nastavíme flag pro restart sítě
    }
    else if (buffer2String.equals("SET-STATIC")) {
        useDHCP = false;
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítě
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
        restartNetwork = true; // Nastavíme flag pro restart sítě
    }
    else if (buffer2String.equals("SET-SSID")) {
        //SSID
        reader_input("Nastavení SSID", ssid);
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítě
    }
    else if (buffer2String.equals("SET-PASSW")) {
        //SSID PASSWORD 
        reader_input("Nastavení hesla", password);
        saveNewConfigData = true;
        restartNetwork = true; // Nastavíme flag pro restart sítě
    }
    else if (buffer2String.equals("SET-APSSID")) {
        reader_input("AP SSID", ap_ssid);
        saveNewConfigData = true;
        apSettingsChanged = true; // Nastavíme flag pro restart sítě
        }
    else if (buffer2String.equals("SET-APPASS")) {
            reader_input("AP HESLO", ap_pass);
            saveNewConfigData = true;
            apSettingsChanged = true; // Nastavíme flag pro restart sítě
            }
    else if (buffer2String.equals("SET-AP-FIX")) {
                ap_ssid_use_mac = false;
                saveNewConfigData = true;
                apSettingsChanged = true; // Nastavíme flag pro restart sítě
                }
    else if (buffer2String.equals("SET-AP-MAC")) {
                    ap_ssid_use_mac = true;
                    saveNewConfigData = true;
                    apSettingsChanged = true; // Nastavíme flag pro restart sítě
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
        const char* code_body = buffer2 + 1; // Zbytek řetězce za prefixem

        // Nová logika pro zobrazení na displeji v OFFLINE režimu
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
            case 'H': // Množství
                display_line_formated(buffer2String.c_str(), 2);
                break;
                // Můžeme přidat další, pokud je potřeba
            default:
                // Neznámý kód zobrazíme na posledním řádku jako dříve
                display_line_formated(buffer2, 3);
                break;
            }
        }

        // Zpracování na základě prvního znaku
        switch (buffer2[0]) {
        case 'A':
            //strcpy(tmp, "\x06");
			first_char_code[0] = '\x06'; // Uložení prvního znaku
            break;
        case 'B':
            //strcpy(tmp, "\x05");
			first_char_code[0] = '\x05'; // Uložení prvního znaku
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
			first_char_code[0] = '\x08'; // Uložení prvního znaku
            break;
        case 'F':
            //strcpy(tmp, "\x07");
			first_char_code[0] = '\x07'; // Uložení prvního znaku
            break;
        case 'G':
            //strcpy(tmp, "\x10");
			first_char_code[0] = '\x10'; // Uložení prvního znaku
            break;
        case 'H':
            //strcpy(tmp, "\x11");
			first_char_code[0] = '\x11'; // Uložení prvního znaku
            break;
        case 'I':
            //strcpy(tmp, "\x12");
			first_char_code[0] = '\x12'; // Uložení prvního znaku
            break;
        case 'J':
            //strcpy(tmp, "\x13");
			first_char_code[0] = '\x13'; // Uložení prvního znaku
            break;
        case 'K':
            //strcpy(tmp, "\x19");
			first_char_code[0] = '\x19'; // Uložení prvního znaku
            break;
        case 'L':
            //strcpy(tmp, "\x20");
			first_char_code[0] = '\x20'; // Uložení prvního znaku
            break;
        default:
            //strcpy(tmp, "\x17");
			first_char_code[0] = '\x17'; // Uložení prvního znaku
            break;
        }

        //strcat(tmp, buffer2);

        //strcat(tmp, buffer2String.c_str());

        // Echo na LCD při offline stavu
        if (!net_on && ((long)(fifo_end - fifo_last) >= off_buffer_size)) {
            /*
            blank_line(2);
            //lcd2.printf("%s", buffer2);
            lcd2.printf("%s", buffer2String.c_str());
            */
			
			// Zobrazíme zprávu na LCD
			// již není potřeba, protože používáme display_line_formated podle prefixu 
            //display_line_formated(buffer2String.c_str(), 2);
        }

        // Přidání času k příkazům typu D, E, J atd.
        /*
        if (strchr("DEJ0123456789", buffer2[0])) {
            strcat(tmp, "~");
            //get_time(SEC_TIMER, buffer);
            get_time(buffer);
            strcat(tmp, buffer);
        }
        */
		// Přidání času k příkazům typu D, E, J atd.
        if (strchr("DEJ0123456789", buffer2[0])) {
            get_time(buffer, sizeof(buffer)); // buffer je pomocné pole, kam se uloží čas
            snprintf(tmp, sizeof(tmp), "%s%s~%s\n", first_char_code, buffer2String.c_str(), buffer);
        }
        else {
            snprintf(tmp, sizeof(tmp), "%s%s\n", first_char_code, buffer2String.c_str());
        }

        //strcat(tmp, "\n");
        
        if (net_on) {
            //client.print(tmp);
			tcp_print_safe(tmp); // Použití bezpečné funkce pro komunikaci přes TCP
        }
        else {
            //strcat(off_buffer, tmp);
            // --- Kontrola velikosti PŘED strcat (z minulé úpravy) ---
            if (strlen(off_buffer) + strlen(tmp) < sizeof(off_buffer)) {
                strcat(off_buffer, tmp); // Přidat do off_buffer, jen pokud se vejde
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

                // --- Přidána kontrola velikosti souboru PŘED zápisem ---
                bool can_write_to_file = false;
                size_t current_file_size = 0;
                size_t data_to_add_size = strlen(off_buffer); // Velikost dat, která chceme přidat

                if (data_to_add_size > 0) { // Kontrolujeme jen pokud máme co zapisovat
                    File bufferFileRead = LittleFS.open(bufferFilePath, FILE_READ); // Otevřít jen pro čtení velikosti
                    if (bufferFileRead) {
                        current_file_size = bufferFileRead.size();
                        bufferFileRead.close(); // Hned zavřít

                        // Zkontrolujeme, zda se nová data vejdou pod limit
                        if (current_file_size + data_to_add_size < MAX_BUFFER_FILE_SIZE) {
                            can_write_to_file = true;
                        }
                        else {
                            // Pokud by přidání dat překročilo limit
                            Serial.printf("CHYBA: Soubor bufferu %s je plny (%u B). Nelze pridat %u B.\n",
                                bufferFilePath, (unsigned int)current_file_size, (unsigned int)data_to_add_size);

                            // Volitelně: Zobrazit chybu i na LCD
                            /*
                            blank_line(0); // Vymažeme řádek 0
                            lcd2.setCursor(0, 0);
                            lcd2.print("CHYBA:SOUBOR PLNY!");
                            */

							display_line_formated("CHYBA: SOUBOR PLNY!", 0);
                            timeout1 = SEC_TIMER + timer2; // Necháme zprávu viditelnou
                        }
                    }
                    else {
                        Serial.printf("CHYBA: Nelze otevrit soubor %s pro kontrolu velikosti.\n", bufferFilePath);
                        // Pokud nelze zjistit velikost, raději nezapisujeme
                        can_write_to_file = false;
                    }
                }
                else {
                    // Není co zapisovat (off_buffer je prázdný)
                    Serial.println("DEBUG: off_buffer je prazdny, neni co zapisovat do souboru.");
                    can_write_to_file = false; // Není potřeba volat appendFile
                }
                // --- Konec kontroly velikosti souboru ---

                // --- Zápis do souboru POUZE pokud kontrola prošla ---
                if (can_write_to_file) {
                    appendFile(LittleFS, bufferFilePath, off_buffer); // Přidá celý obsah off_buffer na konec souboru

                    // Zobrazit "ULOZENO DO SOUBORU" na LCD
                    lcd2.setCursor(0, 0);
                    lcd2.printf("*------------------*");
                    lcd2.setCursor(0, 1);
                    lcd2.printf("|     ULOZENO      |");
                    lcd2.setCursor(0, 2);
                    lcd2.printf("|   DO SOUBORU     |"); // Opravený text
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
        save_config(); // Uložení nových dat do konfigurace
        saveNewConfigData = false; // Reset flagu po uložení

        if (apSettingsChanged) {
            restartAPIfRunning();   // ← tady přesně přijde ta trojice: softAPdisconnect + delay + startAPMode
            apSettingsChanged = false;
        }
	}

    if (restartNetwork) {
		//ESP.restart(); // Restart ESP32 pro aplikaci změn sítě
		/*
        blank_line(3); // Vymažeme řádek 3
		lcd2.printf("RESTART SITE...");
        */

		display_line_unformated("RESTART SITE...", 3);

		initializeNetwork(); // Volání funkce pro inicializaci sítě
		restartNetwork = false; // Reset flagu po restartu sítě
    }

}

void tDEMOscreen() {

    // Zkusíme si vzít klíč (mutex). Pokud je obsazený, task tady počká.
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

	//DateTime now = rtc2.isConnected() ? rtc2.getDateTime() : DateTime(SEC_TIMER); // Použijeme RTC pokud je připojeno, jinak použijeme aktuální čas z millis()  
    //String nowStr = now.timestamp(DateTime::TIMESTAMP_FULL);

    //lcd2.print(nowStr);

	//lcd2.print(rtc2.getDateTimeString()); // Zobrazíme aktuální čas z RTC
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
    else { // Offline - zobrazíme využití souboru
        File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
        if (bufferFile) {
            size_t fileSize = bufferFile.size();
            bufferFile.close();

            // Výpočet procent (bezpečný pro dělení nulou, pokud by MAX_BUFFER_FILE_SIZE byla 0)
            int usage_percent = 0;
            if (MAX_BUFFER_FILE_SIZE > 0) {
                // Použijeme 64bit int pro násobení, abychom předešli přetečení před dělením
                usage_percent = (int)(((uint64_t)fileSize * 100) / MAX_BUFFER_FILE_SIZE);
            }
            if (usage_percent > 100) usage_percent = 100; // Omezení na max 100%

            // Formátování výpisu, např. "OFF-LINE Buf:  15% "
            char statusStr[21];
            snprintf(statusStr, sizeof(statusStr), "OFF-LINE  Volno:%3d%%", 100-usage_percent);
            // Doplnění mezerami do konce řádku
            for (int i = strlen(statusStr); i < 20; ++i) statusStr[i] = ' ';
            statusStr[20] = '\0';
            lcd2.print(statusStr);

        }
        else {
            Serial.printf("tDEMOscreen: Failed to open buffer file '%s' for reading size.\n", bufferFilePath);

            // --- Pokus o zotavení: Zkontrolovat existenci a případně vytvořit ---
            if (!LittleFS.exists(bufferFilePath)) {
                // Soubor skutečně neexistuje
                Serial.println("tDEMOscreen: Buffer file does not exist. Attempting recreation...");
                lcd2.print("OFF-LINE (NEW FILE)"); // Indikace pokusu o vytvoření na LCD
                reset_buffer_file(); // Zavoláme funkci, která vytvoří adresář a prázdný soubor
            }
            else {
                // Soubor existuje, ale přesto nešel otevřít -> Pravděpodobně chyba FS
                Serial.println("tDEMOscreen: Buffer file exists but cannot be opened (FS Error?). Displaying error.");
                lcd2.print("OFF-LINE (File ERR)"); // Zobrazíme původní chybovou hlášku
            }
            // --- Konec pokusu o zotavení ---
        }
    }
    // Práce s LCD je hotová, vrátíme klíč, aby ho mohl použít někdo jiný.
    xSemaphoreGive(i2cMutex);
    }
 else {
     // Nepodařilo se získat mutex do 1 sekundy, něco je špatně.
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
    // Pokud jsme online, zkusíme zpracovat TCP komunikaci a případně odeslat data z offline bufferu.
    if (net_on) {
        TCP(); // Zpracování příchozích TCP dat od serveru.

        // Pokud zrovna neprobíhá velké odesílání souboru, zkontrolujeme, jestli nějaký soubor na odeslání nemáme.
        if (!isSendingFileBuffer) {
            File bufferFileCheck = LittleFS.open(bufferFilePath, FILE_READ);
            bool fileHasData = (bufferFileCheck && bufferFileCheck.size() > 0);
            if (bufferFileCheck) bufferFileCheck.close();

            if (fileHasData) {
                isSendingFileBuffer = true;
                send_entire_file_buffer();
                isSendingFileBuffer = false;
            }
        }
    }

    // Zobrazíme hlavní obrazovku, pokud vypršel její timeout.
    if ((millis() / 1000) >= timeout1) {
        tDEMOscreen();
    }

    // Použijeme vTaskDelay pro lepší spolupráci s FreeRTOS.
    vTaskDelay(pdMS_TO_TICKS(5));
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
    // Buffer pro zobrazení času na LCD
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
    // Hostname odvoď z Wi-Fi STA MAC ještě před DHCP
    String host = "APT1220-" + readStaMacStr(false);
    WiFi.setHostname(host.c_str());

    WiFi.begin(ssid, password);

    int retryAttempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (++retryAttempt > 4) {
            Serial.println("Failed to connect to WiFi.");
            return;
        }
    }
    // pro logy/UX: drž si aktivní MAC i s pomlčkami
    activeMAC = readStaMacStr(true); // "AA-BB-CC-DD-EE-FF"
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
        Serial.println("ETH Started");
        // je OK nastavit znovu – některé stacky to ocení
        ETH.setHostname(("APT1220E-" + readEthMacStr(false)).c_str());
        break;

    case ARDUINO_EVENT_ETH_CONNECTED:
        // This will happen when the Ethernet cable is plugged 
        Serial.println("ETH Connected");
        break;

    case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("Got an IP Address for ETH MAC: ");
        Serial.println(readEthMacStr(true));
        eth_connected = true;
        activeMAC = readEthMacStr(true);  // pro tvoje UI/telemetrii
        // ETH.setHostname(...) případně idempotentně ještě jednou
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
        Serial.println(readStaMacStr(true));
        activeMAC = readStaMacStr(true);
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

    // Používáme gmtime_r pro bezpečné konvertování epochálního času na strukturovaný čas
    gmtime_r((time_t*)&thetime, &thetm);

    // Vytvoření formátovaného řetězce s datem a časem
    /*
    sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d",
        //1900 + thetm.tm_year,    // Rok začíná od 1900
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok začíná od 1900
        thetm.tm_mon + 1,        // Měsíce jsou indexovány od 0
        thetm.tm_mday,           // Den v měsíci
        thetm.tm_hour,           // Hodiny
        thetm.tm_min,            // Minuty
        thetm.tm_sec             // Sekundy
    );
    */

    snprintf(buff, buff_size, "%04d-%02d-%02d %02d:%02d:%02d",
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok začíná od 1900
        thetm.tm_mon + 1,        // Měsíce jsou indexovány od 0
        thetm.tm_mday,           // Den v měsíci
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

// PŮVODNÍ, JEDNODUCHÁ VERZE BEZ MUTEXŮ
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
        // Zkusíme odeslat ping přes naši novou bezpečnou funkci.
        // Ta v sobě kontroluje, jestli je klient připojený a stará se o mutex.
        if (tcp_print_safe("\x01\n")) {
            // Pokud funkce vrátila true, znamená to, že odeslání proběhlo.
            setNetOn(1, 15);
        }
        else {
            // Pokud vrátila false, odeslání se nepovedlo.
            // Nejspíš nejsme připojeni.
            setNetOn(0, 16);
            // Zkusíme se tedy znovu bezpečně připojit.
            connectToServer_safe();
        }

        // Počkáme a opakujeme.
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

    // --- Kontrola stavu připojení ---
    SEC_TIMER = millis() / 1000;
    if (net_on == 1 && (long)(SEC_TIMER - last_ping) > ping_timeout) {
        if (xSemaphoreTake(tcpMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            client.stop();
            xSemaphoreGive(tcpMutex);
        }
        Serial.println("TCP Timeout: " + String(SEC_TIMER) + "/" + String(last_ping));
        setNetOn(0, 7);
    }

    // --- Reconnect, pokud jsme "on", ale socket není připojen ---
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

    // Pokud nejsou žádná data, nic neděláme a resetujeme si paměť posledního příkazu
    if (client.available() == 0) {
        g_last_tcp_command_code = 0;
        return;
    }

    // --- Čtení dat z klienta ---
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
            g_last_tcp_command_code = 0; // 'OK' není příkaz pro displej, resetujeme
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
                // Pokud předchozí příkaz nebyl také rámeček, vykreslíme ho celý.
                if (g_last_tcp_command_code != 0xE) {
                    lcd2.setCursor(0, 0); lcd2.printf("*------------------*");
                    lcd2.setCursor(0, 2); lcd2.printf("*------------------*");
                }
                // Ale text na prostředním řádku aktualizujeme VŽDY.
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

        // Na konci zpracování si zapamatujeme kód aktuálního příkazu pro příští smyčku.
        g_last_tcp_command_code = buffer[0];
    }
}


void display_line_formated(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se děje uvnitř jednoho zamčení
        _blank_line_unsafe(linenumber);
        //lcd2.setCursor(0, linenumber);
        lcd2.printf("%s",message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu
    }
}

void display_line_unformated(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se děje uvnitř jednoho zamčení
        _blank_line_unsafe(linenumber);
        //lcd2.setCursor(0, linenumber);
        lcd2.printf(message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu
    }
}

void display_temporary_status(const char* message, const int linenumber) {
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
        // Všechno se děje uvnitř jednoho zamčení
        _blank_line_unsafe(linenumber);
        lcd2.setCursor(0, linenumber);
        lcd2.printf(message);

        xSemaphoreGive(i2cMutex); // Odemknu hned, jak dopíšu

        vTaskDelay(pdMS_TO_TICKS(2000)); // Počkám

        // Znovu si zamknu, abych po sobě uklidil
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

    // Zobrazení stavu přenosu na LCD
    lcd2.setCursor(0, 0);
    lcd2.printf("*------------------*");
    lcd2.setCursor(0, 1);
    lcd2.printf("|  PRENASIM DATA   |");
    lcd2.setCursor(0, 2);
    lcd2.printf("| DO PC - CEKEJTE  |");
    lcd2.setCursor(0, 3);
    lcd2.printf("*------------------*");

    // Příprava pro odeslání dat
    if (!client.connected() || client.write("\x06A-100\n") != 7) {
        //net_on = 0;
        setNetOn(0, 3);
        return;
    }
    client.flush();
    delay(1000);

    timeout2 = SEC_TIMER + 2;  // Timeout na odpověď
    tmp[0] = '\0';

    // Čekání na odpověď od serveru
    while (((long)(SEC_TIMER - timeout2) <= 0) && (tmp[3] != '?')) {
        tmp[0] = '\0';
        server_read(tmp);
    }

    if (tmp[3] == '?') {
        timeout2 = SEC_TIMER + 30;  // Zakázání přepisu displeje
        last_ping = SEC_TIMER;
        long l = fifo_first;

        // Odeslání dat, dokud je síť připojena a existují data
        while ((fifo_first + 3 < fifo_last) && net_on) {
            memset(off_buffer, 0x00, off_buffer_size);
            // Simulace načtení dat do `off_buffer`
            // xmem2root(off_buffer, fifo_first, off_buffer_size);

            tmp_send_data[0] = '\0';
            i = k = 0;
            set_first = 0;

            // Výběr a formátování dat pro odeslání
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

            // Kontrola připojení a odeslání dat
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
                            tmp[strlen(tmp) - 1] = '\0';  // Odstranění znaku '\n'
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

        // Uvolnění paměti FIFO
        if ((long)(fifo_first + 3 >= fifo_last)) {
            fifo_last = fifo_start;
            fifo_first = fifo_start;
        }
    }
    else {
        // Reset spojení při neúspěchu
        client.stop();
        net_on = 0;
        delay(200);
    }
}

int server_read(char* buffer) {
    int len = 0;
    char tmp[2] = { 0 };  // Vyrovnávací paměť pro jednotlivé znaky
    unsigned long timeout = SEC_TIMER + 2;
    bool ok = false;
    *buffer = '\0';  // Inicializace bufferu na prázdný řetězec

    while (!ok && ((long)(SEC_TIMER - timeout) <= 0)) {
        // Čekejte na data
        if (client.available()) {
            len = client.readBytes(tmp, 1);  // Přečtěte jeden znak
            tmp[1] = '\0';  // Ukončovací znak pro správné spojení s řetězcem
            if (len > 0) {
                strcat(buffer, tmp);
                // Pokud narazíme na nový řádek, ukončujeme čtení
                if (tmp[0] == '\n') {
                    ok = true;
                }
            }
        }

        // Pokud žádná data nepřichází, ukončíme smyčku
        if (strlen(buffer) == 0) {
            ok = true;
        }
    }

    return strlen(buffer);  // Vrací délku načtených dat
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
    String ssidAP;
    if (ap_ssid_use_mac) {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);        // nezávislé na stavu driveru
        ssidAP = String(ap_ssid) + "_" + macToStr(mac, false);  // např. APT1220_AP_7E2233445566
    }
    else {
        ssidAP = String(ap_ssid);                      // čistě fixní SSID
    }

    if (WiFi.softAP(ssidAP.c_str(), ap_pass)) {
        Serial.println("AP mode started. SSID: " + ssidAP);
        Serial.println("IP address: " + WiFi.softAPIP().toString());

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
/*
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
*/
void checkForUpdates() {
    Serial.println("Kontrola aktualizací...");

    // Stažení souboru s verzí z serveru
    HTTPClient http;
    // Explicitně začínáme HTTP spojení se specifikací portu a timeoutu
    http.begin(versionURL);
    http.setTimeout(10000); // Nastavení delšího timeoutu (10 sekund)

    // Přidání hlaviček pro simulaci běžného browseru
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

            // Vytvoříme nový HTTPClient pro stažení aktualizace
            HTTPClient updateClient;

            // Přidání hlaviček pro simulaci běžného browseru i pro update request
            updateClient.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
            updateClient.addHeader("Accept", "*/*");
            updateClient.addHeader("Accept-Language", "cs,en-US;q=0.7,en;q=0.3");
            updateClient.addHeader("Connection", "keep-alive");
            updateClient.setTimeout(30000); // Delší timeout pro stahování firmware (30 sekund)

            Serial.print("Připojuji se k: ");
            Serial.println(firmwareURL);

            // Zkusíme nejprve přímou metodu s použitím WiFiClient místo HTTPClient
            WiFiClient client;

            Serial.println("Pokouším se o aktualizaci přes WiFiClient...");
            t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

            // Pokud první metoda selže, zkusíme původní metodu s HTTPClient
            if (ret == HTTP_UPDATE_FAILED) {
                Serial.println("První metoda selhala, zkouším alternativní metodu...");
                ret = httpUpdate.update(updateClient, String(firmwareURL));
            }

            switch (ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("Aktualizace selhala: %d - %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                // Dodatečné debugování
                Serial.print("URL firmware: ");
                Serial.println(firmwareURL);
                break;
            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("Není k dispozici žádná nová aktualizace.");
                break;
            case HTTP_UPDATE_OK:
                Serial.println("Aktualizace proběhla úspěšně.");
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



// Funkce, která běží jako samostatné vlákno
void otaUpdateTask(void* parameter) {
    Serial.println("OTA task started");

    while (true) {
        // Kontrola dostupnosti aktualizace
        if (!otaInProgress) {
            checkForUpdatesBackground();
        }

        // Pokud je aktualizace dostupná, proveď ji podle schématu URL
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
                    otaUpdateAvailable = false; // Resetuj příznak, aby se to neopakovalo hned
                    // otaInProgress zůstává false
                }
            }
            else {
                Serial.println("CHYBA: firmwareURL je NULL nebo příliš krátká!");
                otaUpdateAvailable = false; // Resetuj příznak
            }
        }

        // Počkej na další kontrolu.
        // POZNÁMKA: 60000 ms je 1 MINUTA, ne 1 hodina. Pro 1 hodinu použijte 3600000 ms.
        vTaskDelay(60000 / portTICK_PERIOD_MS); // Aktuálně 1 minuta
    }
}

// Kontroluje, zda je dostupná nová verze
void checkForUpdatesBackground() {
    Serial.println("Kontrola aktualizací na pozadí...");

    // Stažení souboru s verzí z serveru
    HTTPClient http;
    http.begin(versionURL);
    http.setTimeout(10000);

    // Přidání hlaviček pro simulaci běžného browseru
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

    // Vytvoření indikace na displeji (pokud je potřeba)
    // displayUpdateStatus("Aktualizace...");

    // Použijeme přímou metodu s WiFiClient, která fungovala
    WiFiClient client;

    t_httpUpdate_return ret = httpUpdate.update(client, firmwareURL);

    switch (ret) {
    case HTTP_UPDATE_FAILED:
        Serial.printf("Aktualizace selhala: %d - %s\n",
            httpUpdate.getLastError(),
            httpUpdate.getLastErrorString().c_str());
        // Resetujeme příznaky pro případný další pokus
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_NO_UPDATES:
        Serial.println("Není k dispozici žádná nová aktualizace.");
        otaInProgress = false;
        otaUpdateAvailable = false;
        break;

    case HTTP_UPDATE_OK:
        Serial.println("Aktualizace proběhla úspěšně, restartuju...");
        // Při úspěšné aktualizaci dojde k restartu ESP32
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
        Serial.println("Není k dispozici žádná nová aktualizace (dle httpUpdate).");
        otaInProgress = false;
        break;

    case HTTP_UPDATE_OK:
        Serial.println("Aktualizace proběhla úspěšně, restartuju...");
        // ESP32 will restart automatically.
        break;
    }
}

// Funkce pro inicializaci OTA úlohy - volejte v setup()
void setupOTA() {
    // Nastavíme aktuální verzi
    //currentVersion = "1.0.0.0";  // Pro testování, v reálu by se četla z konstanty

    // Vytvoříme úlohu na jádře 0 (hlavní aplikace běží typicky na jádře 1)
    /* 
    xTaskCreatePinnedToCore(
        otaUpdateTask,      // Funkce, která implementuje úlohu
        "OTATask",          // Název úlohy
        8192,               // Velikost zásobníku (v slovech)
        NULL,               // Parametr úlohy
        1,                  // Priorita úlohy (nižší číslo = nižší priorita)
        &otaTaskHandle,     // Handle úlohy
        0                   // Jádro CPU, na kterém má úloha běžet (0 nebo 1)
    );
    */

    xTaskCreatePinnedToCore(otaUpdateTask, "OTATask", STACK_WORDS(4096), NULL, 1, &otaTaskHandle, 0);

}

//************************************************************************
// Nová funkce pro odeslání bufferu ze souboru aptbuffer.txt
//************************************************************************
void send_file_buffer() {
    // 1. Zkontrolujeme, zda jsme online a připojeni
    if (!net_on || !client.connected()) {
        return; // Nejsme online nebo připojeni, nemá smysl pokračovat
    }

    // 2. Zkusíme otevřít soubor bufferu pro čtení
    File bufferFile = LittleFS.open(bufferFilePath, FILE_READ);
    if (!bufferFile || bufferFile.size() == 0) {
        if (bufferFile) bufferFile.close();
        return; // Soubor neexistuje nebo je prázdný
    }

    // 3. Přečteme první řádek (zprávu) ze souboru
    String lineToSend = "";
    if (bufferFile.available()) {
        lineToSend = bufferFile.readStringUntil('\n');
    }

    if (lineToSend.length() == 0) {
        bufferFile.close(); // Nic jsme nepřečetli (možná chyba nebo jen prázdný řádek na začátku)
        // Můžeme zkusit soubor promazat, pokud obsahuje jen bílé znaky? Prozatím ne.
        return;
    }

    // Přidáme zpět znak nového řádku, pokud ho readStringUntil odstranil a server ho vyžaduje
    lineToSend += "\n";

    Serial.print("Attempting to send from file buffer: ");
    Serial.print(lineToSend);

    // 4. Pokusíme se odeslat první řádek
    bool send_success = false;
    if (client.print(lineToSend)) {
        client.flush(); // Počkáme na odeslání
        Serial.println(" - Sent, waiting for confirmation...");

        // 5. Čekáme na potvrzení od serveru (podobně jako v původním send_buffer)
        //    Použijeme jednoduchý mechanismus čtení s timeoutem
        unsigned long confirm_timeout = millis() + 3000; // Timeout 3 sekundy na odpověď
        String response = "";
        while (millis() < confirm_timeout) {
            if (client.available()) {
                response = client.readStringUntil('\n');
                response.trim(); // Odstraníme bílé znaky
                Serial.print(" - Received response: ");
                Serial.println(response);
                // Zde předpokládáme, že server pošle něco specifického pro potvrzení
                // Původní kód čekal na odpověď začínající na 0x7E (~)
                // Upravte podmínku podle vaší serverové logiky
                if (response.length() > 0 && response.startsWith("~")) { // Příklad: odpověď začíná '~'
                    send_success = true;
                    last_ping = SEC_TIMER; // Aktualizujeme last_ping
                    break;
                }
            }
            delay(10); // Krátká pauza, aby neběžela smyčka naplno
        }
        if (!send_success) {
            Serial.println(" - Confirmation failed or timed out.");
        }

    }
    else {
        Serial.println(" - Client print failed.");
        setNetOn(0, 20); // Pravděpodobně problém se spojením
        bufferFile.close();
        return;
    }

    // 6. Pokud odeslání a potvrzení proběhlo úspěšně, přepíšeme soubor bez odeslaného řádku
    if (send_success) {
        Serial.println(" - Send successful. Removing line from buffer file.");

        // Otevřeme dočasný soubor pro zápis
        File tempFile = LittleFS.open("/apt1220/aptbuffer.tmp", FILE_WRITE);
        if (!tempFile) {
            Serial.println("Error: Failed to open temporary buffer file for writing!");
            bufferFile.close();
            return;
        }

        // Zkopírujeme zbytek původního souboru (od druhého řádku dál) do dočasného
        char copyBuf[128]; // Buffer pro kopírování
        while (bufferFile.available()) {
            int bytesRead = bufferFile.readBytes(copyBuf, sizeof(copyBuf));
            if (bytesRead > 0) {
                tempFile.write((uint8_t*)copyBuf, bytesRead);
            }
        }

        // Zavřeme oba soubory
        bufferFile.close();
        tempFile.close();

        // Smažeme původní soubor a přejmenujeme dočasný
        if (LittleFS.remove(bufferFilePath)) {
            if (LittleFS.rename("/apt1220/aptbuffer.tmp", bufferFilePath)) {
                Serial.println(" - Buffer file updated successfully.");
            }
            else {
                Serial.println("Error: Failed to rename temporary buffer file!");
                // Pokusíme se smazat i dočasný soubor, pokud přejmenování selhalo
                LittleFS.remove("/apt1220/aptbuffer.tmp");
            }
        }
        else {
            Serial.println("Error: Failed to remove original buffer file!");
            // Pokusíme se smazat i dočasný soubor
            LittleFS.remove("/apt1220/aptbuffer.tmp");
        }

    }
    else {
        // Pokud odeslání selhalo, soubor neměníme, zavřeme ho a zkusíme to znovu později
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
// Verze 4: Opraveno nebezpečné volání isspace().
//************************************************************************
void send_entire_file_buffer() {
    // 1. Kontrola sítě - pokud nejsme online, nemá smysl pokračovat.
    if (!net_on) return;

    // Rychlá kontrola připojení klienta pomocí mutexu
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
        return; // Soubor neexistuje nebo je prázdný, není co dělat.
    }

    Serial.println(">>> Zahajuji odesilani offline bufferu ze souboru... <<<");

    // Připravíme si dočasný soubor, kam budeme ukládat řádky, které se nepodařilo odeslat.
    String tempFileName = "/apt1220/aptbuffer.tmp";
    File tempFile = LittleFS.open(tempFileName, FILE_WRITE);
    if (!tempFile) {
        Serial.println("CHYBA: Nelze otevrit docasny soubor pro zapis!");
        bufferFile.close();
        return;
    }

    bool error_occurred = false; // Flag, který nám řekne, jestli se má původní soubor smazat, nebo nahradit dočasným.

    // Indikace na LCD displeji
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        _fast_clear_disp_unsafe();
        lcd2.setCursor(0, 0); lcd2.print(F("*------------------*"));
        lcd2.setCursor(0, 1); lcd2.print(F("|  ODESILAM BUFFER |"));
        lcd2.setCursor(0, 2); lcd2.print(F("|   ZE SOUBORU...  |"));
        lcd2.setCursor(0, 3); lcd2.print(F("*------------------*"));
        xSemaphoreGive(i2cMutex);
    }

    // 3. Smyčka pro čtení a odeslání všech řádků ze souboru
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
                    // Přetypujeme responseBuffer[n-1] na (unsigned char), abychom předešli pádu.
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