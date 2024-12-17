#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_wifi.h>
#include "string.h"
#include <ETH.h>
#include <WebServer.h>
#include "time.h"
#include <Arduino.h>
#include <stdarg.h>  // Knihovna pro variabilnÌ argumenty
#include <stdio.h>   // Knihovna pro vsnprintf
#include <LCDI2C_Multilingual.h>

#include <Wire.h>
#include <I2C_RTC.h>
#include <RTClib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

///vypnutÌ pouûitÌ EEPROM a EspConfigLib
//#include "uEEPROMLib.h"
//#include "uEspConfigLib.h"

#include <Preferences.h>

#define SPIFFS LittleFS
#define FORMAT_LITTLEFS_IF_FAILED true

// Caution: It need to be a global variable or a global pointer.
// if FS is a 'setup' variable it will lead to crashes
///vypnutÌ pouûitÌ EEPROM a EspConfigLib
//uEspConfigLibFSInterface* configFsC;
//uEspConfigLib* config;

// uEEPROMLib eeprom;
///vypnutÌ pouûitÌ EEPROM a EspConfigLib
//uEEPROMLib eeprom(0x50);

static DS1307 rtc2;

LCDI2C_Latin_Symbols lcd2(0x27, 20, 4);    // I2C address = 0x27; LCD = Surenoo SLC1602A (European)
WiFiClient client;
WebServer server(80);

// Inicializace sÈriov˝ch port˘
HardwareSerial SerialC(1); // Pro SerialC
HardwareSerial SerialD(2); // Pro SerialD

#define TCPCONFIG 1
#define VERZE_PRACANT  "3.3.25"
#define ALOC_MEM 64000
#define SERB_USEPORTD
#define timeout_reset 86400;  // jak casto resetovat

// commands of terminal
#define C_ERROR_DATA     $00;  // nekorektni data
#define C_ECHO           $01;  // Poöli odezvu        PC <-> TERMINAL
#define C_SET_TIME       $02;  // Nastav Ëas          PC  -> TERMINAL
#define C_GET_TIME       $03;  // Poöli Ëas           PC <-> TERMINAL
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
static unsigned long last_ping;      // DISPLAY NAME timeout
static int __attribute__((section(".noinit")))  ping_timeout;		// DISPLAY NAME timeout
static int __attribute__((section(".noinit")))  key_maker = 0;    // 0 - jedna se o pracanta
// 1 - jedna se o zamek
static long __attribute__((section(".noinit"))) timeout1;
static long __attribute__((section(".noinit"))) timeout2;

static int __attribute__((section(".noinit")))  rfid_reader_c;  // 0 - normalni ËteËka
static int __attribute__((section(".noinit")))  id12_c;         // 0 - normalni ËteËka
// 1 - rfid modul
static int __attribute__((section(".noinit")))  rfid_reader_d;  // 0 - normalni ËteËka
static int __attribute__((section(".noinit")))  id12_d;         // 0 - normalni ËteËka
// 1 - rfid modul

static long __attribute__((section(".noinit"))) timer1;		      // DISPLAY NAME timeout
static long __attribute__((section(".noinit"))) timer2;		      // CONFIRM TIMEOUT
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

static unsigned long __attribute__((section(".noinit"))) fifo_start;
static unsigned long __attribute__((section(".noinit"))) fifo_end;
static unsigned long __attribute__((section(".noinit"))) fifo_first;
static unsigned long __attribute__((section(".noinit"))) fifo_last;

static unsigned long physaddr;

char __attribute__((section(".noinit"))) off_buffer[251];
#define off_buffer_size 250    // OFFLINE BUFFER SIZE - maximu
static unsigned long SEC_TIMER;

static bool eth_connected = false;
static int  runCounter = 0;

// NastavenÌ WiFi a TCP
//const char* ssid = "AGERIT_AC 2GHz";
//const char* password = "AGERITagerit512";
static char* ssid = "blackies";
static char* password = "Blackies105111";
const char* serverIP = "192.168.225.221";
const int serverPort = 54321;

const bool vSerialDebug = true;
const int  vLogLevel = 0;

static bool useWifi = false;
static bool useETH = true;

static bool useDHCP = true;

static int efect = 0;

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

TaskHandle_t tSEC_TIMER;
TaskHandle_t tLAST_PING;
TaskHandle_t tTCP;
TaskHandle_t tDEMO;

char c_string2[16] = { 0 };

///vypnutÌ pouûitÌ EEPROM a EspConfigLib
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
    writeFile(LittleFS, "/apt1220/aptbuffer.txt", ""); // Create a hello1.txt file with the content "Hello1"
}

void setup() {
    Serial.begin(115200);

    delay(500);

    Wire.begin();
    delay(100);

    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        Serial.println("LittleFS Mount Failed");
        return;
    }

    if (!LittleFS.exists("/apt1220/aptbuffer.txt")) {
        reset_buffer_file();
    }

    ///vypnutÌ pouûitÌ EEPROM a EspConfigLib
    //setup_inifile();

    ///vypnutÌ pouûitÌ EEPROM a EspConfigLib
    //eeprom.eeprom_read(240, &loaded_default);

    delay(100);

    //if (loaded_default!=1) { set_default(); save_config();}
    boolean confiLoaded = load_config();
    delay(100);

    //set_default();

    lcd2.init();
    //lcd2.clear();                             // Initialize the LCD
    lcd2.backlight();                         // Turn on the LCD backlight
    lcd2.createChar(0, customChar);
    lcd2.clear();
    delay(200);
    lcd2.printf("       eMISTR       ");
    delay(200);

    rtc2.setHourMode(CLOCK_H24);
    rtc2.startClock();

    xTaskCreatePinnedToCore(
        tSEC_TIMERcode, /* Function to implement the task */
        "tSEC_TIMER", /* Name of the task */
        4096,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &tSEC_TIMER,  /* Task handle. */
        1); /* Core where the task should run */
    delay(200);


    SerialC.begin(9600, SERIAL_8N1, 36, 14);  // Nastavte spr·vnÈ piny pro SerialC
    //SerialC.begin(9600, SERIAL_8N1, 36, 35);  // Nastavte spr·vnÈ piny pro SerialC
    SerialD.begin(9600, SERIAL_8N1, 32, 33);   // Nastavte spr·vnÈ piny pro SerialD  

    // Add a handler for network events. This is misnamed "WiFi" because the ESP32 is historically WiFi only,
    // but in our case, this will react to Ethernet events.
    Serial.print("Registering event handler for ETH events...");
    WiFi.onEvent(WiFiEvent);

    delay(200);

    if (useWifi) {
        connectToWiFi();
    }

    // Starth Ethernet (this does NOT start WiFi at the same time)
    if (useETH) {
        Serial.print("Starting ETH interface...");

        ETH.begin();
    }

    connectToServer();

    xTaskCreatePinnedToCore(
        tLAST_PINGcode, /* Function to implement the task */
        "tLAST_PING", /* Name of the task */
        4096,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &tLAST_PING,  /* Task handle. */
        1); /* Core where the task should run */
    delay(100);

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

    timeout1 = 999999999;
    timer_reset = SEC_TIMER + 86400;  // DennÌ reset      
}

void fast_clear_disp() {
    lcd2.clear();
    lcd2.home();
};

void blank_line(int line) {
    lcd2.setCursor(0, line);                       // UmÌstÌ kurzor na zaË·tek ¯·dku
    lcd2.printf("                    ");       // VypÌöe pr·zdn˝ ¯etÏzec o 20 mezer·ch
    lcd2.setCursor(0, line);                       // UmÌstÌ kurzor zpÏt na zaË·tek ¯·dku
}

boolean load_config() {
    //naËtenÌ konfigurace z Preferences
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

    ping_timeout = 4;
    preferences.end();
    return true;
}

boolean save_config() {
    //uloûenÌ konfigurace do Preferences
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
    ///vypnutÌ pouûitÌ EEPROM a EspConfigLib
    //eeprom.eeprom_write(240, &loaded_default);
}

void init_lcd() {
    lcd2.init();
    lcd2.clear();                             // Initialize the LCD
    lcd2.backlight();                         // Turn on the LCD backlight
    lcd2.noAutoscroll();
}

void menu_dump() {
    char long_str[20];  // Vyrovn·vacÌ pamÏù pro p¯eveden· ËÌsla

    fast_clear_disp();  // VyËiötÏnÌ displeje

    // ZobrazenÌ hodnoty fifo_start
    lcd2.setCursor(0, 0);
    lcd2.printf("START:");
    ltoa(fifo_start, long_str, 10);  // P¯evede `fifo_start` na ¯etÏzec
    lcd2.printf(long_str);

    // ZobrazenÌ hodnoty fifo_end
    lcd2.setCursor(0, 1);
    lcd2.printf("END:");
    ltoa(fifo_end, long_str, 10);
    lcd2.printf(long_str);

    // ZobrazenÌ hodnoty fifo_first
    lcd2.setCursor(0, 2);
    lcd2.printf("FIRST:");
    ltoa(fifo_first, long_str, 10);
    lcd2.printf(long_str);

    // ZobrazenÌ hodnoty fifo_last
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
    // Zav¯ete existujÌcÌ TCP p¯ipojenÌ
    if (client.connected()) {
        client.stop();
        Serial.println("TCP client disconnected");
    }

    // Restart WiFi p¯ipojenÌ
    WiFi.disconnect(true); // OdpojÌ a zapomene p¯ipojenÌ
    delay(1000);           // PoËkejte na odpojenÌ

    // NastavenÌ statickÈ IP (pokud je poûadov·na statick· IP)
    if (!WiFi.config(IPAddress(ip_adr), IPAddress(ip_gate), IPAddress(ip_mask))) {
        Serial.println("STA Failed to configure");
    }

    Serial.println(useWifi ? "WIFI connect" : "ETH connect");

    // P¯ipojte se znovu k WiFi sÌti
    if (useWifi) {
        connectToWiFi();
    }

    // Starth Ethernet (this does NOT start WiFi at the same time)
    if (useETH) {
        Serial.print("Starting ETH interface...");
        ETH.begin();
    }

    // Inicializace FIFO promÏnn˝ch
      //fifo_start = malloc(ALOC_MEM);
      //fifo_end =fifo_start+ALOC_MEM;

    fifo_first = fifo_start;
    fifo_last = fifo_start;

    // OvÏ¯enÌ a nastavenÌ ËasovaË˘
    if (timer1 > 60 || timer1 < 1) timer1 = 10;
    if (timer2 > 60 || timer2 < 1) timer2 = 5;

    // Otev¯ete novÈ TCP p¯ipojenÌ
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
    char new_data[16] = "";  // Buffer pro nov· data
    String buffer2;          // Pomocn˝ buffer pro sÈriov˝ vstup

    fast_clear_disp();       // Vymaz·nÌ LCD displeje

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
                //buffer2 = Serial.readStringUntil('\n');  // »tenÌ vstupu do bufferu do znaku novÈho ¯·dku
            buffer2 = SerialC.readStringUntil('\n');  // »tenÌ vstupu do bufferu do znaku novÈho ¯·dku

            buffer2.trim();  // OdstranÏnÌ bÌl˝ch znak˘

            if (buffer2.equals("STORNO")) {
                break;  // P¯eruöenÌ p¯i p¯Ìkazu "STORNO"
            }
            else if (buffer2.equals("OK")) {
                strncpy(data_to_fill, new_data, sizeof(new_data));  // UloûenÌ nov˝ch dat
                break;
            }
            else if (buffer2.equals("DEFAULT")) {
                // OdstranÏnÌ poslednÌho znaku
                if (strlen(new_data) > 0) {
                    new_data[strlen(new_data) - 1] = '\0';
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                    lcd2.printf(" ");
                    lcd2.setCursor(strlen(new_data) + 4, 2);
                }
            }
            else {
                // P¯id·nÌ vstupu do `new_data` a zobrazenÌ na LCD
                strncat(new_data, buffer2.c_str(), sizeof(new_data) - strlen(new_data) - 1);
                lcd2.printf("%s", buffer2.c_str());
            }
        }
    }
}

void serial(char* buffer2, int port) {
    /* void serial(const char* buffer2, int port) {  */
    char buffer[100] = { 0 };
    char tmp[100] = { 0 };
    int counter1;
    char tmp2[100] = { 0 };
    /*
        char buffer2_copy[100];
        strncpy(buffer2_copy, buffer2, sizeof(buffer2_copy) - 1);
        buffer2_copy[sizeof(buffer2_copy) - 1] = '\0';  // ZajiötÏnÌ null-terminace

        // NynÌ pracujte s `buffer2_copy` mÌsto `buffer2`
        if (buffer2_copy[strlen(buffer2_copy) - 1] == '\n') {
            buffer2_copy[strlen(buffer2_copy) - 1] = '\0';
        }
    */
    // OdstranÏnÌ koncov˝ch znak˘ `\n`
    if (buffer2[strlen(buffer2) - 1] == '\n') {
        buffer2[strlen(buffer2) - 1] = '\0';
    }

    logToSerial(buffer2, 0);

    String buffer2String = buffer2;
    buffer2String.trim();

    // Zpracov·nÌ r˘zn˝ch p¯Ìkaz˘
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
        reader_input("NastavenÌ SSID", ssid);
    }
    else if (buffer2String.equals("SET-PASSW")) {
        //SSID PASSWORD 
        reader_input("NastavenÌ hesla", password);                
    }
    else if (buffer2String.equals("SHOW-NET")) {
        //SSID PASSWORD 
        reader_input("NastavenÌ hesla", password);
    }
    else if (buffer2String.equals("SHOW-WIFI")) {
        //SSID PASSWORD 
        reader_input("NastavenÌ hesla", password);
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

        // Zpracov·nÌ na z·kladÏ prvnÌho znaku
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
        default:
            strcpy(tmp, "\x17");
            break;
        }

        // Echo na LCD p¯i offline stavu
        if (!net_on && ((long)(fifo_end - fifo_last) >= off_buffer_size)) {
            blank_line(2);
            lcd2.printf("%s", buffer2);
        }

        // P¯id·nÌ Ëasu k p¯Ìkaz˘m typu D, E, J atd.
        if (strchr("DEJ0123456789", buffer2[0])) {
            strcat(tmp, "~");
            get_time(SEC_TIMER, buffer);
            strcat(tmp, buffer);
        }

        strcat(tmp, "\n");
        if (net_on) {
            client.print(tmp);
        }
        else {
            strcat(off_buffer, tmp);
        }

        if (!net_on && strchr("D0123456789", buffer2[0]) &&
            ((long)(fifo_end - fifo_last) >= off_buffer_size)) {
            strncpy((char*)fifo_last, off_buffer, strlen(off_buffer));
            fifo_last += strlen(off_buffer) + 1;

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
            memset(off_buffer, 0x00, off_buffer_size);
        }
    }
}

void loop_bbbb() {
    char buffer2[100] = { 0 };

    // ZobrazenÌ informacÌ na LCD
    if (key_maker == 0) {
        lcd2.setCursor(0, 0);
        //lcd2.printf("   eMISTR ESP32    ");
        String tmpConnType = " eMISTR ESP32   ";
        tmpConnType += useWifi ? "WIFI" : " ETH";
        lcd2.printf(tmpConnType.c_str());
    }
    if (key_maker == 1) {
        lcd2.setCursor(0, 0);
        lcd2.printf(" eMISTR ESP32 DVERE ");
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
    print_time(rtc2.getEpoch(), buffer2);

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
    //hlavnÌ obrazovka
    // Buffer pro zobrazenÌ Ëasu na LCD
    if (net_on) {
        TCP();
        if ((long)(fifo_last - fifo_start) > 0) {
            send_buffer();
        }
    }

    loop_bbbb();

    // »tenÌ z r˘zn˝ch sÈriov˝ch port˘ (SerialC, SerialD)
    if (SerialC.available()) {
        char buffer2[100] = { 0 };  // Deklarace lok·lnÌ promÏnnÈ pro ËtenÌ ze SerialC
        SerialC.readBytesUntil('\n', buffer2, sizeof(buffer2));
        logToSerial(buffer2, 1);
        if (strlen(buffer2) > 0) {
            serial(buffer2, 1);
        }
    }

    if (SerialD.available()) {
        char buffer2[100] = { 0 };  // Deklarace lok·lnÌ promÏnnÈ pro ËtenÌ ze SerialD
        SerialD.readBytesUntil('\n', buffer2, sizeof(buffer2));
        logToSerial(buffer2, 1);
        if (strlen(buffer2) > 0) {
            serial(buffer2, 2);
        }
    }

    // Automatick˝ reset kaûd˝ch 24 hodin
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
    delay(100);
}

void tDEMOcode(void* parameter) {
    for (;;) {
        tDEMOrun();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void tDEMOrun() {
    //hlavnÌ obrazovka
    // Buffer pro zobrazenÌ Ëasu na LCD
    char buffer2[100] = { 0 };

    // ZobrazenÌ informacÌ na LCD
    if (key_maker == 0) {
        lcd2.setCursor(0, 0);
        lcd2.printf("   eMISTR ESP32    ");
    }
    if (key_maker == 1) {
        lcd2.setCursor(0, 0);
        lcd2.printf(" eMISTR ESP32 DVERE ");
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
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
}

void connectToServer() {
    if (client.connect(serverIP, serverPort)) {
        //net_on = 1;
        setNetOn(1, 12);

        SEC_TIMER = rtc2.getEpoch();
        //SEC_TIMER = rtc2.now().unixtime();

        last_ping = SEC_TIMER;

        Serial.println("Connected to server");
    }
    else {
        setNetOn(0, 13);
        Serial.println("Connection to server failed");
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
        ETH.setHostname("esp32-ethernet");
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

    // PouûÌv·me gmtime_r pro bezpeËnÈ konvertov·nÌ epoch·lnÌho Ëasu na strukturovan˝ Ëas
    gmtime_r((time_t*)&thetime, &thetm);

    // Vytvo¯enÌ form·tovanÈho ¯etÏzce s datem a Ëasem
    sprintf(buff, "%04d-%02d-%02d %02d:%02d:%02d",
        //1900 + thetm.tm_year,    // Rok zaËÌn· od 1900
        RTC_YEAR_OFFSET + thetm.tm_year,    // Rok zaËÌn· od 1900
        thetm.tm_mon + 1,        // MÏsÌce jsou indexov·ny od 0
        thetm.tm_mday,           // Den v mÏsÌci
        thetm.tm_hour,           // Hodiny
        thetm.tm_min,            // Minuty
        thetm.tm_sec             // Sekundy
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
        // Odeöleme p¯Ìkaz ping ve form·tu "\01\n" (bin·rnÌ 0x01 n·sledovan˝ '\n')
        if (client.connected()) {
            if (client.write("\x01\n", 2) != 2) {
                setNetOn(0, 15);
            }
            else
            {
                setNetOn(1, 15);
            }
            client.flush();  // Vypr·zdnÌ v˝stupnÌ buffer
        }
        else {
            setNetOn(0, 16); //net_on = 0;  // Pokud nenÌ p¯ipojen, nastavÌ `net_on` na 0
            connectToServer();
        }
        vTaskDelay(1900 / portTICK_PERIOD_MS);
    }
}

void tSEC_TIMERcode(void* parameter) {
    for (;;) {

        SEC_TIMER = rtc2.getEpoch();
        //SEC_TIMER = rtc2.now().unixtime();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TCP() {
    char buffer[80] = { 0 };     // Buffer pro p¯ijat· data
    char buffer2[80] = { 0 };    // DalöÌ pomocn˝ buffer
    int counter1;
    struct tm thetm;

    // Kontrola stavu p¯ipojenÌ
    if (net_on == 1 && ((long)(SEC_TIMER - last_ping) > ping_timeout)) {
        client.stop();  // Zav¯Ìt spojenÌ, pokud doölo k timeoutu
        //net_on = 0;
        Serial.println("values: " + String(SEC_TIMER) + "/" + String(last_ping) + "/" + String(ping_timeout) + "/");
        setNetOn(0, 7);
        //delay(200);
        delay(5);
    }
    /*
        if (!client.connected() && !inSetup) {
              if (WiFi.status() == WL_CONNECTED) {
                if (client.connect(serverIP, serverPort)) {  // ZmÏÚte na IP serveru a port
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
        // Restart spojenÌ, pokud nenÌ p¯ipojenÌ aktivnÌ
        client.stop();
        //delay(200);
        delay(5);
        if (client.connect(serverIP, serverPort)) {  // ZmÏÚte na IP serveru a port

            SEC_TIMER = rtc2.getEpoch();
            //SEC_TIMER = rtc2.now().unixtime();

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
        // »tenÌ dat z klienta
        while (client.available() > 0) {
            int bytes_read = client.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            buffer[bytes_read] = '\0';  // UkonËovacÌ znak pro ¯etÏzec
            SEC_TIMER = rtc2.getEpoch();
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
            // OvÏ¯enÌ p¯ÌchozÌho p¯Ìkazu
            switch (buffer[0]) {
            case 2:  // NastavenÌ Ëasu
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
                thetm.tm_year = atoi(buffer + 2) - 1900;
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
                timer_reset = SEC_TIMER + 86400;
                last_ping = SEC_TIMER;
                //logToSerial(String(thetm.tm_year) + "-" + String(thetm.tm_mon) + "-" + String(thetm.tm_mday), 1);
                //logToSerial(String(timer_reset) + " - " + String(SEC_TIMER),1);
                break;

            case 3:  // ZÌsk·nÌ Ëasu
                strcpy(buffer, "\x02~");
                get_time(SEC_TIMER, buffer2);
                strcat(buffer, buffer2);
                strcat(buffer, "\n");
                client.print(buffer);
                break;

            case 1:  // Echo p¯Ìkaz
                client.print("OK\n");
                break;

            case 4:
                blank_line(3);
                lcd2.printf("%s", buffer + 2);
                break;
            case 5:
                blank_line(1);
                lcd2.printf("%s", buffer + 2);
                break;
            case 6:
                blank_line(0);
                lcd2.printf("%s", buffer + 2);
                break;
            case 7:
                blank_line(2);
                lcd2.printf("%s", buffer + 2);
                break;
            case 0xB:  // NastavenÌ timeoutu
                timer1 = atoi(buffer + 2);
                break;
            case 0xC:
                timeout1 = 0;
                break;
            case 0xE:  // ZobrazenÌ zpr·vy na LCD
                lcd2.setCursor(0, 0);
                lcd2.printf("*------------------*");
                lcd2.setCursor(0, 1);
                lcd2.printf("|                  |");
                lcd2.setCursor(((20 - strlen(buffer)) / 2) + 1, 1);
                lcd2.printf("%s", buffer + 2);
                lcd2.setCursor(0, 2);
                lcd2.printf("*------------------*");

                saved = 1;
                timeout1 = SEC_TIMER + timer2;
                break;

            case 0x16:  // Vymaz·nÌ displeje
                saved = 1;
                timeout1 = SEC_TIMER + timer2;
                break;

            case 0x22:  // P¯Ìkaz k signalizaci
                strcpy(buffer, "\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07\x07");
                last_ping = SEC_TIMER;
                for (counter1 = 0; counter1 <= 5; counter1++) {
                    Serial.write(buffer, 10);  // Signalizace pomocÌ sÈriovÈ komunikace
                    delay(20);  // 20 ms zpoûdÏnÌ
                }
                break;

            default:
                break;
            }
            logToSerial("e:" + String(SEC_TIMER), 0);
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

    // ZobrazenÌ stavu p¯enosu na LCD
    lcd2.setCursor(0, 0);
    lcd2.printf("*------------------*");
    lcd2.setCursor(0, 1);
    lcd2.printf("|  PRENASIM DATA   |");
    lcd2.setCursor(0, 2);
    lcd2.printf("| DO PC - CEKEJTE  |");
    lcd2.setCursor(0, 3);
    lcd2.printf("*------------------*");

    // P¯Ìprava pro odesl·nÌ dat
    if (!client.connected() || client.write("\x06A-100\n") != 7) {
        //net_on = 0;
        setNetOn(0, 3);
        return;
    }
    client.flush();
    delay(1000);

    timeout2 = SEC_TIMER + 2;  // Timeout na odpovÏÔ
    tmp[0] = '\0';

    // »ek·nÌ na odpovÏÔ od serveru
    while (((long)(SEC_TIMER - timeout2) <= 0) && (tmp[3] != '?')) {
        tmp[0] = '\0';
        server_read(tmp);
    }

    if (tmp[3] == '?') {
        timeout2 = SEC_TIMER + 30;  // Zak·z·nÌ p¯episu displeje
        last_ping = SEC_TIMER;
        long l = fifo_first;

        // Odesl·nÌ dat, dokud je sÌù p¯ipojena a existujÌ data
        while ((fifo_first + 3 < fifo_last) && net_on) {
            memset(off_buffer, 0x00, off_buffer_size);
            // Simulace naËtenÌ dat do `off_buffer`
            // xmem2root(off_buffer, fifo_first, off_buffer_size);

            tmp_send_data[0] = '\0';
            i = k = 0;
            set_first = 0;

            // V˝bÏr a form·tov·nÌ dat pro odesl·nÌ
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

            // Kontrola p¯ipojenÌ a odesl·nÌ dat
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
                            tmp[strlen(tmp) - 1] = '\0';  // OdstranÏnÌ znaku '\n'
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

            // ZobrazenÌ pokroku na LCD2
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

        // UvolnÏnÌ pamÏti FIFO
        if ((long)(fifo_first + 3 >= fifo_last)) {
            fifo_last = fifo_start;
            fifo_first = fifo_start;
        }
    }
    else {
        // Reset spojenÌ p¯i ne˙spÏchu
        client.stop();
        net_on = 0;
        delay(200);
    }
}

int server_read(char* buffer) {
    int len = 0;
    char tmp[2] = { 0 };  // Vyrovn·vacÌ pamÏù pro jednotlivÈ znaky
    unsigned long timeout = SEC_TIMER + 2;
    bool ok = false;
    *buffer = '\0';  // Inicializace bufferu na pr·zdn˝ ¯etÏzec

    while (!ok && ((long)(SEC_TIMER - timeout) <= 0)) {
        // »ekejte na data
        if (client.available()) {
            len = client.readBytes(tmp, 1);  // P¯eËtÏte jeden znak
            tmp[1] = '\0';  // UkonËovacÌ znak pro spr·vnÈ spojenÌ s ¯etÏzcem
            if (len > 0) {
                strcat(buffer, tmp);
                // Pokud narazÌme na nov˝ ¯·dek, ukonËujeme ËtenÌ
                if (tmp[0] == '\n') {
                    ok = true;
                }
            }
        }

        // Pokud û·dn· data nep¯ich·zÌ, ukonËÌme smyËku
        if (strlen(buffer) == 0) {
            ok = true;
        }
    }

    return strlen(buffer);  // VracÌ dÈlku naËten˝ch dat
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