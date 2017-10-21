extern "C" {
    bool system_update_cpu_freq(int);
}

#include <ESP.h>              // ESP
#include <SPI.h>              // SPI
#include <SD.h>               // SD
#include <ESP8266WiFi.h>      // ESPWifi

#include <Metro.h>            // Include Metro library - time
#include <TinyGPS++.h>        // Tiny GPS Plus Library
#include <SoftwareSerial.h>   // Software Serial Library so we can use other Pins for communication with the GPS module

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library
#include <ArduinoJson.h>      // Json library
#include <Ticker.h>

#include <IniFile.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

uint16_t RECV_PIN = D3;
IRrecv irrecv(RECV_PIN);

decode_results results;
char bufIR[8];
char postBufIR[8];
String selectX = "";
int countSelectX = 0;
int isExecuting = 0;

// TIMERS
Ticker tickArrow;           // 0.05 SEG.
Ticker tickFooter;          // 1.00 SEG.
Ticker tickSDGPSWriter;     // 2.00 SEG.
Ticker tickGpsReadSerial;   // 0.50 SEG.
Ticker tickIR;              // 0.50 SEG.
Ticker tickMenu;            // 0.40 SEG.
  
// SDCARD CONTROL
Sd2Card     card;
SdVolume  volume;
SdFile      root;

// SDCARD PIN
const int chipSelect = D4;
#define SD_SELECT D4

// DEFINES THE INITIA POSITION 
const double Home_LAT = -7.5330;  // Your Home Latitude
const double Home_LNG = -46.0359; // Your Home Longitude

// GPS LIB REFERENCE
TinyGPSPlus gps;

// WIFI CONFIGURATION
const char* host = "192.168.254.114"; // MAIN SERVER
const byte credentialCount = 2;       // NUMBER OF CREDENTIAL/WIFI NETWORKS THAT YOUR ESP8266 WILL TRY TO CONNECT
const char* credentials[] = {
  "rsgonzaga", "1248163264",
  "Sala-1907", "1248163264"
};

const byte bufMax = 20;
char ssid[bufMax];
char password[bufMax];

int countPing = 0;
int countPingTotal = 0;

boolean go = true;

// TFT CONFIGURATION
#define TFT_CS     D0
#define TFT_RST    -1  // you can also connect this to the Arduino reset
#define TFT_DC     D1

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//SCREEN CONFIGURATION

char strFooter[5];
String strSpinner = "|";
String x = "";               // Read Jog switch input

String keydown = "";       // Jog key pressed?
char sd_card = 0;       // SD Card inserted?

enum COLORS {
  BLACK = 0x0000, BLUE = 0x001F, RED = 0xF800, ORANGE = 0xFA60, GREEN = 0x07E0,
  CYAN = 0x07FF, MAGENTA = 0xF81F, YELLOW = 0xFFE0, GRAY = 0xCCCC, WHITE = 0xFFFF
};



//##################################################################################################
// METHODS
//##################################################################################################


void ScanWiFi() {
    bool found = false;

    WiFi.disconnect();
    delay(20);
    WiFi.mode(WIFI_STA);
    delay(20);
    //while (WiFi.status() != WL_CONNECTED) {}

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    //WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.println("scan start");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; (i < n) && !found; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.print((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "* ");
            delay(10);

            char foundssid[bufMax];
            WiFi.SSID(i).toCharArray(foundssid, bufMax);

            for (byte i = 0; i < credentialCount; i++) {
                if (strcmp(foundssid, credentials[i * 2]) == 0) {
                    Serial.print("<-- MATCH");
                    strcpy(ssid, credentials[i * 2]);
                    strcpy(password, credentials[i * 2 + 1]);
                    found = true;
                    exit;
                }
            }
            Serial.println();
        }
    }
    Serial.println("");
}

void wifiConfig(){
    ScanWiFi(); WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {

        delay(200);
        
        Serial.print(".");
        countPing++;

        if(countPing > 21){
            countPing = 0;
            countPingTotal++;
        }

        if(countPingTotal > 2){
          break;
        }
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

//##################################################################################################
// -----------------( DELAY FOR GPS )
//##################################################################################################

static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (Serial.available())
            gps.encode(Serial.read());
    } while (millis() - start < ms);
}

//##################################################################################################
// -----------------( LEITURA DE ARQUIVO DE CONFIGURAÇÃO )
//##################################################################################################
void readConfigFile() {

  const char *filename = "/CFG.TXT";
  const size_t bufferLen = 80;
  char buffer[bufferLen];

  IniFile ini(filename);
  if (!ini.open()) {
    Serial.print("Ini file ");
    Serial.print(filename);
    Serial.println(" does not exist");
    // Cannot do anything else
    while (1)
      ;
  }
  Serial.println("Ini file exists");

  // Check the file is valid. This can be used to warn if any lines
  // are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print("ini file ");
    Serial.print(ini.getFilename());
    Serial.print(" not valid: ");
    while (1)
      ;
  }

  // Fetch a value from a key which is present
  if (ini.getValue("network", "mac", buffer, bufferLen)) {
    Serial.print("section 'network' has an entry 'mac' with value ");
    Serial.println(buffer);
  }
  else {
    Serial.print("Could not read 'mac' from section 'network', error was ");
  }
}

// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU

    char* menu[] = {"  MENU PRINCIPAL",
                    "  OPCAO 1     ",
                    "  OPCAO 2     ",
                    "  OPCAO 3     ",
                    "  OPCAO 4     ",
                    "  OPCAO 5     ",
                    "  OPCAO 6     ",
                    "  OPCAO 7     "};
    #define numMenu (sizeof(menu)/sizeof(char *))-1 //array size]
    #define menu_top 14   // Postition of first menu item from top of screen
    char menu_select;     // Currently elected menu item

    // It's usefult to know the number of menu items without hardcoding it
    // We can calculate it thus.. (subtract 1 for the menu heading)


    void tftMenuInit() {
      // Clear screen and display the menu
      char i;

      tft.setTextWrap(false);
      tft.fillScreen(ST7735_BLACK);
      tft.setTextSize(1);

      tft.setCursor(0, 0);
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.println(menu[0]);

      tft.drawLine(0, 9, tft.width()-1, 9, ST7735_GREEN);

      tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
      for(i=1;i<=numMenu;i++) {
         tft.setCursor(0, ((i-1)*10)+menu_top);
         tft.println(menu[i]);
      }
    }

    void tftMenuSelect(char menuitem)
    {
      // Highlight a selected menu item
      char i;
      // Remove highlight of current item
      tft.setCursor(0, ((menu_select-1)*10)+menu_top);
      tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
      tft.println(menu[menu_select]);
      // Highlight new menu item
      tft.setCursor(0, ((menuitem-1)*10)+menu_top);
      tft.setTextColor(ST7735_YELLOW, ST7735_BLUE);
      tft.println(menu[menuitem]);
      // change menu_select to new item
      menu_select=menuitem;

    }

    void tftPrintTest() {
      // Print different font sizes
      char * hello="Hello World!";
      tft.setTextWrap(false);
      tft.fillScreen(ST7735_BLACK);
      tft.setCursor(0, 30);
      tft.setTextColor(ST7735_MAGENTA);
      tft.setTextSize(1);
      tft.println(hello);
      tft.setTextColor(ST7735_YELLOW);
      tft.setTextSize(2);
      tft.println(hello);
      tft.setTextColor(ST7735_GREEN);
      tft.setTextSize(3);
      tft.println(hello);
      tft.setTextColor(ST7735_BLUE);
      tft.setTextSize(4);
      tft.print(1234.567);
    }

    void tftBarGraphTest(void) {
      // Print Bar Graph
      int16_t x;
      unsigned char origin_x=10;
      unsigned char origin_y=115;
      unsigned char width=15;
      unsigned char spacing=20;
      unsigned char height=10;
      tft.fillScreen(ST7735_BLACK);

      tft.drawLine(origin_x, origin_y, origin_x, 1, ST7735_BLUE);
      tft.drawLine(origin_x, origin_y, tft.width(), origin_y, ST7735_BLUE);

      for (int16_t x=origin_x+1; x <tft.width()-spacing; x+=spacing) {
        tft.fillRect(x, origin_y-height , width, height, ST7735_GREEN);
        height+=10;
      }
      
      tft.setTextColor(ST7735_WHITE);
      tft.setTextSize(1);
      tft.setCursor(0, origin_y);
      tft.print("0");
      tft.setCursor(0, 1);
      tft.print("10");
      for (char i=0; i <7; i++) {
        x=origin_x+8+(spacing*i);
        tft.setCursor(x, origin_y+3);
        tft.print(i+1);
      }
      tft.setCursor(40, 20);
      tft.print("Bar Graph");

      delay(2000);
    }

    void guiFooter(){
      //  footer for the screen
      tft.setTextColor(GREEN, BLACK);
      tft.setCursor(0,105);
      tft.print(gps.time.hour()); tft.print(":"); tft.print(gps.time.minute()); tft.print(":"); tft.println(gps.time.second());

      Serial.print(gps.time.hour()); Serial.print(":"); Serial.print(gps.time.minute()); Serial.print(":"); Serial.println(gps.time.second());
    }


    //##################################################################################################
    // -----------------( MENU OPTIONS )
    //##################################################################################################

    typedef void (* MenuFuncPtr) (); // this is a typedef to the menu functions

    MenuFuncPtr menu_func[] = {0,
                    tftBarGraphTest,
                    tftPrintTest,
                    tftBarGraphTest,
                    tftBarGraphTest,
                    tftBarGraphTest,
                    tftBarGraphTest,
                    tftBarGraphTest};

// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU
// MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU --  MENU













uint8_t gpsIcon[10] = {

  0b01000000, //       #
  0b01100000, //      ##
  0b01010000, //     # #
  0b01011000, //    ## #
  0b01101100, //   ## ##
  0b01100010, //  #   ##
  0b01111111, // #######
  0b01110000, //     ###
  0b01100000, //      ##
  0b01000000, //       #

};

 // 'tractorcolor'
const unsigned char meeeeeeeeee [] PROGMEM = {
  0b00000111,0b11111111,0b00000000,0b00000000, //      ###########
  0b00001111,0b11111111,0b10000000,0b00000000, //     #############
  0b00000111,0b00000001,0b00000000,0b00000000, //      ###       #
  0b00000111,0b00000001,0b00000000,0b00000000, //      ###       #
  0b00000111,0b00000001,0b00000000,0b00000000, //      ###       #
  0b00000111,0b00000001,0b10000000,0b00000000, //      ###       ##
  0b00001111,0b00000000,0b10000000,0b00000000, //     ####        #
  0b00001111,0b00000000,0b10000000,0b00000000, //     ####        #
  0b00001111,0b11100000,0b10000000,0b00000000, //     #######     #
  0b00000000,0b00011111,0b11100000,0b00000000, //            ########
  0b00011111,0b11000111,0b11111111,0b11100000, //    #######   ##############
  0b01111111,0b11110011,0b11111111,0b11110000, //  ###########  ##############
  0b11110000,0b01111001,0b11111111,0b11110000, // ####     ####  #############
  0b01001111,0b10011100,0b11111111,0b11110000, //  #  #####  ###  ############
  0b00111111,0b11101110,0b11111111,0b11110000, //   ######### ### ############
  0b01111111,0b11110110,0b00000000,0b00000000, //  ########### ##
  0b01111111,0b11110111,0b01111001,0b11100000, //  ########### ### ####  ####
  0b11111000,0b11111011,0b01110111,0b11111000, // #####   ##### ## ### ########
  0b11110000,0b01111011,0b01110111,0b11111000, // ####     #### ## ### ########
  0b11110000,0b01111000,0b11101111,0b00111100, // ####     ####   ### ####  ####
  0b11110000,0b01111011,0b11101110,0b00011100, // ####     #### ##### ###    ###
  0b11111000,0b11111011,0b11101110,0b00011100, // #####   ##### ##### ###    ###
  0b01111111,0b11110000,0b00001111,0b00111100, //  ###########        ####  ####
  0b01111111,0b11110000,0b00000111,0b11111000, //  ###########         ########
  0b00111111,0b11100000,0b00000111,0b11111000, //   #########          ########
  0b00001111,0b10000000,0b00000001,0b11100000, //     #####              ####
};


void onSpinnerArrow(){

}

void readIR(){
  if (irrecv.decode(&results)) {
    // print() & println() can't handle printing long longs. (uint64_t)
    serialPrintUint64(results.value, HEX);
    
    sprintf(bufIR, "%02x", results.value );
    //sprintf(out,"%08X",strlint);

    Serial.println("");
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    Serial.println(bufIR);
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    irrecv.resume();  // Receive the next value
  }
}

void checkMenu(){

  if (strcmp(bufIR, "739e7c6") == 0) {
    // Select
    menu_func[menu_select]();       // Call the appropriate menu function from array
                                    // Note the syntax for doing this
    delay(2000);
    tftMenuInit();                  // Redraw the Menu
    tftMenuSelect(menu_select);     // Highlight the current menu item
    memset(bufIR, 0, sizeof bufIR);
  };

  if (strcmp(bufIR, "807620df") == 0) {
    // Up
    // move up one menu item, if at top wrap to bottom
    //keydown=1;    
    if (menu_select>1){ tftMenuSelect(menu_select-1); } else { tftMenuSelect(numMenu); }

    memset(bufIR, 0, sizeof bufIR);

  } else if (strcmp(bufIR, "BFD6AA08") == 0){
    // Left
    //keydown=1;    
  } else if (strcmp(bufIR, "8076a05f") == 0){
    // Down
    // move down one menu item, if at bottom wrap to top
    //keydown=1;    
    if (menu_select<numMenu){ tftMenuSelect(menu_select+1); } else { tftMenuSelect(1); };

    memset(bufIR, 0, sizeof bufIR);

  } else if (strcmp(bufIR, "80767887") == 0) {
    // Right
    //keydown=1;    
  } else if (strcmp(bufIR, "8076d827") == 0) {
    //keydown=0; // key released
  }
}

void sdWriteGps(){
  
  File myFile = SD.open("datalog.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {

      myFile.print("{lat:");
      myFile.print(gps.location.lat(), 5);

      myFile.print(", lng:");
      myFile.print(gps.location.lng(), 4);

      myFile.print(", time:");
      myFile.print(gps.time.hour()); // GPS time UTC
      myFile.print("-");
      myFile.print(gps.time.minute()); // Minutes
      myFile.print("-");
      myFile.print(gps.time.second());
      myFile.println("}");
      myFile.println("-----------------------------------------");

      // close the file:
      myFile.close();
      // Rafael // display.println("done.");
      Serial.println("done.");
  } else {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");

      myFile.close();
  }
  
  myFile.close();

  //Serial.print("ESP.getBootMode(); "); Serial.println(ESP.getBootMode());
  //Serial.print("ESP.getSdkVersion(); "); Serial.println(ESP.getSdkVersion());
  //Serial.print("ESP.getBootVersion(); "); Serial.println(ESP.getBootVersion());
  //Serial.print("ESP.getChipId(); "); Serial.println(ESP.getChipId());
  //Serial.print("ESP.getFlashChipSize(); "); Serial.println(ESP.getFlashChipSize());
  //Serial.print("ESP.getFlashChipRealSize(); "); Serial.println(ESP.getFlashChipRealSize());
  //Serial.print("ESP.getFlashChipSizeByChipId(); "); Serial.println(ESP.getFlashChipSizeByChipId());
  //Serial.print("ESP.getFlashChipId(); "); Serial.println(ESP.getFlashChipId());
  //Serial.print("ESP.getFreeHeap(); "); Serial.println(ESP.getFreeHeap());
  //Serial.print("ESP.getCpuFreqMHz(); "); Serial.println(ESP.getCpuFreqMHz());

}


/***
  * Configura o cartão SD do aparelho
  *
*/
void sdCardConfig(){

    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, chipSelect)) {
        Serial.println("Deu merda");
    } else {
         delay(2000);
        if (!SD.begin(chipSelect)) {
            Serial.println("** BOSTA **");
        } else {
            Serial.println("** AEEEE **");
        }
    }

    yield();

    switch (card.type()) {
    case SD_CARD_TYPE_SD1:
        Serial.println("SD1");
        break;
    case SD_CARD_TYPE_SD2:
        Serial.println("SD2");
        break;
    case SD_CARD_TYPE_SDHC:
        Serial.println("SDHC");
        break;
    default:
        Serial.println("Unknown");
    }

    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
        Serial.println("Não sei o sistema");
        //return;
    }

    yield();

    uint32_t volumesize;

    Serial.print("FAT ");
    Serial.println(volume.fatType(), DEC);

    volumesize = volume.blocksPerCluster(); volumesize *= volume.clusterCount(); volumesize *= 512; volumesize /= 1024;
    Serial.print("Volume size (Mbytes): ");
    
    volumesize /= 1024;
    Serial.println(volumesize);

    Serial.println("Files: ");
    root.openRoot(volume);

    root.ls(LS_R | LS_DATE | LS_SIZE);
}

void setup() {

    yield();

                        //--------------------------------
    // -----------------// Força a frequencia em 160 mhz //-----------------------------------------------
                        //--------------------------------
    system_update_cpu_freq(160);

    //##################################################################################################
    // -----------------( Inicia a seria com a velocidade 9600 )
    //##################################################################################################
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.setDebugOutput(true);

    //##################################################################################################
    // -----------------(Delay )
    //##################################################################################################
    delay(10);

    Serial.println("") + Serial.println("1 - Serial 9600 iniciado");
    Serial.println(""); Serial.println("2 - FlashChipSpeed: "); Serial.print(ESP.getFlashChipSpeed());


    //##################################################################################################
    // -----------------( INICIA A TELA DEFININDO O TAMANHO DELA - ROTACIONA A TELA )
    //##################################################################################################
    // Use this initializer if you're using a 1.8" TFT
    tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
    //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab
    Serial.println("") + Serial.println("3 - Display iniciado");
    tft.setRotation(3);
    Serial.println("") + Serial.println("4 - Rotação do display");
    yield();

    //##################################################################################################
    // -----------------( PREENCHE O DISPLAY )
    //##################################################################################################
    tft.fillScreen(ST7735_BLACK);
    Serial.println("") + Serial.println("5 - Display preenchido com preto");

    //##################################################################################################
    // -----------------( CONFIGURA SDCARD )
    //##################################################################################################
    pinMode(SD_SELECT, OUTPUT);
    digitalWrite(SD_SELECT, HIGH); // disable SD card
    Serial.println("") + Serial.println("6 - SD Card OUTPU e HIGH");

    //##################################################################################################
    // -----------------( Inicia o menu e desenha a tela )
    //##################################################################################################
    tftMenuInit();                    // Draw menu
    menu_select=1;                    // Select 1st menu item
    tftMenuSelect(menu_select);       // Highlight selected menu item
    guiFooter();
    tft.drawBitmap(120, 0, gpsIcon, 7, 10, ST7735_GREEN, ST7735_BLACK);

    tft.drawBitmap(40, 60, meeeeeeeeee, 30, 26, ST7735_GREEN, ST7735_BLACK);

    yield();

    //##################################################################################################
    // -----------------( Inicia o sistema de arquivos )
    //##################################################################################################
    Serial.println(""); Serial.println("7 - Tentando iniciar o cartão");
    Serial.println(""); Serial.print("8 - SPI_HALF_SPEED: "); Serial.print(SPI_HALF_SPEED); Serial.println("");

    // Configura e inicia o cartão SD
    sdCardConfig();

    // Da tempo para o atividades em segundo plano
    yield();

    // Conecta na rede configurada
    wifiConfig();

    // Faz a leitura do arquivo de configuração
    readConfigFile();

    //tick.attach(0.2, onSpinner); //Executa oneMinute a cada 60 segundos
    //tickArrow.attach(0.05, onSpinnerArrow);

    //tickGpsReadSerial.attach(0.5, gpsReadSerial);
    tickSDGPSWriter.attach(5,sdWriteGps);

    tickFooter.attach(1, guiFooter);

    irrecv.enableIRIn(); // Start the receiver
    tickIR.attach(0.5, readIR);

    //tickMenu.attach(0.4, checkMenu);

    // Tempo para atividades de segundo plano
    yield();

}


void loop() {

  delay(10);

  smartDelay(500); // Run Procedure smartDelay

  checkMenu();

  yield();
}
