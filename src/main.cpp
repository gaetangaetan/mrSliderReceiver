// version artnet en cours

/* basée sur version fonctionnelle du 15 06 2023
D1 bouton
D2 datapin LED strip
D5 GND bouton
75 LEDs
Mode 0 : 234 RGB for all strip at once
Mode 1 : 234 RGB + 5 LENGTH + 6 OFFSET    
Mode 2 : 234 RGB + 5 LENGTH + 6 OFFSET tapered
Mode 3 : 234 RGB + 5 LENGTH + 6 OFFSET + 7 OFFSET TUBE  
Mode 4 : 234 RGB + 5 LENGTH + 6 OFFSET + 7 OFFSET TUBE (tapered)
Mode 5 : individual rgb 234 567 ...
Mode 6 : RGB séparé pour chaque groupe : 234 RGB GRP0, 567 RGB GRP1, 8910 RGB GRP2, ...
Mode 7 : FX1 Segments montant : 6 canaux par groupe : 234 RGB + 5 longueur segment + 6 longueur espace (pixels éteints entre deux segments) + 7 speed (0 = vitesse max vers le bas, 128 = arrêt, 255 = vitesse max vers le haut)
Mode 8 : FX2 Segments montant R G B : 6 canaux par groupe : 234 RGB + 5 6 7 longueur segment rgb + 8 9 10 longueur espace rgb (pixels éteints entre deux segments) + 11 12 13 speed r g b (0 = vitesse max vers le bas, 128 = arrêt, 255 = vitesse max vers le haut)
Mode 255 : on affiche le numéro du groupe (0 à 10)

SETUP (clic long pour y accéder ou en sortir) : réglage du numéro de groupe
Le nombre de LEDS correspondant au numéro de groupe clignote
Le numéro de groupe est enregistré en EEPROM
*/ 

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
WiFiManager wifiManager;
#define APNAME "mrLEDTUBE12"
#define VERSION 14

#include <ArtnetWifi.h>
WiFiUDP UdpSend;
ArtnetWifi artnet;

#define EEPROM_SIZE 32
#define DMXMODE true
#define ARTNETMODE false
bool runningMode = DMXMODE;

#include <FastLED.h>



// #include <TM1637Display.h>
// TM1637Display display(D7, D6); // clck DIO

// #include "OneButton.h"
// OneButton button1(D1, true);
// Setup a new OneButton on pin D6.  


#include <ESP8266WiFiMulti.h>
#include <espnow.h>

int setupAddress = 1;
int setupMode = 254;
int setupTubeNumber = 1;



int flashInterval;

uint8_t dmxChannels[512];

typedef struct struct_message {
    uint8_t status;
    uint8_t data;    
} struct_message;


struct_message incomingMessage;
struct_message outgoingMessage;

typedef struct struct_dmx_message {
    uint8_t dmx001;
    uint8_t dmx002;
    uint8_t dmx003;
    uint8_t dmx004;
        
} struct_dmx_message;

struct_dmx_message incomingDMXMessage;

typedef struct struct_dmx_packet {
    uint8_t blockNumber; // on divise les 512 adresses en 4 blocs de 128 adresses
    uint8_t dmxvalues[128];   
} struct_dmx_packet;

struct_dmx_packet incomingDMXPacket;


void OnDataSent(u8 *mac_addr, u8 status) {  
     
}


// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingDMXPacket, incomingData, sizeof(incomingDMXPacket));  
  uint8_t packetNumber = incomingDMXPacket.blockNumber;
  for(int i=0;i<128;i++)
  {
    dmxChannels[(packetNumber*128)+i]=incomingDMXPacket.dmxvalues[i];
  }
 
}



double mrdoublemodulo(double nombre, double diviseur)
{
  while(nombre<0)nombre+=diviseur;
  while(nombre>=diviseur)nombre-=diviseur;
  return nombre;
}

void DMX2LEDSTRIP()
{
  float position = 0;
  float pan = 0;
  float tilt = 0;

  setupMode = dmxChannels[300];

  switch (setupMode)
  {
  case 1: // mouvement slider

    position = dmxChannels[301];
    Serial.print("x ");
    Serial.println(position);
    delay(100);
    pan = dmxChannels[302];
    Serial.print("p ");
    Serial.println(pan);
    delay(100);
    tilt = dmxChannels[303];
    Serial.print("t ");
    Serial.println(tilt);
    delay(100);
    break;

  case 2: // enregistrement positions array

    Serial.println("C"); // clear all positions in the moves array
    Serial.println("["); // move to the first position in the moves array

    for (int i = 0; i < 50; i += 5)
    {
      position = dmxChannels[306 + i];
      pan = dmxChannels[307 + i];
      tilt = dmxChannels[308 + i];

      Serial.print("x ");
      Serial.println(position);
      Serial.print("p ");
      Serial.println(pan);
      Serial.print("t ");
      Serial.println(tilt);
      delay(5000);
      Serial.println("#"); // add current position to the moves array
      delay(100);
    }

    break;

    case 3: // lancement séquence

    Serial.println("["); // move to the first position in the moves array
    // todo
    
    break;
  }
}



void setup() {

  
  Serial.begin(57600);
    
  
  WiFi.disconnect();
  ESP.eraseConfig();
 
  // Wifi STA Mode
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("\nESP-Now Receiver");
  
  // Initializing the ESP-NOW
   if (esp_now_init() != 0) {
    Serial.println("Problem during ESP-NOW init");
    return;
     }

    esp_now_register_recv_cb(OnDataRecv);
  

  
  
// EEPROM.begin(EEPROM_SIZE);
// setupAddress = EEPROM.read(0);
// setupMode = EEPROM.read(4);
// setupTubeNumber = EEPROM.read(8);
// if((setupAddress<1)||(setupAddress>512))setupAddress=1;
// if((setupMode<1)||(setupMode>255))setupMode=1;
// if((setupTubeNumber<0)||(setupTubeNumber>32))setupTubeNumber=0;


  


}

void loop() {
  int mode = dmxChannels[299];
  //DMX2LEDSTRIP();
  switch (mode)
  {
  case 1: // mouvement slider
          /* code */
    Serial.print(" || dmx300 =");
    Serial.print(dmxChannels[299]);
    Serial.print(" | dmx301 =");
    Serial.print(dmxChannels[300]);
    Serial.print(" | dmx302 =");
    Serial.print(dmxChannels[301]);
    Serial.print(" | dmx303 =");
    Serial.print(dmxChannels[302]);
    Serial.print(" | dmx304 =");
    Serial.print(dmxChannels[303]);
    Serial.print(" | dmx305 =");
    Serial.println(dmxChannels[304]);
    delay(100);
    break;

  case 2: // lancement séquence
          /* code */
    Serial.println("Tableau des positions, pan, tilt, delay speed");    
    for(int i=1;i<10;i++)
    {
      Serial.print("Position ");
      Serial.print(i) ;
      Serial.print(" : x =");
      Serial.print(dmxChannels[305+(5*i)]) ;
      Serial.print(" | pan =");
      Serial.print(dmxChannels[306+(5*i)]) ;
      Serial.print(" | tilt =");
      Serial.print(dmxChannels[307+(5*i)]) ;
      Serial.print(" | speed =");
      Serial.print(dmxChannels[308+(5*i)]) ;
      Serial.print(" | delay =");
      Serial.print(dmxChannels[309+(5*i)]) ;
      Serial.println(" ");
    }
    delay(1000);
    break;

  default:
    break;
  }
}



// void setup() {
// Serial.begin(57600) ;
// }

// void loop() {
//   Serial.println("x 10");
//   delay(4000);
//   Serial.println("x -20");
//   delay(5000);

// }

