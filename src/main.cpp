#define FIRMWARE_VERSION 118

#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
// D5 6 7 ok en input ou output sans restriction
// dir D0 D3 D4 step D5 D6 D7
#define PIN_SLIDER_DIR D0
#define PIN_PAN_DIR D3
#define PIN_TILT_DIR D4
#define PIN_SLIDER_STEP D5
#define PIN_PAN_STEP D6
#define PIN_TILT_STEP D7

AccelStepper stepper_slider = AccelStepper(1, PIN_SLIDER_STEP, PIN_SLIDER_DIR);
AccelStepper stepper_pan = AccelStepper(1, PIN_PAN_STEP, PIN_PAN_DIR );
AccelStepper stepper_tilt = AccelStepper(1, PIN_TILT_STEP, PIN_TILT_DIR);
/**
   The MIT License (MIT)

   Copyright (c) 2018 by ThingPulse, Daniel Eichhorn
   Copyright (c) 2018 by Fabrice Weinberg

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   ThingPulse invests considerable time and money to develop these open source libraries.
   Please support us by buying our products (and not the clones) from
   https://thingpulse.com

*/

// Include the correct display library

// For a connection via I2C using the Arduino Wire include:
// #include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
// OR #include "SH1106Wire.h"   // legacy: #include "SH1106.h"

// For a connection via I2C using brzo_i2c (must be installed) include:
// #include <brzo_i2c.h>        // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Brzo.h"
// OR #include "SH1106Brzo.h"

// For a connection via SPI include:
// #include <SPI.h>             // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Spi.h"
// OR #include "SH1106SPi.h"


// Optionally include custom images
//#include "images.h"


// Initialize the OLED display using Arduino Wire:
// SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SSD1306Wire display(0x3c, D3, D5);  // ADDRESS, SDA, SCL  -  If not, they can be specified manually.
// SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);  // ADDRESS, SDA, SCL, OLEDDISPLAY_GEOMETRY  -  Extra param required for 128x32 displays.
// SH1106Wire display(0x3c, SDA, SCL);     // ADDRESS, SDA, SCL

// Initialize the OLED display using brzo_i2c:
// SSD1306Brzo display(0x3c, D3, D5);  // ADDRESS, SDA, SCL
// or
// SH1106Brzo display(0x3c, D3, D5);   // ADDRESS, SDA, SCL

// Initialize the OLED display using SPI:
// D5 -> CLK
// D7 -> MOSI (DOUT)
// D0 -> RES
// D2 -> DC
// D8 -> CS
// SSD1306Spi display(D0, D2, D8);  // RES, DC, CS
// or
// SH1106Spi display(D0, D2);       // RES, DC


// #define DEMO_DURATION 3000
// typedef void (*Demo)(void);

// int demoMode = 0;
// int counter = 1;

// void setup() {
//   Serial.begin(115200);
//   Serial.println();
//   Serial.println();


//   // Initialising the UI will init the display too.
//   display.init();

//   display.flipScreenVertically();
//   display.setFont(ArialMT_Plain_10);

// }

// void drawFontFaceDemo() {
//   // Font Demo1
//   // create more fonts at http://oleddisplay.squix.ch/
//   display.setTextAlignment(TEXT_ALIGN_LEFT);
//   display.setFont(ArialMT_Plain_10);
//   display.drawString(0, 0, "Hello world");
//   display.setFont(ArialMT_Plain_16);
//   display.drawString(0, 10, "Hello world");
//   display.setFont(ArialMT_Plain_24);
//   display.drawString(0, 26, "Hello world");
// }

// void drawTextFlowDemo() {
//   display.setFont(ArialMT_Plain_10);
//   display.setTextAlignment(TEXT_ALIGN_LEFT);
//   display.drawStringMaxWidth(0, 0, 128,
//                              "Lorem ipsum\n dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore." );
// }

// void drawTextAlignmentDemo() {
//   // Text alignment demo
//   display.setFont(ArialMT_Plain_10);

//   // The coordinates define the left starting point of the text
//   display.setTextAlignment(TEXT_ALIGN_LEFT);
//   display.drawString(0, 10, "Left aligned (0,10)");

//   // The coordinates define the center of the text
//   display.setTextAlignment(TEXT_ALIGN_CENTER);
//   display.drawString(64, 22, "Center aligned (64,22)");

//   // The coordinates define the right end of the text
//   display.setTextAlignment(TEXT_ALIGN_RIGHT);
//   display.drawString(128, 33, "Right aligned (128,33)");
// }

// void drawRectDemo() {
//   // Draw a pixel at given position
//   for (int i = 0; i < 10; i++) {
//     display.setPixel(i, i);
//     display.setPixel(10 - i, i);
//   }
//   display.drawRect(12, 12, 20, 20);

//   // Fill the rectangle
//   display.fillRect(14, 14, 17, 17);

//   // Draw a line horizontally
//   display.drawHorizontalLine(0, 40, 20);

//   // Draw a line horizontally
//   display.drawVerticalLine(40, 0, 20);
// }

// void drawCircleDemo() {
//   for (int i = 1; i < 8; i++) {
//     display.setColor(WHITE);
//     display.drawCircle(32, 32, i * 3);
//     if (i % 2 == 0) {
//       display.setColor(BLACK);
//     }
//     display.fillCircle(96, 32, 32 - i * 3);
//   }
// }

// void drawProgressBarDemo() {
//   int progress = (counter / 5) % 100;
//   // draw the progress bar
//   display.drawProgressBar(0, 32, 120, 10, progress);

//   // draw the percentage as String
//   display.setTextAlignment(TEXT_ALIGN_CENTER);
//   display.drawString(64, 15, String(progress) + "%");
// }

// void drawImageDemo() {
//   // see http://blog.squix.org/2015/05/esp8266-nodemcu-how-to-create-xbm.html
//   // on how to create xbm files
//  // display.drawXbm(34, 14, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
// }

// Demo demos[] = {drawFontFaceDemo, drawTextFlowDemo, drawTextAlignmentDemo, drawRectDemo, drawCircleDemo, drawProgressBarDemo, drawImageDemo};
// int demoLength = (sizeof(demos) / sizeof(Demo));
// long timeSinceLastModeSwitch = 0;

// void loop() {
//   // clear the display
//   display.clear();
//   // draw the current demo method
//   demos[demoMode]();

//   display.setFont(ArialMT_Plain_10);
//   display.setTextAlignment(TEXT_ALIGN_RIGHT);
//   display.drawString(128, 54, String(millis()));
//   // write the buffer to the display
//   display.display();

//   if (millis() - timeSinceLastModeSwitch > DEMO_DURATION) {
//     demoMode = (demoMode + 1)  % demoLength;
//     timeSinceLastModeSwitch = millis();
//   }
//   counter++;
//   delay(10);
// }
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

int lastposX= 0;
int lastPan=0;
int lastTilt=0;
int lastSpeed=0;
int lastAccel=0;

int coeffPosX = 200;
int coeffPan = 100;
int coeffTilt = 30;

int coeffSpeedPosX = 100;
int coeffSpeedPan = 40;
int coeffSpeedTilt = 40;


int coeffAccelPosX = 100;
int coeffAccelPan = 50;
int coeffAccelTilt = 50;


/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x64 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

// #include <SPI.h>
// #include <Wire.h>


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



void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
  
}

void update_finished() {
  
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void updateFirmware()
{

  ESPhttpUpdate.setClientTimeout(2000);  // default was 8000
  
  if ((WiFi.status() == WL_CONNECTED)) {

    WiFiClient client;

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    String firmwareURL = "http://mrsliderfirmware.gaetanstreel.com/firmware.bin";
    firmwareURL.concat(FIRMWARE_VERSION+1);

    Serial.print("firmwareURL = ");
    Serial.println(firmwareURL);
    
  


    t_httpUpdate_return ret = ESPhttpUpdate.update(client, firmwareURL.c_str());
    // Or:
    // t_httpUpdate_return ret = ESPhttpUpdate.update(client, "server", 80, "file.bin");

    switch (ret) {
      case HTTP_UPDATE_FAILED: Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str()); 
                
        break;

      case HTTP_UPDATE_NO_UPDATES: Serial.println("HTTP_UPDATE_NO_UPDATES"); break;

      case HTTP_UPDATE_OK: Serial.println("HTTP_UPDATE_OK"); 
          
      break;
    }
  }
  
}

void setup() {

  
  Serial.begin(57600);




// initialisation des pins du step motor driver
  pinMode(PIN_SLIDER_STEP, OUTPUT);
  pinMode(PIN_SLIDER_DIR, OUTPUT);
  pinMode(PIN_PAN_STEP, OUTPUT);
  pinMode(PIN_PAN_DIR, OUTPUT);
  pinMode(PIN_TILT_STEP, OUTPUT);
  pinMode(PIN_TILT_DIR, OUTPUT);
  // pinMode(D5,OUTPUT);
  // pinMode(D6,OUTPUT);

    


  // display.init();

  // display.flipScreenVertically();
  // display.setFont(ArialMT_Plain_10);
 
  
  WiFi.disconnect();
  ESP.eraseConfig();
 
  // Wifi STA Mode
  WiFi.mode(WIFI_STA);
  // Get Mac Add
  //Serial.print("Mac Address: ");
  Serial.print(WiFi.macAddress());
  //Serial.println("\nESP-Now Receiver");
  
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


//pour signifier la version, on fait bouger le tilt  



}

void loop2() {
  int mode = dmxChannels[299];
  //DMX2LEDSTRIP();
  switch (mode)
  {
  case 1: // mouvement slider
          /* code */


    if(dmxChannels[303]!=0)
    {
        Serial.print(0x04); //INSTRUCTION_BYTES_SLIDER_PAN_TILT_SPEED 
        
        Serial.print(0x00);
        Serial.print(0x00);
        
        Serial.print(0x00);
        Serial.print(0x00);

        Serial.print(0x00);
        byte tiltspeed = dmxChannels[304];
        Serial.println(tiltspeed);
        delay(10);
      
      return;
    }

    if(dmxChannels[300]!=lastposX)
    {
      lastposX = dmxChannels[300];
      Serial.print("x ");
      Serial.println(dmxChannels[300]);
      delay(100);
    }

    if(dmxChannels[301]!=lastPan)
    {
      lastPan = dmxChannels[301];
      Serial.print("p ");
      Serial.println(dmxChannels[301]);
      delay(100);
    }

    if(dmxChannels[302]!=lastTilt)
    {
      lastTilt = dmxChannels[302];
      Serial.print("t ");
      Serial.println(dmxChannels[302]);
      delay(100);
    }

    
    
//      // clear the display
//    display.clear();
// //   // draw the current demo method


//    display.setFont(ArialMT_Plain_10);
//    display.setTextAlignment(TEXT_ALIGN_LEFT);
   
//   // display.drawString(1,1, String(dmxChannels[302]));
//    display.drawString(1,1, String(dmxChannels[300]));
// //   // write the buffer to the display
//    display.display();
//    delay(10);
    
    break;

  case 2: // lancement séquence
          /* code */
    Serial.println("Tableau des positions, pan, tilt, delay speed");    
    for(int i=1;i<10;i++)
    {
      // Serial.print("Position ");
      // Serial.print(i) ;
      // Serial.print(" : x =");
      // Serial.print(dmxChannels[305+(5*i)]) ;
      // Serial.print(" | pan =");
      // Serial.print(dmxChannels[306+(5*i)]) ;
      // Serial.print(" | tilt =");
      // Serial.print(dmxChannels[307+(5*i)]) ;
      // Serial.print(" | speed =");
      // Serial.print(dmxChannels[308+(5*i)]) ;
      // Serial.print(" | delay =");
      // Serial.print(dmxChannels[309+(5*i)]) ;
      // Serial.println(" ");
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

void setSpeed(int speed)
{
      stepper_slider.setMaxSpeed(speed * coeffSpeedPosX );
      stepper_pan.setMaxSpeed(speed * coeffSpeedPan );
      stepper_tilt.setMaxSpeed(speed * coeffSpeedTilt );
}

void gotoPosition(int posNumber)
{
   stepper_slider.moveTo(dmxChannels[300+(5*posNumber)]*coeffPosX);
    stepper_pan.moveTo(dmxChannels[301 + (5*posNumber)]*coeffPan);
    stepper_tilt.moveTo(dmxChannels[302+ (5*posNumber)]*coeffTilt);
    while((stepper_slider.distanceToGo()>0) || (stepper_pan.distanceToGo()>0) || (stepper_tilt.distanceToGo()>0))
    {
          stepper_slider.run();
          stepper_pan.run();
          stepper_tilt.run();
    }
}

void loop() {
  if(dmxChannels[500]==255)
  {
    dmxChannels[500]=0;
      // update automatique à chaque allumage (pendant la conception)
  
//   // Initialising the UI will init the display too.
  
  
   WiFi.begin("OpenPoulpy", "youhououhou");
          
          int tentatives = 0;
      while (WiFi.status() != WL_CONNECTED)
      {
          delay(1000);
          
          Serial.print(".");
          
          if (tentatives > 20)
          {            
            break;
          }
          tentatives++;
      }
    updateFirmware();
    delay(1000);
    // fin update firmware (à retirer quand le code sera bon)
    
  }
  int currentpos = dmxChannels[299];



  if(currentpos<11)
  {
    if(dmxChannels[300+(5*currentpos)]!=lastposX)
    {
      lastposX=dmxChannels[300+(5*currentpos)];
      stepper_slider.moveTo(lastposX*coeffPosX);   	
    }
    
    if(dmxChannels[301+(5*currentpos)]!=lastPan)
    {
      lastPan=dmxChannels[301+(5*currentpos)];
      stepper_pan.moveTo(lastPan*coeffPan);   	
    }
    
    if(dmxChannels[302+(5*currentpos)]!=lastTilt)
    {
      lastTilt=dmxChannels[302+(5*currentpos)];
      stepper_tilt.moveTo(lastTilt*coeffTilt);   	
    }

     

    stepper_slider.run();
    stepper_pan.run();
    stepper_tilt.run();

  }
  else if (currentpos<255) // entre 11 et 254, on revient à la postition 1 (pour préparer une séquence)
  {
   gotoPosition(1);
   return;
  }
  else if (currentpos==255) // on lance la séquence
  {
    for (int i=1;i<11;i++)
    {
      setSpeed(dmxChannels[303+(5*i)]);
      gotoPosition(i);
      delay(100*dmxChannels[304+(5*i)]);
    }
    return;
  }


   if(dmxChannels[303]!=lastSpeed)
    {
      lastSpeed=dmxChannels[303];
      stepper_slider.setMaxSpeed(lastSpeed * coeffSpeedPosX );
      stepper_pan.setMaxSpeed(lastSpeed * coeffSpeedPan );
      stepper_tilt.setMaxSpeed(lastSpeed * coeffSpeedTilt );
      
    }

    if(dmxChannels[304]!=lastAccel)
    {
      lastAccel=dmxChannels[304];
      stepper_slider.setAcceleration(lastAccel * coeffAccelPosX );
      stepper_pan.setAcceleration(lastAccel * coeffAccelPan );
      stepper_tilt.setAcceleration(lastAccel * coeffAccelTilt );      
    }
  
   
  


        
    // Serial.println("Tableau des positions, pan, tilt, delay speed");    
    // for(int i=-1;i<10;i++)
    // {
    //   Serial.print("Position ");
    //   Serial.print(i) ;
    //   Serial.print(" : x =");
    //   Serial.print(dmxChannels[305+(5*i)]) ;
    //   Serial.print(" | pan =");
    //   Serial.print(dmxChannels[306+(5*i)]) ;
    //   Serial.print(" | tilt =");
    //   Serial.print(dmxChannels[307+(5*i)]) ;
    //   Serial.print(" | speed =");
    //   Serial.print(dmxChannels[308+(5*i)]) ;
    //   Serial.print(" | delay =");
    //   Serial.print(dmxChannels[309+(5*i)]) ;
    //   Serial.println(" ");
    // }
    // delay(1000);

  //  Serial.print("position = ");
  // Serial.println(stepper_slider.currentPosition());
  // digitalWrite(D5,HIGH);    
  // for(int i=0;i<5000;i++)
  // {
  //   digitalWrite(D6,HIGH);
  //   delayMicroseconds(200); 
  //   digitalWrite(D6,LOW);
  //   delayMicroseconds(200); 
  // }
  // delay(500);
  // digitalWrite(D5,LOW);    
  // for(int i=0;i<500;i++)
  // {
  //   digitalWrite(D6,HIGH);
  //   delayMicroseconds(500); 
  //   digitalWrite(D6,LOW);
  //   delayMicroseconds(500); 
  // }
  // delay(500);
  
  
//    display.clear();
// //   // draw the current demo method


//    display.setFont(ArialMT_Plain_10);
//    display.setTextAlignment(TEXT_ALIGN_LEFT);
   
//   // display.drawString(1,1, String(dmxChannels[302]));
//    display.drawString(0,0, "position = ");
//    display.drawString(10,10, String(stepper_slider.currentPosition()));
// //   // write the buffer to the display
//    display.display();
//    delay(10);

}

