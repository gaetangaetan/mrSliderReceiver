#define FIRMWARE_VERSION 142
#define DEBUG_ENABLE false

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

#include <ArtnetWifi.h>
WiFiUDP UdpSend;
ArtnetWifi artnet;

#define EEPROM_SIZE 32
#define DMXMODE true
#define ARTNETMODE false

#define CHANNEL_POS_SLIDER 298
#define CHANNEL_MODE_SLIDER 299

#define CHANNEL_POSX 300
#define CHANNEL_PAN 301
#define CHANNEL_TILT 302
#define CHANNEL_SPEED_SLIDER 303
#define CHANNEL_SPEED_PAN 304
#define CHANNEL_SPEED_TILT 305
#define CHANNEL_DELAY 306

#define NB_CHANNELS 7

#define CHANNEL_OFFSET_PAN 509
#define CHANNEL_OFFSET_TILT 510
#define CHANNEL_ACCELERATION 511
#define CHANNEL_UPDATE_SLIDER 512

bool runningMode = DMXMODE;

int lastposX= 0;
int lastPan=0;
int lastTilt=0;
int lastSpeedSlider=0;
int lastSpeedPan=0;
int lastSpeedTilt=0;
int lastOffsetPan=0;
int lastOffsetTilt=0;
int lastAccel=0;
unsigned long lastPositionTime=0;

bool newPosition = true;

int coeffPosX = 200;
int coeffPan = 100;
int coeffTilt = 25;

int coeffSpeedPosX = 100;
int coeffSpeedPan = 80;
int coeffSpeedTilt = 20;


int coeffAccelPosX = 100;
int coeffAccelPan = 50;
int coeffAccelTilt = 50;

int position = 0;
int seqPosition = 0;

unsigned long remainingWaitTime = 0;
bool sequenceStart = true;


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

uint8_t dmxChannels[513];

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
    dmxChannels[(packetNumber*128)+i+1]=incomingDMXPacket.dmxvalues[i];
  }
 
}



double mrdoublemodulo(double nombre, double diviseur)
{
  while(nombre<0)nombre+=diviseur;
  while(nombre>=diviseur)nombre-=diviseur;
  return nombre;
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
  Serial.print("FIRMWARE VERSION : ");
  Serial.println(FIRMWARE_VERSION);




// initialisation des pins du step motor driver
  pinMode(PIN_SLIDER_STEP, OUTPUT);
  pinMode(PIN_SLIDER_DIR, OUTPUT);
  pinMode(PIN_PAN_STEP, OUTPUT);
  pinMode(PIN_PAN_DIR, OUTPUT);
  pinMode(PIN_TILT_STEP, OUTPUT);
  pinMode(PIN_TILT_DIR, OUTPUT);

 
  
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
  
  // stepper_tilt.setMaxSpeed(200);
  // stepper_tilt.moveTo(10000);



}


void setSpeed(int speed)
{
      stepper_slider.setMaxSpeed(speed * coeffSpeedPosX );
      stepper_pan.setMaxSpeed(speed * coeffSpeedPan );
      stepper_tilt.setMaxSpeed(speed * coeffSpeedTilt );
}

void setSpeedAll(int speedSlider, int speedPan, int speedTilt)
{
      stepper_slider.setMaxSpeed(speedSlider * coeffSpeedPosX );
      stepper_pan.setMaxSpeed(speedPan * coeffSpeedPan );
      stepper_tilt.setMaxSpeed(speedTilt * coeffSpeedTilt );
}

void debug(const char debug_txt[], int debug_var)
{
  if(DEBUG_ENABLE)
  {
    Serial.print(debug_txt);
    Serial.print(" = ");
    Serial.println(debug_var);
  }
  else return;

}

void loop() {
  if(dmxChannels[CHANNEL_UPDATE_SLIDER]==255)
  {
    dmxChannels[CHANNEL_UPDATE_SLIDER]=0;
      // update automatique à chaque allumage (pendant la conception)
  
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
    ESP.restart();
    // fin update firmware (à retirer quand le code sera bon)
    
  }

//test pour voir si l'update fonctionne toujours
      //  stepper_tilt.run();



 

  int mode_slider = dmxChannels[CHANNEL_MODE_SLIDER];
  



  if(mode_slider==0)
  {
    position =  dmxChannels[CHANNEL_POS_SLIDER];
    sequenceStart=true; // au prochain lancement d'une séquence, il faudra initialiser lastPositionTime
    
    
       if(dmxChannels[CHANNEL_SPEED_SLIDER+(NB_CHANNELS*position)]!=lastSpeedSlider)
    {
      lastSpeedSlider=dmxChannels[CHANNEL_SPEED_SLIDER+(NB_CHANNELS*position)];
      stepper_slider.setMaxSpeed(lastSpeedSlider * coeffSpeedPosX );
      
    }
    
     if(dmxChannels[CHANNEL_SPEED_PAN+(NB_CHANNELS*position)]!=lastSpeedPan)
    {
      lastSpeedPan=dmxChannels[CHANNEL_SPEED_PAN+(NB_CHANNELS*position)];
      stepper_pan.setMaxSpeed(lastSpeedPan * coeffSpeedPan );
      
    }
     if(dmxChannels[CHANNEL_SPEED_TILT+(NB_CHANNELS*position)]!=lastSpeedTilt)
    {
      lastSpeedTilt=dmxChannels[CHANNEL_SPEED_TILT+(NB_CHANNELS*position)];
      stepper_tilt.setMaxSpeed(lastSpeedTilt * coeffSpeedTilt );
      
    }



    if(dmxChannels[CHANNEL_ACCELERATION]!=lastAccel)
    {
      lastAccel=dmxChannels[CHANNEL_ACCELERATION];
      stepper_slider.setAcceleration(lastAccel * coeffAccelPosX );
      stepper_pan.setAcceleration(lastAccel * coeffAccelPan );
      stepper_tilt.setAcceleration(lastAccel * coeffAccelTilt );      
    }
  
    if(dmxChannels[CHANNEL_POSX+(NB_CHANNELS*position)]!=lastposX)
    {
      lastposX=dmxChannels[CHANNEL_POSX+(NB_CHANNELS*position)];
      stepper_slider.moveTo(lastposX*coeffPosX);   	
    }
    
     if(dmxChannels[CHANNEL_OFFSET_PAN]!=lastOffsetPan)
    {
      lastOffsetPan=dmxChannels[CHANNEL_OFFSET_PAN];     
      stepper_pan.moveTo((lastPan-lastOffsetPan)*coeffPan);   	
    }

    if(dmxChannels[CHANNEL_PAN+(NB_CHANNELS*position)]!=lastPan)
    {
      lastPan=dmxChannels[CHANNEL_PAN+(NB_CHANNELS*position)];
      stepper_pan.moveTo((lastPan-lastOffsetPan)*coeffPan);   	
    }
    
   

    if(dmxChannels[CHANNEL_OFFSET_TILT]!=lastOffsetTilt)
    {
      lastOffsetTilt=dmxChannels[CHANNEL_OFFSET_TILT];     
      stepper_tilt.moveTo((lastTilt-lastOffsetTilt)*coeffTilt);   	
    }

    if(dmxChannels[CHANNEL_TILT+(NB_CHANNELS*position)]!=lastTilt)
    {
      lastTilt=dmxChannels[CHANNEL_TILT+(NB_CHANNELS*position)];
      stepper_tilt.moveTo((lastTilt-lastOffsetTilt)*coeffTilt);   	
    }

     

    stepper_slider.run();
    stepper_pan.run();
    stepper_tilt.run();

    

  }
  else if (mode_slider>0) // on lance la séquence
  {
    if(sequenceStart) // au lancement de la séquence, on initialise lastPositionTime
    {
      newPosition=true;
      lastPositionTime=millis();      
      sequenceStart = false;
      debug("position", position);
    }

    if(newPosition)
    {
      if((position<0) || (position>28))position=0;

      lastSpeedSlider = dmxChannels[CHANNEL_SPEED_SLIDER + (NB_CHANNELS * position)];
      lastSpeedPan = dmxChannels[CHANNEL_SPEED_PAN + (NB_CHANNELS * position)];
      lastSpeedTilt = dmxChannels[CHANNEL_SPEED_TILT + (NB_CHANNELS * position)];
      
      stepper_slider.setMaxSpeed(lastSpeedSlider * coeffSpeedPosX );
      stepper_pan.setMaxSpeed(lastSpeedPan * coeffSpeedPan );
      stepper_tilt.setMaxSpeed(lastSpeedTilt * coeffSpeedTilt );

      stepper_slider.moveTo(dmxChannels[CHANNEL_POSX + (NB_CHANNELS * position)] * coeffPosX);
      stepper_pan.moveTo((dmxChannels[CHANNEL_PAN + (NB_CHANNELS * position)]-lastOffsetPan) * coeffPan);
      stepper_tilt.moveTo((dmxChannels[CHANNEL_TILT + (NB_CHANNELS * position)]-lastOffsetTilt) * coeffTilt);

      newPosition=false;
    }
    
    bool srun = stepper_slider.run();
    bool prun = stepper_pan.run();
    bool trun = stepper_tilt.run();

      if(!srun && !prun && !trun) // quand les moteurs ont tous atteints leur cible, on passe à la postition suivante dans la séquence
      {
        
        //delay(1000);
        unsigned long elapsedTime = millis()-lastPositionTime;
        unsigned long totalTime = 100 * dmxChannels[CHANNEL_DELAY + (NB_CHANNELS * position)];
        debug("elapsed time",elapsedTime);
        debug("total time",totalTime);

        if(totalTime>elapsedTime)
        {
          remainingWaitTime=totalTime-elapsedTime;
          debug("remainingwaittime",remainingWaitTime);
          delay(remainingWaitTime);
        }
        else
        {
          debug("remainingwaittime negatif",0);
        }
        position++;
        newPosition=true;
        lastPositionTime=millis();
        debug("position", position);
      }
      
  }


}

