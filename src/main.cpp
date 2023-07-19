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
#include <ArtnetWifi.h>
WiFiUDP UdpSend;
ArtnetWifi artnet;

#define EEPROM_SIZE 32

#include <FastLED.h>
#include "OneButton.h"

#include <ESP8266WiFiMulti.h>
#include <espnow.h>

void setup() {
Serial.begin(57600) ;
}

void loop() {
  Serial.println("x 10");
  delay(4000);
  Serial.println("x -20");
  delay(4000);
}

