/*
   Exemple : Lien "Portail WiFiManager" arrête le serveur principal,
   lance wifiManager.startConfigPortal("MrSlider_AP"), puis relance le serveur
*/

#define FIRMWARE_VERSION 155
#define DEBUG_ENABLE true

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <AccelStepper.h>

// Pins
#define PIN_SLIDER_DIR   D0
#define PIN_SLIDER_STEP  D5
#define PIN_PAN_DIR      D3
#define PIN_PAN_STEP     D6
#define PIN_TILT_DIR     D4
#define PIN_TILT_STEP    D7

AccelStepper stepper_slider(AccelStepper::DRIVER, PIN_SLIDER_STEP, PIN_SLIDER_DIR);
AccelStepper stepper_pan   (AccelStepper::DRIVER, PIN_PAN_STEP,  PIN_PAN_DIR);
AccelStepper stepper_tilt  (AccelStepper::DRIVER, PIN_TILT_STEP, PIN_TILT_DIR);

int posx         = 0;
int pan          = 0;
int tilt         = 0;
int speed_posx   = 10;
int speed_pan    = 10;
int speed_tilt   = 10;
int acceleration = 10;

String firmwareURL = "http://mrsliderfirmware.gaetanstreel.com/firmware.bin";

// Coeffs
int coeffPosX       = 200;
int coeffPan        = 100;
int coeffTilt       = 25;
int coeffSpeedPosX  = 100;
int coeffSpeedPan   = 80;
int coeffSpeedTilt  = 20;
int coeffAccelPosX  = 100;
int coeffAccelPan   = 50;
int coeffAccelTilt  = 50;

// WiFiManager en global
WiFiManager wifiManager;
ESP8266WebServer server(80);

void debug(const char* txt, int val)
{
  if (DEBUG_ENABLE) {
    Serial.print(txt);
    Serial.print(" = ");
    Serial.println(val);
  }
}
void debug(const char* txt, const String &val)
{
  if (DEBUG_ENABLE) {
    Serial.print(txt);
    Serial.print(" = ");
    Serial.println(val);
  }
}

void update_started() { Serial.println("HTTP update started"); }
void update_finished() { Serial.println("HTTP update finished"); }
void update_progress(int cur, int total) {
  Serial.printf("HTTP update progress: %d of %d\n", cur, total);
}
void update_error(int err) { Serial.printf("HTTP update error %d\n", err); }

void updateFirmware()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Pas de WiFi => updateFirmware annulé");
    return;
  }
  Serial.print("Update from: ");
  Serial.println(firmwareURL);

  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  WiFiClient client;
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, firmwareURL);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED: %s\n", ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

// Page HTML
String htmlForm()
{
  String page = "<!DOCTYPE html><html><head><meta charset='utf-8'/>";
  page += "<title>MrSlider</title></head><body>";
  
  page += "<h1>MrSlider (FW v" + String(FIRMWARE_VERSION) + ")</h1>";
  page += "<form action='/setParams' method='GET'>";
  page += "<label>posx: </label><input type='number' name='posx' value='" + String(posx) + "'><br>";
  page += "<label>pan: </label><input type='number' name='pan' value='" + String(pan) + "'><br>";
  page += "<label>tilt: </label><input type='number' name='tilt' value='" + String(tilt) + "'><br>";
  page += "<label>vitesse_posx: </label><input type='number' name='speed_posx' value='" + String(speed_posx) + "'><br>";
  page += "<label>vitesse_pan: </label><input type='number' name='speed_pan' value='" + String(speed_pan) + "'><br>";
  page += "<label>vitesse_tilt: </label><input type='number' name='speed_tilt' value='" + String(speed_tilt) + "'><br>";
  page += "<label>acceleration: </label><input type='number' name='acceleration' value='" + String(acceleration) + "'><br><br>";

  page += "<label>Firmware URL : </label><input type='text' name='fwurl' size='60' value='" + firmwareURL + "'><br><br>";

  page += "<input type='submit' value='Appliquer'/>";
  page += "</form>";

  page += "<p><a href='/update_firmware'>Mise à jour du firmware</a></p>";
  page += "<p><a href='/portail_wifimanager'>Portail WiFiManager</a></p>";
  page += "</body></html>";
  return page;
}

void handleRoot() {
  server.send(200, "text/html", htmlForm());
}

void handleSetParams()
{
  if (server.hasArg("posx"))         posx         = server.arg("posx").toInt();
  if (server.hasArg("pan"))          pan          = server.arg("pan").toInt();
  if (server.hasArg("tilt"))         tilt         = server.arg("tilt").toInt();
  if (server.hasArg("speed_posx"))   speed_posx   = server.arg("speed_posx").toInt();
  if (server.hasArg("speed_pan"))    speed_pan    = server.arg("speed_pan").toInt();
  if (server.hasArg("speed_tilt"))   speed_tilt   = server.arg("speed_tilt").toInt();
  if (server.hasArg("acceleration")) acceleration = server.arg("acceleration").toInt();
  if (server.hasArg("fwurl"))        firmwareURL  = server.arg("fwurl");

  debug("posx", posx);
  debug("pan", pan);
  debug("tilt", tilt);
  debug("speed_posx", speed_posx);
  debug("speed_pan", speed_pan);
  debug("speed_tilt", speed_tilt);
  debug("acceleration", acceleration);
  debug("firmwareURL", firmwareURL);

  stepper_slider.moveTo(posx * coeffPosX);
  stepper_pan.moveTo(pan * coeffPan);
  stepper_tilt.moveTo(tilt * coeffTilt);

  stepper_slider.setMaxSpeed(max(1, speed_posx) * coeffSpeedPosX);
  stepper_pan.setMaxSpeed(max(1, speed_pan)    * coeffSpeedPan);
  stepper_tilt.setMaxSpeed(max(1, speed_tilt)  * coeffSpeedTilt);

  stepper_slider.setAcceleration(acceleration * coeffAccelPosX);
  stepper_pan.setAcceleration(acceleration     * coeffAccelPan);
  stepper_tilt.setAcceleration(acceleration    * coeffAccelTilt);

  server.send(200, "text/html", "<h2>Paramètres mis à jour</h2><a href='/'>Retour</a>");
}

void handleUpdateFirmware()
{
  server.send(200, "text/html", "<h2>Mise à jour en cours...</h2><p>Voir Serial</p>");
  updateFirmware();
}

// ICI on arrête le serveur, puis on lance startConfigPortal
void handlePortalWiFiManager()
{
  server.send(200, "text/html",
    "<h2>Passage en mode AP...</h2>"
    "<p>Veuillez vous connecter au wifi 'MrSlider_AP' et saisir 192.168.4.1</p>");

  delay(1000);

  // 1) Arrêter le serveur
  server.stop();

  // 2) Lancer le portail
  wifiManager.startConfigPortal("MrSlider_AP");

  // 3) On revient ici après la config ou timeout
  // On relance le serveur
  server.begin();
  Serial.println("Serveur principal relancé après configPortal");
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.printf("FIRMWARE VERSION : %d\n", FIRMWARE_VERSION);

  pinMode(PIN_SLIDER_DIR,  OUTPUT);
  pinMode(PIN_SLIDER_STEP, OUTPUT);
  pinMode(PIN_PAN_DIR,     OUTPUT);
  pinMode(PIN_PAN_STEP,    OUTPUT);
  pinMode(PIN_TILT_DIR,    OUTPUT);
  pinMode(PIN_TILT_STEP,   OUTPUT);

  stepper_slider.setAcceleration(2000);
  stepper_slider.setMaxSpeed(2000);
  stepper_pan.setAcceleration(2000);
  stepper_pan.setMaxSpeed(2000);
  stepper_tilt.setAcceleration(2000);
  stepper_tilt.setMaxSpeed(2000);

  // Connexion WiFi via WiFiManager
  wifiManager.autoConnect("MrSlider_AP");

  Serial.print("Connecté au WiFi => ");
  Serial.println(WiFi.localIP());

  // mDNS
  if (MDNS.begin("mrslider")) {
    Serial.println("mDNS => http://mrslider.local");
  }

  server.on("/",                HTTP_GET, handleRoot);
  server.on("/setParams",       HTTP_GET, handleSetParams);
  server.on("/update_firmware", HTTP_GET, handleUpdateFirmware);
  server.on("/portail_wifimanager", HTTP_GET, handlePortalWiFiManager);

  server.onNotFound([](){
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Serveur principal démarré");
}

void loop()
{
  server.handleClient();
  MDNS.update();

  stepper_slider.run();
  stepper_pan.run();
  stepper_tilt.run();
}
