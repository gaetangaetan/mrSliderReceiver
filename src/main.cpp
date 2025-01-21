/*
    Exemple d'un serveur web sur ESP8266 :
    - Accessible via "http://mrslider.local"
    - Formulaire pour configurer 7 paramètres (posx, pan, tilt, vitesses, accel)
    - Route "/update_firmware" pour télécharger et flasher un nouveau firmware

    Nécessite:
      - tzapu/WiFiManager
      - AccelStepper
      - ESP8266mDNS (inclus dans le core ESP8266)
      - ESP8266HTTPUpdate
*/

#define FIRMWARE_VERSION 149
#define DEBUG_ENABLE true

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <AccelStepper.h>

// Pins des moteurs
#define PIN_SLIDER_DIR   D0
#define PIN_SLIDER_STEP  D5
#define PIN_PAN_DIR      D3
#define PIN_PAN_STEP     D6
#define PIN_TILT_DIR     D4
#define PIN_TILT_STEP    D7

// Objets AccelStepper
AccelStepper stepper_slider(AccelStepper::DRIVER, PIN_SLIDER_STEP, PIN_SLIDER_DIR);
AccelStepper stepper_pan   (AccelStepper::DRIVER, PIN_PAN_STEP,  PIN_PAN_DIR);
AccelStepper stepper_tilt  (AccelStepper::DRIVER, PIN_TILT_STEP, PIN_TILT_DIR);

// Paramètres globaux
int posx         = 0;
int pan          = 0;
int tilt         = 0;
int speed_posx   = 10;
int speed_pan    = 10;
int speed_tilt   = 10;
int acceleration = 10;

// Coefficients de conversion
int coeffPosX       = 200;
int coeffPan        = 100;
int coeffTilt       = 25;
int coeffSpeedPosX  = 100;
int coeffSpeedPan   = 80;
int coeffSpeedTilt  = 20;
int coeffAccelPosX  = 100;
int coeffAccelPan   = 50;
int coeffAccelTilt  = 50;

// Création du serveur web
ESP8266WebServer server(80);

//=== Fonctions de debug ===
void debug(const char* txt, int val)
{
  if (DEBUG_ENABLE) {
    Serial.print(txt);
    Serial.print(" = ");
    Serial.println(val);
  }
}

//=== Callbacks de mise à jour ===
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

//=== Fonction updateFirmware() ===
void updateFirmware()
{
  // (Ré)essayer de se connecter sur "mrVOOlpy" / "youhououhou"
  // Si vous préférez réutiliser la connexion WiFi actuelle,
  // commentez ces lignes.
  WiFi.mode(WIFI_STA);
  WiFi.begin("mrVOOlpy", "youhououhou");

  Serial.println("Tentative de connexion WiFi pour update firmware...");
  int tentatives = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print('.');
    if (++tentatives > 20) {
      Serial.println("Échec de connexion WiFi. Abandon de l'update.");
      return;
    }
  }
  Serial.println("\nWiFi connecté.");

  // Configuration de l'update
  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  // Exemple : on utilise le firmware "http://mrsliderfirmware.gaetanstreel.com/firmware.binXXX"
  String firmwareURL = "http://mrsliderfirmware.gaetanstreel.com/firmware.bin";
  firmwareURL += (FIRMWARE_VERSION + 1);

  Serial.print("URL Firmware: ");
  Serial.println(firmwareURL);

  WiFiClient client;
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, firmwareURL.c_str());
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n",
                    ESPhttpUpdate.getLastError(),
                    ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      // L’ESP redémarre automatiquement après update OK
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

//=== Page HTML du formulaire ===
String htmlForm()
{
  String page = "<!DOCTYPE html><html><head><meta charset='utf-8'/>";
  page += "<title>MrSlider - Paramètres</title></head><body>";
  page += "<h1>Configuration Slider (FW v";
  page += FIRMWARE_VERSION;
  page += ")</h1>";
  page += "<form action='/setParams' method='GET'>";

  page += "<label>posx: </label><input type='number' name='posx' value='" + String(posx) + "'><br><br>";
  page += "<label>pan: </label><input type='number' name='pan' value='" + String(pan) + "'><br><br>";
  page += "<label>tilt: </label><input type='number' name='tilt' value='" + String(tilt) + "'><br><br>";
  page += "<label>vitesse_posx: </label><input type='number' name='speed_posx' value='" + String(speed_posx) + "'><br><br>";
  page += "<label>vitesse_pan: </label><input type='number' name='speed_pan' value='" + String(speed_pan) + "'><br><br>";
  page += "<label>vitesse_tilt: </label><input type='number' name='speed_tilt' value='" + String(speed_tilt) + "'><br><br>";
  page += "<label>acceleration: </label><input type='number' name='acceleration' value='" + String(acceleration) + "'><br><br>";

  page += "<input type='submit' value='Appliquer'/>";
  page += "</form><br>";
  page += "<p><a href='/update_firmware'>Mise à jour du firmware</a></p>";
  page += "</body></html>";
  return page;
}

//=== Handler page racine "/" ===
void handleRoot()
{
  server.send(200, "text/html", htmlForm());
}

//=== Handler soumission formulaire "/setParams" ===
void handleSetParams()
{
  if (server.hasArg("posx"))         posx         = server.arg("posx").toInt();
  if (server.hasArg("pan"))          pan          = server.arg("pan").toInt();
  if (server.hasArg("tilt"))         tilt         = server.arg("tilt").toInt();
  if (server.hasArg("speed_posx"))   speed_posx   = server.arg("speed_posx").toInt();
  if (server.hasArg("speed_pan"))    speed_pan    = server.arg("speed_pan").toInt();
  if (server.hasArg("speed_tilt"))   speed_tilt   = server.arg("speed_tilt").toInt();
  if (server.hasArg("acceleration")) acceleration = server.arg("acceleration").toInt();

  debug("posx", posx);
  debug("pan", pan);
  debug("tilt", tilt);
  debug("speed_posx", speed_posx);
  debug("speed_pan", speed_pan);
  debug("speed_tilt", speed_tilt);
  debug("acceleration", acceleration);

  // Mise à jour des stepper
  stepper_slider.moveTo(posx * coeffPosX);
  stepper_pan.moveTo(pan * coeffPan);
  stepper_tilt.moveTo(tilt * coeffTilt);

  stepper_slider.setMaxSpeed(max(1, speed_posx) * coeffSpeedPosX);
  stepper_pan.setMaxSpeed(max(1, speed_pan)    * coeffSpeedPan);
  stepper_tilt.setMaxSpeed(max(1, speed_tilt)  * coeffSpeedTilt);

  stepper_slider.setAcceleration(acceleration * coeffAccelPosX);
  stepper_pan.setAcceleration(acceleration     * coeffAccelPan);
  stepper_tilt.setAcceleration(acceleration    * coeffAccelTilt);

  String message = "<!DOCTYPE html><html><head><meta charset='utf-8'/>";
  message += "<title>MrSlider - Paramètres appliqués</title></head><body>";
  message += "<h2>Paramètres mis à jour !</h2>";
  message += "<p><a href='/'>Retour</a></p>";
  message += "</body></html>";

  server.send(200, "text/html", message);
}

//=== Handler "/update_firmware" ===
void handleUpdateFirmware()
{
  // Retour HTTP immédiat (optionnel). 
  // Vous pouvez aussi afficher "Mise à jour en cours..." puis faire la mise à jour.
  server.send(200, "text/html", "<h2>Mise à jour en cours... (voir Serial)</h2>");

  // Lancement de la mise à jour
  updateFirmware();

  // Après le flash, si ça réussit, l'ESP redémarrera de lui-même. 
  // Sinon, il reste sur l'ancienne version.
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("FIRMWARE VERSION : ");
  Serial.println(FIRMWARE_VERSION);

  // Pins en sortie
  pinMode(PIN_SLIDER_DIR,  OUTPUT);
  pinMode(PIN_SLIDER_STEP, OUTPUT);
  pinMode(PIN_PAN_DIR,     OUTPUT);
  pinMode(PIN_PAN_STEP,    OUTPUT);
  pinMode(PIN_TILT_DIR,    OUTPUT);
  pinMode(PIN_TILT_STEP,   OUTPUT);

  // Config moteur
  stepper_slider.setAcceleration(2000);
  stepper_slider.setMaxSpeed(2000);
  stepper_pan.setAcceleration(2000);
  stepper_pan.setMaxSpeed(2000);
  stepper_tilt.setAcceleration(2000);
  stepper_tilt.setMaxSpeed(2000);

  // WiFiManager : connexion ou AP si pas d'ID stocké
  WiFiManager wifiManager;
  // wifiManager.resetSettings(); // Décommentez pour effacer la config WiFi enregistrée
  wifiManager.autoConnect("MrSlider_AP");

  Serial.print("Connecté au WiFi. IP: ");
  Serial.println(WiFi.localIP());

  // Activer mDNS
  if (MDNS.begin("mrslider")) {
    Serial.println("mDNS responder started => http://mrslider.local");
  } else {
    Serial.println("Erreur initialisation mDNS");
  }

  // Routes du serveur
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/setParams",HTTP_GET, handleSetParams);
  server.on("/update_firmware", HTTP_GET, handleUpdateFirmware);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Serveur web démarré sur le port 80");
}

void loop()
{
  server.handleClient();
  MDNS.update();

  // Faire tourner les stepper
  stepper_slider.run();
  stepper_pan.run();
  stepper_tilt.run();
}
