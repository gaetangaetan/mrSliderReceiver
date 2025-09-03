/*
   Exemple : Lien "Portail WiFiManager" arrête le serveur principal,
   lance wifiManager.startConfigPortal("MrSlider_AP"), puis relance le serveur
*/

#define FIRMWARE_VERSION 162
#define DEBUG_ENABLE true

// Limites des axes
#define POSX_MIN 0
#define POSX_MAX 100
#define PAN_MIN -50
#define PAN_MAX 50
#define TILT_MIN -50
#define TILT_MAX 50
#define SPEED_MIN 1
#define SPEED_MAX 100
#define ACCEL_MIN 1
#define ACCEL_MAX 100

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <AccelStepper.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

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
String littlefsURL = "http://mrsliderfirmware.gaetanstreel.com/littlefs.bin";

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

// Structure pour les presets
struct Preset {
  int posx;
  int pan;
  int tilt;
  int speed_posx;
  int speed_pan;
  int speed_tilt;
  int acceleration;
};

// Structure pour les limites configurables
struct ConfigLimits {
  int posx_min;
  int posx_max;
  int pan_min;
  int pan_max;
  int tilt_min;
  int tilt_max;
  int speed_min;
  int speed_max;
  int accel_min;
  int accel_max;
};

// Variables globales pour les limites configurables
ConfigLimits limits = {
  POSX_MIN, POSX_MAX,    // posx: 0, 100
  PAN_MIN, PAN_MAX,      // pan: -50, 50
  TILT_MIN, TILT_MAX,    // tilt: -50, 50
  SPEED_MIN, SPEED_MAX,  // speed: 1, 100
  ACCEL_MIN, ACCEL_MAX   // accel: 1, 100
};

// WiFiManager en global
WiFiManager wifiManager;
ESP8266WebServer server(80);

// Variables pour la mise à jour temps réel
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // 100ms

// Variables pour le mode automatique
bool autoMode = false;
int presetA = 1;
int presetB = 2;
int autoDelay = 5; // secondes
unsigned long lastAutoSwitch = 0;
bool currentPresetIsA = true;

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

// Fonctions pour la gestion des presets
bool savePreset(int slot, const Preset& preset) {
  if (slot < 1 || slot > 6) return false;
  
  String filename = "/preset" + String(slot) + ".json";
  File file = LittleFS.open(filename, "w");
  if (!file) return false;
  
  DynamicJsonDocument doc(256);
  doc["posx"] = preset.posx;
  doc["pan"] = preset.pan;
  doc["tilt"] = preset.tilt;
  doc["speed_posx"] = preset.speed_posx;
  doc["speed_pan"] = preset.speed_pan;
  doc["speed_tilt"] = preset.speed_tilt;
  doc["acceleration"] = preset.acceleration;
  
  serializeJson(doc, file);
  file.close();
  return true;
}

bool loadPreset(int slot, Preset& preset) {
  if (slot < 1 || slot > 6) return false;
  
  String filename = "/preset" + String(slot) + ".json";
  if (!LittleFS.exists(filename)) return false;
  
  File file = LittleFS.open(filename, "r");
  if (!file) return false;
  
  DynamicJsonDocument doc(256);
  deserializeJson(doc, file);
  file.close();
  
  preset.posx = doc["posx"];
  preset.pan = doc["pan"];
  preset.tilt = doc["tilt"];
  preset.speed_posx = doc["speed_posx"];
  preset.speed_pan = doc["speed_pan"];
  preset.speed_tilt = doc["speed_tilt"];
  preset.acceleration = doc["acceleration"];
  
  return true;
}

// Fonctions pour la gestion de la configuration des limites
bool saveConfig(const ConfigLimits& config) {
  File file = LittleFS.open("/config.json", "w");
  if (!file) return false;
  
  DynamicJsonDocument doc(512);
  doc["posx_min"] = config.posx_min;
  doc["posx_max"] = config.posx_max;
  doc["pan_min"] = config.pan_min;
  doc["pan_max"] = config.pan_max;
  doc["tilt_min"] = config.tilt_min;
  doc["tilt_max"] = config.tilt_max;
  doc["speed_min"] = config.speed_min;
  doc["speed_max"] = config.speed_max;
  doc["accel_min"] = config.accel_min;
  doc["accel_max"] = config.accel_max;
  
  serializeJson(doc, file);
  file.close();
  return true;
}

bool loadConfig(ConfigLimits& config) {
  if (!LittleFS.exists("/config.json")) return false;
  
  File file = LittleFS.open("/config.json", "r");
  if (!file) return false;
  
  DynamicJsonDocument doc(512);
  deserializeJson(doc, file);
  file.close();
  
  config.posx_min = doc["posx_min"];
  config.posx_max = doc["posx_max"];
  config.pan_min = doc["pan_min"];
  config.pan_max = doc["pan_max"];
  config.tilt_min = doc["tilt_min"];
  config.tilt_max = doc["tilt_max"];
  config.speed_min = doc["speed_min"];
  config.speed_max = doc["speed_max"];
  config.accel_min = doc["accel_min"];
  config.accel_max = doc["accel_max"];
  
  return true;
}

void updateSteppers() {
  stepper_slider.moveTo(posx * coeffPosX);
  stepper_pan.moveTo(pan * coeffPan);
  stepper_tilt.moveTo(tilt * coeffTilt);

  stepper_slider.setMaxSpeed(max(1, speed_posx) * coeffSpeedPosX);
  stepper_pan.setMaxSpeed(max(1, speed_pan) * coeffSpeedPan);
  stepper_tilt.setMaxSpeed(max(1, speed_tilt) * coeffSpeedTilt);

  stepper_slider.setAcceleration(acceleration * coeffAccelPosX);
  stepper_pan.setAcceleration(acceleration * coeffAccelPan);
  stepper_tilt.setAcceleration(acceleration * coeffAccelTilt);
}

void applyPreset(const Preset& preset) {
  posx = preset.posx;
  pan = preset.pan;
  tilt = preset.tilt;
  speed_posx = preset.speed_posx;
  speed_pan = preset.speed_pan;
  speed_tilt = preset.speed_tilt;
  acceleration = preset.acceleration;
  
  updateSteppers();
}

// Fonction pour gérer le mode automatique
void handleAutoMode() {
  if (!autoMode) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastAutoSwitch >= (autoDelay * 1000)) {
    // Il est temps de changer de preset
    Preset preset;
    int targetSlot = currentPresetIsA ? presetB : presetA;
    
    if (loadPreset(targetSlot, preset)) {
      applyPreset(preset);
      currentPresetIsA = !currentPresetIsA;
      lastAutoSwitch = currentTime;
      
      debug("Mode auto: passage au preset", targetSlot);
    } else {
      // Si le preset n'existe pas, désactiver le mode auto
      autoMode = false;
      debug("Erreur preset", targetSlot);
      Serial.println(" - Mode automatique désactivé");
    }
  }
}

void updateFirmware()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Pas de WiFi => updateFirmware annulé");
    return;
  }
  Serial.print("Update firmware from: ");
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

void updateLittleFS()
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Pas de WiFi => updateLittleFS annulé");
    return;
  }
  Serial.print("Update LittleFS from: ");
  Serial.println(littlefsURL);

  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  WiFiClient client;
  t_httpUpdate_return ret = ESPhttpUpdate.updateFS(client, littlefsURL);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("LittleFS UPDATE_FAILED: %s\n", ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("LittleFS UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("LittleFS UPDATE_OK");
      break;
  }
}

// Fonction pour servir les fichiers statiques
void handleStaticFile(String path, String contentType) {
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "File not found");
  }
}

void handleRoot() {
  handleStaticFile("/index.html", "text/html");
}

void handleCSS() {
  handleStaticFile("/style.css", "text/css");
}

void handleJS() {
  handleStaticFile("/script.js", "application/javascript");
}

void handleConfig() {
  handleStaticFile("/config.html", "text/html");
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

  updateSteppers();

  server.send(200, "text/html", "<h2>Paramètres mis à jour</h2><a href='/'>Retour</a>");
}

// Nouvel endpoint pour mise à jour API
void handleAPIUpdate() {
  if (server.hasArg("posx"))         posx         = constrain(server.arg("posx").toInt(), limits.posx_min, limits.posx_max);
  if (server.hasArg("pan"))          pan          = constrain(server.arg("pan").toInt(), limits.pan_min, limits.pan_max);
  if (server.hasArg("tilt"))         tilt         = constrain(server.arg("tilt").toInt(), limits.tilt_min, limits.tilt_max);
  if (server.hasArg("speed_posx"))   speed_posx   = constrain(server.arg("speed_posx").toInt(), limits.speed_min, limits.speed_max);
  if (server.hasArg("speed_pan"))    speed_pan    = constrain(server.arg("speed_pan").toInt(), limits.speed_min, limits.speed_max);
  if (server.hasArg("speed_tilt"))   speed_tilt   = constrain(server.arg("speed_tilt").toInt(), limits.speed_min, limits.speed_max);
  if (server.hasArg("acceleration")) acceleration = constrain(server.arg("acceleration").toInt(), limits.accel_min, limits.accel_max);

  updateSteppers();
  
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

// Endpoint pour obtenir l'état actuel
void handleAPIStatus() {
  DynamicJsonDocument doc(1024);
  doc["posx"] = posx;
  doc["pan"] = pan;
  doc["tilt"] = tilt;
  doc["speed_posx"] = speed_posx;
  doc["speed_pan"] = speed_pan;
  doc["speed_tilt"] = speed_tilt;
  doc["acceleration"] = acceleration;
  doc["firmware_version"] = FIRMWARE_VERSION;
  
  // Ajouter les limites actuelles
  JsonObject limitsObj = doc.createNestedObject("limits");
  limitsObj["posx_min"] = limits.posx_min;
  limitsObj["posx_max"] = limits.posx_max;
  limitsObj["pan_min"] = limits.pan_min;
  limitsObj["pan_max"] = limits.pan_max;
  limitsObj["tilt_min"] = limits.tilt_min;
  limitsObj["tilt_max"] = limits.tilt_max;
  limitsObj["speed_min"] = limits.speed_min;
  limitsObj["speed_max"] = limits.speed_max;
  limitsObj["accel_min"] = limits.accel_min;
  limitsObj["accel_max"] = limits.accel_max;
  
  // Ajouter les informations du mode automatique
  JsonObject autoObj = doc.createNestedObject("auto_mode");
  autoObj["enabled"] = autoMode;
  autoObj["preset_a"] = presetA;
  autoObj["preset_b"] = presetB;
  autoObj["delay"] = autoDelay;
  autoObj["current_preset_is_a"] = currentPresetIsA;
  if (autoMode) {
    unsigned long timeRemaining = (autoDelay * 1000) - (millis() - lastAutoSwitch);
    autoObj["time_remaining"] = max(0L, (long)timeRemaining);
  } else {
    autoObj["time_remaining"] = 0;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Endpoint pour obtenir la configuration des limites
void handleConfigGet() {
  DynamicJsonDocument doc(512);
  doc["posx_min"] = limits.posx_min;
  doc["posx_max"] = limits.posx_max;
  doc["pan_min"] = limits.pan_min;
  doc["pan_max"] = limits.pan_max;
  doc["tilt_min"] = limits.tilt_min;
  doc["tilt_max"] = limits.tilt_max;
  doc["speed_min"] = limits.speed_min;
  doc["speed_max"] = limits.speed_max;
  doc["accel_min"] = limits.accel_min;
  doc["accel_max"] = limits.accel_max;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// Endpoint pour sauvegarder la configuration des limites
void handleConfigSave() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  DynamicJsonDocument doc(512);
  deserializeJson(doc, server.arg("plain"));
  
  // Validation des valeurs
  ConfigLimits newLimits;
  newLimits.posx_min = doc["posx_min"];
  newLimits.posx_max = doc["posx_max"];
  newLimits.pan_min = doc["pan_min"];
  newLimits.pan_max = doc["pan_max"];
  newLimits.tilt_min = doc["tilt_min"];
  newLimits.tilt_max = doc["tilt_max"];
  newLimits.speed_min = doc["speed_min"];
  newLimits.speed_max = doc["speed_max"];
  newLimits.accel_min = doc["accel_min"];
  newLimits.accel_max = doc["accel_max"];
  
  // Vérifications de cohérence
  if (newLimits.posx_min >= newLimits.posx_max ||
      newLimits.pan_min >= newLimits.pan_max ||
      newLimits.tilt_min >= newLimits.tilt_max ||
      newLimits.speed_min >= newLimits.speed_max ||
      newLimits.accel_min >= newLimits.accel_max) {
    server.send(400, "application/json", "{\"error\":\"Invalid limits: min must be < max\"}");
    return;
  }
  
  // Sauvegarder la configuration
  if (saveConfig(newLimits)) {
    limits = newLimits; // Appliquer les nouvelles limites
    server.send(200, "application/json", "{\"status\":\"saved\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Save failed\"}");
  }
}

// Endpoint pour configurer le mode automatique
void handleAutoModeConfig() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  DynamicJsonDocument doc(512);
  deserializeJson(doc, server.arg("plain"));
  
  // Récupérer les paramètres
  bool newAutoMode = doc["enabled"];
  int newPresetA = doc["preset_a"];
  int newPresetB = doc["preset_b"];
  int newAutoDelay = doc["delay"];
  
  // Validation
  if (newPresetA < 1 || newPresetA > 6 || newPresetB < 1 || newPresetB > 6) {
    server.send(400, "application/json", "{\"error\":\"Invalid preset numbers (1-6)\"}");
    return;
  }
  
  if (newPresetA == newPresetB) {
    server.send(400, "application/json", "{\"error\":\"Preset A and B must be different\"}");
    return;
  }
  
  if (newAutoDelay < 1 || newAutoDelay > 3600) { // max 1 heure
    server.send(400, "application/json", "{\"error\":\"Delay must be between 1 and 3600 seconds\"}");
    return;
  }
  
  // Si on active le mode auto, vérifier que les presets existent
  if (newAutoMode) {
    Preset testPreset;
    if (!loadPreset(newPresetA, testPreset)) {
      server.send(400, "application/json", "{\"error\":\"Preset A does not exist\"}");
      return;
    }
    if (!loadPreset(newPresetB, testPreset)) {
      server.send(400, "application/json", "{\"error\":\"Preset B does not exist\"}");
      return;
    }
  }
  
  // Appliquer les nouveaux paramètres
  autoMode = newAutoMode;
  presetA = newPresetA;
  presetB = newPresetB;
  autoDelay = newAutoDelay;
  
  if (autoMode) {
    lastAutoSwitch = millis(); // Réinitialiser le timer
    currentPresetIsA = true;   // Commencer par le preset A
    
    // Charger immédiatement le preset A
    Preset preset;
    if (loadPreset(presetA, preset)) {
      applyPreset(preset);
    }
  }
  
  server.send(200, "application/json", "{\"status\":\"configured\"}");
}

// Endpoints GET pour contrôler le mode automatique depuis l'URL
void handleAutoModeOn() {
  if (presetA == presetB) {
    server.send(400, "text/html", "<h2>Erreur</h2><p>Les presets A et B doivent être différents</p><a href='/'>Retour</a>");
    return;
  }
  
  // Vérifier que les presets existent
  Preset testPreset;
  if (!loadPreset(presetA, testPreset)) {
    server.send(400, "text/html", "<h2>Erreur</h2><p>Le preset A (" + String(presetA) + ") n'existe pas</p><a href='/'>Retour</a>");
    return;
  }
  if (!loadPreset(presetB, testPreset)) {
    server.send(400, "text/html", "<h2>Erreur</h2><p>Le preset B (" + String(presetB) + ") n'existe pas</p><a href='/'>Retour</a>");
    return;
  }
  
  autoMode = true;
  lastAutoSwitch = millis();
  currentPresetIsA = true;
  
  // Charger immédiatement le preset A
  if (loadPreset(presetA, testPreset)) {
    applyPreset(testPreset);
  }
  
  server.send(200, "text/html", 
    "<h2>Mode automatique activé</h2>"
    "<p>Alternance entre les presets " + String(presetA) + " et " + String(presetB) + "</p>"
    "<p>Délai : " + String(autoDelay) + " secondes</p>"
    "<a href='/'>Retour à l'accueil</a>");
}

void handleAutoModeOff() {
  autoMode = false;
  
  server.send(200, "text/html", 
    "<h2>Mode automatique désactivé</h2>"
    "<p>Retour en mode manuel</p>"
    "<a href='/'>Retour à l'accueil</a>");
}

// Endpoint pour sauvegarder un preset
void handlePresetSave() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method not allowed");
    return;
  }
  
  DynamicJsonDocument doc(512);
  deserializeJson(doc, server.arg("plain"));
  
  int slot = doc["slot"];
  if (slot < 1 || slot > 6) {
    server.send(400, "application/json", "{\"error\":\"Invalid slot\"}");
    return;
  }
  
  Preset preset;
  preset.posx = doc["preset"]["posx"];
  preset.pan = doc["preset"]["pan"];
  preset.tilt = doc["preset"]["tilt"];
  preset.speed_posx = doc["preset"]["speed_posx"];
  preset.speed_pan = doc["preset"]["speed_pan"];
  preset.speed_tilt = doc["preset"]["speed_tilt"];
  preset.acceleration = doc["preset"]["acceleration"];
  
  if (savePreset(slot, preset)) {
    server.send(200, "application/json", "{\"status\":\"saved\"}");
  } else {
    server.send(500, "application/json", "{\"error\":\"Save failed\"}");
  }
}

// Endpoint pour charger un preset
void handlePresetLoad() {
  int slot = server.arg("slot").toInt();
  if (slot < 1 || slot > 6) {
    server.send(400, "application/json", "{\"error\":\"Invalid slot\"}");
    return;
  }
  
  Preset preset;
  if (loadPreset(slot, preset)) {
    applyPreset(preset);
    
    DynamicJsonDocument doc(512);
    doc["preset"]["posx"] = preset.posx;
    doc["preset"]["pan"] = preset.pan;
    doc["preset"]["tilt"] = preset.tilt;
    doc["preset"]["speed_posx"] = preset.speed_posx;
    doc["preset"]["speed_pan"] = preset.speed_pan;
    doc["preset"]["speed_tilt"] = preset.speed_tilt;
    doc["preset"]["acceleration"] = preset.acceleration;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  } else {
    server.send(404, "application/json", "{\"error\":\"Preset not found\"}");
  }
}

// Endpoint pour lister les presets existants
void handlePresetList() {
  DynamicJsonDocument doc(256);
  JsonArray presets = doc.createNestedArray("presets");
  
  for (int i = 1; i <= 6; i++) {
    String filename = "/preset" + String(i) + ".json";
    if (LittleFS.exists(filename)) {
      presets.add(i);
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleUpdateFirmware()
{
  server.send(200, "text/html", "<h2>Mise à jour en cours...</h2><p>Voir Serial</p>");
  updateFirmware();
}

// Endpoints GET pour lancer les mises à jour depuis le navigateur
void handleUploadFirmware()
{
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/html", "<h2>Erreur</h2><p>WiFi non connecté</p><a href='/'>Retour</a>");
    return;
  }
  
  server.send(200, "text/html", 
    "<h2>Mise à jour du firmware en cours...</h2>"
    "<p>Le firmware est en cours de téléchargement depuis :<br>"
    "<code>http://mrsliderfirmware.gaetanstreel.com/firmware.bin</code></p>"
    "<p>Suivez les logs sur le port série pour le progrès.</p>"
    "<p><strong>Attention :</strong> L'appareil va redémarrer automatiquement après la mise à jour.</p>"
    "<a href='/'>Retour à l'accueil</a>");
  
  updateFirmware();
}

void handleUploadLittleFS()
{
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/html", "<h2>Erreur</h2><p>WiFi non connecté</p><a href='/'>Retour</a>");
    return;
  }
  
  server.send(200, "text/html", 
    "<h2>Mise à jour de LittleFS en cours...</h2>"
    "<p>Le système de fichiers est en cours de téléchargement depuis :<br>"
    "<code>http://mrsliderfirmware.gaetanstreel.com/littlefs.bin</code></p>"
    "<p>Suivez les logs sur le port série pour le progrès.</p>"
    "<p><strong>Attention :</strong> L'appareil va redémarrer automatiquement après la mise à jour.</p>"
    "<a href='/'>Retour à l'accueil</a>");
  
  updateLittleFS();
}

void handleUploadComplete()
{
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/html", "<h2>Erreur</h2><p>WiFi non connecté</p><a href='/'>Retour</a>");
    return;
  }
  
  server.send(200, "text/html", 
    "<h2>Mise à jour complète en cours...</h2>"
    "<p>Mise à jour en 2 étapes :</p>"
    "<ol>"
    "<li>LittleFS : <code>http://mrsliderfirmware.gaetanstreel.com/littlefs.bin</code></li>"
    "<li>Firmware : <code>http://mrsliderfirmware.gaetanstreel.com/firmware.bin</code></li>"
    "</ol>"
    "<p>Suivez les logs sur le port série pour le progrès.</p>"
    "<p><strong>Attention :</strong> L'appareil va redémarrer automatiquement après chaque mise à jour.</p>"
    "<a href='/'>Retour à l'accueil</a>");
  
  Serial.println("=== Début de la mise à jour complète ===");
  updateLittleFS();
  delay(2000); // Pause entre les deux mises à jour
  updateFirmware();
}

// Endpoint POST pour l'upload OTA du firmware
void handleOTAFirmware() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "application/json", "{\"error\":\"WiFi not connected\"}");
    return;
  }
  
  // Répondre immédiatement avant de commencer la mise à jour
  server.send(200, "application/json", "{\"status\":\"firmware_update_started\",\"message\":\"Mise à jour du firmware en cours...\"}");
  
  // Lancer la mise à jour du firmware
  updateFirmware();
}

// Endpoint POST pour l'upload OTA de LittleFS
void handleOTALittleFS() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "application/json", "{\"error\":\"WiFi not connected\"}");
    return;
  }
  
  // Répondre immédiatement avant de commencer la mise à jour
  server.send(200, "application/json", "{\"status\":\"littlefs_update_started\",\"message\":\"Mise à jour de LittleFS en cours...\"}");
  
  // Lancer la mise à jour de LittleFS
  updateLittleFS();
}

// Endpoint POST pour l'upload OTA complet (firmware + LittleFS)
void handleOTAComplete() {
  if (server.method() != HTTP_POST) {
    server.send(405, "application/json", "{\"error\":\"Method not allowed\"}");
    return;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "application/json", "{\"error\":\"WiFi not connected\"}");
    return;
  }
  
  // Répondre immédiatement avant de commencer les mises à jour
  server.send(200, "application/json", "{\"status\":\"complete_update_started\",\"message\":\"Mise à jour complète en cours (LittleFS puis firmware)...\"}");
  
  // D'abord mettre à jour LittleFS, puis le firmware
  Serial.println("=== Début de la mise à jour complète ===");
  updateLittleFS();
  delay(2000); // Pause entre les deux mises à jour
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

  // Initialisation de LittleFS
  if (!LittleFS.begin()) {
    Serial.println("Erreur lors de l'initialisation de LittleFS");
  } else {
    Serial.println("LittleFS initialisé avec succès");
    
    // Charger la configuration des limites
    if (loadConfig(limits)) {
      Serial.println("Configuration des limites chargée");
    } else {
      Serial.println("Configuration par défaut utilisée");
    }
  }

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

  // Routes statiques
  server.on("/",                HTTP_GET, handleRoot);
  server.on("/style.css",       HTTP_GET, handleCSS);
  server.on("/script.js",       HTTP_GET, handleJS);
  server.on("/config",          HTTP_GET, handleConfig);
  
  // Routes API
  server.on("/api/update",      HTTP_GET, handleAPIUpdate);
  server.on("/api/status",      HTTP_GET, handleAPIStatus);
  server.on("/api/config",      HTTP_GET, handleConfigGet);
  server.on("/api/config",      HTTP_POST, handleConfigSave);
  server.on("/api/auto",        HTTP_POST, handleAutoModeConfig);
  server.on("/api/preset/save", HTTP_POST, handlePresetSave);
  server.on("/api/preset/load", HTTP_GET, handlePresetLoad);
  server.on("/api/preset/list", HTTP_GET, handlePresetList);
  
  // Routes OTA
  server.on("/api/ota/firmware", HTTP_POST, handleOTAFirmware);
  server.on("/api/ota/littlefs", HTTP_POST, handleOTALittleFS);
  server.on("/api/ota/complete", HTTP_POST, handleOTAComplete);
  
  // Routes pour upload OTA depuis le navigateur
  server.on("/uploadfirmware",  HTTP_GET, handleUploadFirmware);
  server.on("/uploadlittlefs",  HTTP_GET, handleUploadLittleFS);
  server.on("/uploadcomplete",  HTTP_GET, handleUploadComplete);
  
  // Routes pour contrôle du mode automatique depuis l'URL
  server.on("/auto/on",  HTTP_GET, handleAutoModeOn);
  server.on("/auto/off", HTTP_GET, handleAutoModeOff);
  
  // Routes legacy (compatibilité)
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

  // Gestion du mode automatique
  handleAutoMode();

  stepper_slider.run();
  stepper_pan.run();
  stepper_tilt.run();
}
