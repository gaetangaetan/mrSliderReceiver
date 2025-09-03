# MrSlider Control Interface

Interface moderne pour contrôler le système de caméra motorisée MrSlider.

## Fonctionnalités

### Contrôles
- **7 sliders interactifs** avec couleurs distinctives :
  - 3 sliders pour les positions (X, Pan, Tilt)
  - 3 sliders pour les vitesses correspondantes
  - 1 slider pour l'accélération globale

### Système de Presets
- **6 emplacements de sauvegarde** (S1-S6)
- **6 boutons de chargement** (L1-L6)
- Stockage persistant en mémoire flash (LittleFS)
- Indicateurs visuels pour les presets existants

### Interface Moderne
- Mode sombre élégant
- Sliders colorés avec animations
- Mise à jour temps réel (100ms)
- Interface responsive pour mobile/desktop

## Utilisation

### Configuration des limites
Les limites des axes sont définies dans le code :
```cpp
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
```

### Accès à l'interface
- **Adresse IP locale** : `http://[IP_ESP8266]/`
- **mDNS** : `http://mrslider.local/`

### API REST

#### Mise à jour des paramètres
```
GET /api/update?posx=50&pan=30&tilt=45&speed_posx=20
```

#### Obtenir l'état actuel
```
GET /api/status
```

#### Gestion des presets
```
POST /api/preset/save
GET /api/preset/load?slot=1
GET /api/preset/list
```

### Contrôle manuel via URL (compatibilité legacy)
```
http://mrslider.local/setParams?posx=50&pan=30&tilt=45&speed_posx=20&speed_pan=15&speed_tilt=10&acceleration=20
```

## Installation

1. **Compilation** : Utilisez PlatformIO
2. **Upload du firmware** : `pio run -t upload`
3. **Upload des fichiers web** : `pio run -t uploadfs`

## Structure des fichiers

```
mrSliderReceiver/
├── src/
│   └── main.cpp           # Code principal
├── data/                  # Fichiers web (LittleFS)
│   ├── index.html        # Interface principale
│   ├── style.css         # Styles modernes
│   └── script.js         # Logique interactive
├── platformio.ini        # Configuration PlatformIO
└── README.md            # Documentation
```

## Changelog v156+

- ✅ Interface web moderne avec mode sombre
- ✅ Système de presets persistants (LittleFS)
- ✅ API REST complète
- ✅ Mise à jour temps réel (100ms)
- ✅ Validation des limites des axes
- ✅ Sliders colorés et animations
- ✅ Interface responsive
- ✅ Indicateurs de statut de connexion

## Support

Pour des questions techniques, vérifiez :
- La connexion WiFi
- L'initialisation de LittleFS dans le Serial Monitor
- Les logs d'erreur dans la console du navigateur
