# Requirements - BatteryKalman v1.5

## 🔴 DÉPENDANCE OBLIGATOIRE

### Interface BatteryModel (Requise)

BatteryKalman **REQUIRE** une classe implémentant l'interface `BatteryModel` avec les méthodes:

```cpp
class BatteryModel {
public:
    ChargeState detectChargeState(float V, float I, float C);
    float correctVoltageToOCV(float V, float I, float T, float dt, float* r_eff, ...);
    float ocvToSoc(float V, float T);
    uint8_t getCellCount();
    float getNominalCapacity();
    float getMinVoltage() / getMaxVoltage();
    const char* getTechnologyName();
    bool isAutoDetect();
    bool isOcvReliable();
    // ... autres méthodes
};
```

### 💚 IMPLÉMENTATION RECOMMANDÉE: BatteryModels v1.2.0+

**[GitHub: https://github.com/Fo170/BatteryModels](https://github.com/Fo170/BatteryModels)**

**Totalement compatible et recommandée**, mais **PAS OBLIGATOIRE**.

#### Vous pouvez utiliser:
- ✅ [BatteryModels](https://github.com/Fo170/BatteryModels) v1.2.0+ (recommandé)
- ✅ Votre propre implémentation BatteryModel
- ✅ Implémentation BMS personnalisée

#### Interfaces Requises

BatteryKalman appelle ces méthodes sur votre objet `BatteryModel*`:

```cpp
// Détection état (BULK, ABSORPTION, FLOAT, REST, DISCHARGE)
ChargeState detectChargeState(float voltage, float current, float capacity);

// Correction tension → OCV (modèle Thevenin)
float correctVoltageToOCV(float V, float I, float T, float dt, float* r_eff, ...);

// Lookup OCV → SoC
float ocvToSoc(float V, float T);

// Métadonnées
uint8_t getCellCount();
float getNominalCapacity();
float getMinVoltage();
float getMaxVoltage();
const char* getTechnologyName();

// Flags
bool isAutoDetect();
bool isOcvReliable();
```

#### Installation de BatteryModels (Recommandée)

**PlatformIO:**
```ini
[env:esp32]
lib_deps = 
    Fo170/BatteryModels >= 1.2.0
    Fo170/BatteryKalman >= 1.5
```

**Arduino IDE:**
1. Sketch → Include Library → Manage Libraries
2. Search "BatteryModels" → Install v1.2.0+
3. Search "BatteryKalman" → Install v1.5+

**Vérification:**
```cpp
#include <BatteryModels.h>
// Si erreur "file not found" → BatteryModels pas installé
```

---

## ✅ DÉPENDANCES OPTIONNELLES

### Coulomb (Optionnel)

**Purpose**: Ampere-hour integration over time

**GitHub**: [Fo170/Coulomb](https://github.com/Fo170/Coulomb)

**Status**: Optional si vous implémentez votre propre coulomb counter

**Alternative**: Implémenter `Coulomb` interface avec vos propres méthodes

---

## 💾 DÉPENDANCES POUR PERSISTANCE

Si vous utilisez `saveState()` / `loadState()` avec JSON:

### ArduinoJson (Optionnel)

**Purpose**: Serialization/deserialization JSON

**PlatformIO**:
```ini
lib_deps = 
    bblanchon/ArduinoJson @ ^6.20.0
```

**Arduino IDE**:
- Library Manager → Search "ArduinoJson"
- Install latest v6.x

**Alternative**: Utiliser vos propres méthodes de sérialisation

---

## 🎛️ HARDWARE REQUIREMENTS

### Processeur
- **Minimum**: STM32 (32-bit, ~1 MHz)
- **Recommended**: ESP32, ESP8266, Teensy, STM32H7
- **Memory**: ≥32 KB FLASH, ≥4 KB RAM

### Support Arduino Framework
- ✓ ESP32 / ESP8266
- ✓ STM32 (via STM32duino)
- ✓ Teensy 3.x / 4.x
- ✓ Arduino Due, Mega, Uno (marginal, juste memory)
- ✓ Raspberry Pi Pico (via Arduino IDE 2.0)

### Capteurs Requis
1. **Tension batterie** — ADC ou I²C sensor
2. **Courant batterie** — Shunt + ampli OU INA226/INA219
3. **Température** — Optional mais recommandé (NTC, DS18B20)

---

## 📦 INSTALLATION COMPLÈTE

### Checklist

```
☐ 1. Arduino IDE 1.8.13+ OU PlatformIO
☐ 2. BatteryModels v1.2.0+ (★ OBLIGATOIRE)
☐ 3. BatteryKalman v1.5
☐ 4. Coulomb (si pas implémenté en propre)
☐ 5. ArduinoJson (si persistance JSON)
☐ 6. Board support (ESP32, STM32, etc.)
```

### PlatformIO (Recommandé)

```ini
[env:esp32-dev]
platform = espressif32
board = esp32-devkitc-32
framework = arduino

lib_deps =
    Fo170/BatteryModels >= 1.2.0    # ← OBLIGATOIRE
    Fo170/BatteryKalman >= 1.5       # Cette librairie
    Fo170/Coulomb >= 1.0             # Optional
    bblanchon/ArduinoJson @ ^6.20.0 # Optional (persistance)

monitor_speed = 115200
```

### Arduino IDE

1. **Télécharger**:
   - Arduino IDE 1.8.13+ from [arduino.cc](https://www.arduino.cc/en/software)

2. **Installer board support** (exemple ESP32):
   - File → Preferences → Additional Boards Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Board Manager
   - Install "esp32" by Espressif

3. **Installer libraries** (Tools → Manage Libraries):
   - Search "BatteryModels" → Install (v1.2.0+)
   - Search "BatteryKalman" → Install (v1.5+)
   - Search "Coulomb" → Install (optional)
   - Search "ArduinoJson" → Install (optional)

4. **Vérifier setup**:
   ```cpp
   #include <BatteryModels.h>
   #include <BatteryKalman.h>
   
   void setup() {
       // Devrait compiler
   }
   ```

---

## 🔧 CONFIGURATION RECOMMANDÉE

### Version Complète (Recommandée)

```ini
# platformio.ini
[env:production]
lib_deps =
    Fo170/BatteryModels >= 1.2.0
    Fo170/BatteryKalman >= 1.5
    Fo170/Coulomb >= 1.0
    bblanchon/ArduinoJson @ ^6.20.0
```

### Version Minimale

```ini
[env:minimal]
lib_deps =
    Fo170/BatteryModels >= 1.2.0    # ★ ONLY MANDATORY
    Fo170/BatteryKalman >= 1.5
```

---

## 🐛 TROUBLESHOOTING

### Erreur: `BatteryModels.h: No such file`

**Cause**: BatteryModels pas installé

**Solution**:
1. Vérifier: Sketch → Include Library → Manage Libraries
2. Search "BatteryModels"
3. Installer v1.2.0+
4. Restart Arduino IDE

### Erreur: `detectChargeState undefined`

**Cause**: BatteryModels < v1.2.0 installé (API changée)

**Solution**:
1. Library Manager → BatteryModels
2. Vérifier version = 1.2.0+
3. Si old version: Uninstall, puis Install 1.2.0+

### Erreur: `out of memory` (Arduino Uno)

**Cause**: Pas assez de RAM

**Solution**:
- Downgrade optionnel: Supprimer P[2][2], revenir à P scalaire
- OU: Utiliser ESP32 (RAM 512 KB)
- OU: Utiliser version v2.0.0_backup (1D Kalman)

### Erreur: `Coulomb not found`

**Cause**: Coulomb optional; vous devez implémenter interface

**Solution**: Voir `Exemples/avec_persistance/` pour créer votre propre Coulomb wrapper

---

## 📋 VÉRIFICATION FINALE

Avant déploiement, vérifier:

```cpp
#include <Arduino.h>
#include <BatteryModels.h>  // ✓ Compile?
#include <BatteryKalman.h>   // ✓ Compile?

SoCData socData;
KalmanState2D kalmanState;
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

void setup() {
    battery.begin();  // ✓ Initialise?
}

void loop() {
    battery.update(12.5f, 10.0f, 25.0f);  // ✓ Update marche?
    float soc = battery.getSoC();          // ✓ Getter OK?
}
```

---

## 📞 VERSION MATRIX

| Component | Min Version | Recommended | Max Tested |
|-----------|-------------|-------------|------------|
| Arduino IDE | 1.8.13 | 2.0+ | 2.3 |
| BatteryModels | **1.2.0** | 1.2.0+ | 1.2.x |
| BatteryKalman | **1.5** | 1.5+ | 1.5 |
| Coulomb | 1.0 | 1.0+ | 1.0+ |
| ArduinoJson | 6.18 | 6.20+ | 7.0 |

---

**Last Updated**: July 20, 2026  
**Status**: Current for v1.5 release
