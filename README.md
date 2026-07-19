# BatteryKalman - Extended Kalman Filter pour l'estimation de capacité batterie

[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue)](https://www.arduino.cc/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange)](https://platformio.org/)
[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)
[![Version](https://img.shields.io/badge/Version-1.5-brightgreen.svg)](https://github.com/Fo170/BatteryKalman/releases)
[![GitHub Repository](https://img.shields.io/badge/GitHub-Fo170/BatteryKalman-blue)](https://github.com/Fo170/BatteryKalman)
[![Compatible: BatteryModels](https://img.shields.io/badge/Compatible-BatteryModels%20v1.2.0%2B-success)](https://github.com/Fo170/BatteryModels)

**Extended Kalman Filter (EKF) 2D** pour l'estimation simultanée de:
- **Capacité batterie (C)** - en Ampere-heures (Ah)
- **Taux de vieillissement (dC/dcycle)** - appris automatiquement
- **État de charge (SoC)** - en fusion coulomb-counting + tension

Cette librairie est **technologie-agnostique** et utilise une **interface `BatteryModel` abstraite** pour les paramètres spécifiques (OCV, résistances, courbes de charge, etc.).

### 💚 Compatible avec [BatteryModels v1.2.0+](https://github.com/Fo170/BatteryModels)

[**BatteryModels**](https://github.com/Fo170/BatteryModels) est une implémentation **recommandée et totalement compatible** de l'interface `BatteryModel`. 

Vous pouvez aussi utiliser:
- ✅ Votre propre implémentation de `BatteryModel`
- ✅ Une autre librairie compatible
- ✅ Votre code BMS personnalisé

## 🔗 Repository GitHub

**Official Repository**: [**https://github.com/Fo170/BatteryKalman**](https://github.com/Fo170/BatteryKalman)

- **Main Branch**: Latest stable release (v1.5)
- **License**: GNU General Public License v3
- **Issue Tracker**: [GitHub Issues](https://github.com/Fo170/BatteryKalman/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Fo170/BatteryKalman/discussions)

**Clone from GitHub**:
```bash
git clone https://github.com/Fo170/BatteryKalman.git
cd BatteryKalman
git checkout main
```

## 📦 Installation

### 🟢 Interface BatteryModel Requise (Recommandé: BatteryModels v1.2.0+)

BatteryKalman **REQUIRE** une classe implémentant l'interface `BatteryModel`.  
**Recommandé**: [BatteryModels v1.2.0+](https://github.com/Fo170/BatteryModels) - fully compatible

### Installation (PlatformIO) - Recommandé

```ini
[env:your_board]
lib_deps = 
    Fo170/BatteryModels >= 1.2.0    # ← Implémentation BatteryModel (recommandée)
    Fo170/BatteryKalman >= 1.5      # Extended Kalman Filter
    Fo170/Coulomb >= 1.0             # Compteur coulombs (optionnel)
```

### Installation (Arduino IDE)

1. **Installer BatteryModels** (recommandé):
   - Sketch → Include Library → Manage Libraries
   - Chercher "BatteryModels"
   - Installer v1.2.0 ou plus récent

2. **Installer BatteryKalman**:
   - Sketch → Include Library → Manage Libraries
   - Chercher "BatteryKalman"
   - Installer v1.5 ou plus récent

### Installation (GitHub - Branche Main)

```bash
# Cloner le repository
git clone https://github.com/Fo170/BatteryKalman.git
cd BatteryKalman

# Copier dans Arduino libraries
cp -r . ~/Arduino/libraries/BatteryKalman
```

### Vérifier l'Installation

Après installation, votre sketch doit compiler:
```cpp
#include <BatteryModels.h>  // Interface BatteryModel (recommandé)
#include <BatteryKalman.h>  // Extended Kalman Filter
#include <Coulomb.h>        // Si utilisé

// Crée votre modèle batterie
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);

// Filtre Kalman
KalmanState2D kalmanState;
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
```

---

## 📋 Caractéristiques (v1.5)

### Kalman Filter
- **Extended Kalman Filter 2D** : Estime simultanément capacité + taux vieillissement
- **Process Noise Q** : Croissance d'incertitude entre mesures
- **Online R Estimation** : Adaptation automatique du bruit de mesure
- **Continuous Confidence** : Métrique [0,1] au lieu de phases discrètes

### Batterie
- **Technologie-agnostique** : fonctionne avec tout type batterie via `BatteryModel`
- **Détection automatique d'état** : BULK, ABSORPTION, FLOAT, REST via [BatteryModels](https://github.com/Fo170/BatteryModels)
- **Apprentissage du vieillissement** : dC/dcycle appris automatiquement (pas hard-codé)
- **Détection remplacement batterie** : Détection d'outliers + magnitude check
- **Cycle counting** : Suivi des cycles complets et partiels
- **Gestion des outliers** : 3-sigma test robuste

### Persistance
- **État Kalman persistable** : SoCData + KalmanState2D
- **Support multi-backend** : EEPROM, LittleFS, Preferences, SD card, etc.

## 📊 Phases d'apprentissage

| Phase | Nom | Précision | Description |
|-------|-----|-----------|-------------|
| 0 | Bootstrap | ±15-20% | Capacité inconnue, SoC basé sur OCV brut |
| 1 | Convergence rapide | ±10-15% | Première estimation grossière |
| 2 | Raffinement | ±5-8% | Précision croissante |
| 3 | Suivi vieillissement | ±3-5% | Précision maximale, suivi de dérive |

## 🔧 Principe de fonctionnement

L'algorithme utilise un filtre de Kalman 1D pour estimer la capacité de la batterie :
