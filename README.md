# BatteryKalman - Extended Kalman Filter pour l'estimation de capacité batterie

[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue)](https://www.arduino.cc/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange)](https://platformio.org/)
[![License: GPLv3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)
[![Version](https://img.shields.io/badge/Version-1.7.4-brightgreen.svg)](https://github.com/Fo170/BatteryKalman/releases)
[![GitHub Repository](https://img.shields.io/badge/GitHub-Fo170/BatteryKalman-blue)](https://github.com/Fo170/BatteryKalman)
[![Compatible: BatteryModels](https://img.shields.io/badge/Compatible-BatteryModels%20v1.6.9%2B-success)](https://github.com/Fo170/BatteryModels)

**Extended Kalman Filter (EKF) 2D** pour l'estimation simultanée de:
- **Capacité batterie (C)** - en Ampere-heures (Ah)
- **Taux de vieillissement (dC/dcycle)** - appris automatiquement
- **État de charge (SoC)** - en fusion coulomb-counting + tension

> **v1.7.3** — **Compteurs Ah réconciliés avec le SoC.** `Ah_total_discharged` intégrait
> `(−I)·dt_h` **brut** et `Ah_total_charged` un **0,98 codé en dur**, alors que `SoC_coulomb`
> utilise le `dAh` **corrigé** (rendement η) du compteur `Coulomb` : à 5 A les deux divergeaient
> de **1,7-1,9×**, et les compteurs exposés en UI (`ah_chg`/`ah_dis`) contredisaient le SoC et
> l'autonomie. Les deux compteurs dérivent désormais de la **même source** (`dAh`).
> Ajout de `getCoulombMeter()` pour que l'UI affiche le η **réellement appliqué** au lieu de
> recalculer la loi depuis `BatteryModel` (qui diverge : η constant côté compteur vs η(I) de
> Peukert côté modèle). **Aucun breaking change d'API** ; le comportement de `Ah_total_*`
> change volontairement (cohérence avec `SoC_coulomb`).
>
> **v1.7.2** — **« le coulomb est la référence »** : `fuseSoC()` ne pondère plus par la
> confiance et `alpha = 1` en BULK/ABSORPTION/DISCHARGE (SoC fusionné = SoC coulomb, factuel),
> alpha faible au repos (FLOAT 0 · REST_LONG 0.02 · REST 0.10) → **fin du saut de SoC**
> quand la confiance montait en phase Convergence (cas 80 % coulomb / 95 % tension :
> 92,15 % en COARSE → 80,75 % en TRACK, soit ~11 % de saut). Corollaire :
> `correctVoltageToOCV(..., soc_ref)` reçoit désormais `SoC_coulomb` → fin de l'emballement
> `SoC_voltage = 100 %` sous charge. `KALMAN_COARSE_ALPHA_MIN` supprimé. Ajout de la macro
> `__BATTERYKALMAN_H_VERSION__` (source unique de version). **Aucun breaking change.**
>
> **v1.7.1** — Ajout de l'**état persistable de l'observateur passif** : accesseurs
> `getRegressionRing()`/`setRegressionRing()` (ring P2) et `setPseudoRestEvents()`
> (compteur P1), pour sauver/restaurer P1/P2 dans votre stockage. Aucun breaking change.
>
> **v1.7.0** — Chasse aux bugs (K1–K12, R1–R7) + observateur passif (P1/P2) :
> 12 correctifs d'algorithme/état, **seuil de planéité REST_LONG auto-adaptatif au bruit
> de mesure** (R6), **profil REST_LONG par technologie** via `applyRecommendedRestLong()`
> (R7), **mode pseudo-repos** `enablePseudoRest()` (P1) et **régression de capacité
> pondérée** (P2). L'algorithme EKF a été **vérifié mathématiquement sur 4 sources**
> (§ *Principe de fonctionnement*). Détails :
> [`CHANGELOG.md`](CHANGELOG.md) (Annexe K/R) et [`doc/rest_long_tuning.md`](doc/rest_long_tuning.md).

> **v1.6.0** — Configuration par technologie : le filtre consomme
> `model->getKalmanTuning()` (P/Q/R, seuils de segment, REST_LONG, confiance)
> avec une hiérarchie **setter runtime > modèle > macros compile-time > défaut**.
> Rétro-compatible : sans setter ni BatteryModels v1.6.9+, comportement v1.5.3.
> Voir [`CHANGELOG.md`](CHANGELOG.md) et la comparaison détaillée dans [`doc/comparaison_kalman.md`](doc/comparaison_kalman.md).

Cette librairie est **technologie-agnostique** et utilise une **interface `BatteryModel` concrète** pour les paramètres spécifiques (OCV, résistances, courbes de charge, etc.).

### 💚 Compatible avec [BatteryModels v1.6.9+](https://github.com/Fo170/BatteryModels)

[**BatteryModels**](https://github.com/Fo170/BatteryModels) est une implémentation **recommandée et totalement compatible** de l'interface `BatteryModel`.
Les réglages Kalman (P/Q/R, seuils, REST_LONG, confiance) sont fournis **par technologie** via `getKalmanTuning()`.

Vous pouvez aussi utiliser:
- ✅ Votre propre implémentation de `BatteryModel`
- ✅ Une autre librairie compatible
- ✅ Votre code BMS personnalisé

## 🔗 Repository GitHub

**Official Repository**: [**https://github.com/Fo170/BatteryKalman**](https://github.com/Fo170/BatteryKalman)

- **Main Branch**: Latest stable release (v1.7.4)
- **License**: GNU General Public License v3
- **Issue Tracker**: [GitHub Issues](https://github.com/Fo170/BatteryKalman/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Fo170/BatteryKalman/discussions)

**Clone from GitHub**:
```bash
git clone https://github.com/Fo170/BatteryKalman.git
cd BatteryKalman
git checkout main
```

## 📑 Sommaire

- [Installation](#-installation)
- [Dépendances & matériel](#-dépendances--matériel)
- [Caractéristiques](#-caractéristiques-v172)
- [Phases d'apprentissage](#-phases-dapprentissage)
- [Principe de fonctionnement (EKF 2D)](#-principe-de-fonctionnement--algorithme-ekf-2d)
- [Tuning recommandé](#-tuning-recommandé)
- [Observateur passif (P1/P2)](#-observateur-passif--installations-sans-repos-p1p2)
- [Compatibilité & implémentation personnalisée](#-compatibilité--implémentation-personnalisée)
- [Migration](#-migration)
- [Historique v1.5](#-historique-v15--design--release)
- [Vérifications locales](#-vérifications-locales-sans-matériel)
- [Références](#-références)

## 📦 Installation

### 🟢 Interface BatteryModel Requise (Recommandé: BatteryModels v1.6.9+)

BatteryKalman **REQUIRE** une classe implémentant l'interface `BatteryModel`.
**Recommandé**: [BatteryModels v1.6.9+](https://github.com/Fo170/BatteryModels) - fully compatible

### Installation (PlatformIO) - Recommandé

```ini
[env:your_board]
lib_deps =
    Fo170/BatteryModels >= 1.6.9    # ← Implémentation BatteryModel (recommandée, soc_ref requis)
    Fo170/BatteryKalman >= 1.7.4    # Extended Kalman Filter
    Fo170/CoulombsAh >= 1.1         # Compteur coulombs (Coulomb.h)
```

### Installation (Arduino IDE)

1. **Installer BatteryModels** (recommandé):
   - Sketch → Include Library → Manage Libraries
   - Chercher "BatteryModels"
   - Installer v1.6.9 ou plus récent

2. **Installer BatteryKalman**:
   - Sketch → Include Library → Manage Libraries
   - Chercher "BatteryKalman"
   - Installer v1.7.4 ou plus récent

3. **Installer CoulombsAh** (obligatoire, `Coulomb.h`):
   - Chercher "CoulombsAh" → Installer v1.1+

4. **ArduinoJson** (optionnel, uniquement pour l'exemple `avec_persistance`) :
   - Chercher "ArduinoJson" → Installer v6.20+

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
#include <Coulomb.h>        // Compteur coulombs (obligatoire, depuis Fo170/CoulombsAh)

// Crée votre modèle batterie
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);

// Données à persister
SoCData socData;
KalmanState2D kalmanState;

// Compteur coulombs (CoulombsAh v1.1+)
Coulomb coulomb;

// Filtre Kalman (API v1.7.4 : SoCData + KalmanState2D + BatteryModel + Coulomb)
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

// Profil REST_LONG adapté à la technologie (plomb 30 min, Li-ion 15 min, ...)
battery.begin();
battery.applyRecommendedRestLong();
```

## 🧩 Dépendances & matériel

### Interface `BatteryModel` (requise)

BatteryKalman appelle ces méthodes sur votre objet `BatteryModel*` :

```cpp
// Détection état (BULK, ABSORPTION, FLOAT, REST, DISCHARGE)
ChargeState detectChargeState(float voltage, float current, float capacity);

// Correction tension → OCV (modèle Thévenin + thermique)
float correctVoltageToOCV(float V, float I, float T, float dt, float* r_eff, float* v_pol);

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

> La v1.6.0 consomme en plus `model->getKalmanTuning()` (BatteryModels v1.6.9+), optionnel.

### `Coulomb` (obligatoire)

Intégration ampère-heure (`I×dt`). Fourni par [Fo170/CoulombsAh](https://github.com/Fo170/CoulombsAh)
(`Coulomb.h`, API `addMeasurement(current)` qui intègre via `micros()`).

**Obligatoire depuis la v1.6.0** : le constructeur `BatteryKalman(..., Coulomb*)` n'a pas de
null-guard. Persistance déléguée à [Fo170/Persistance](https://github.com/Fo170/Persistance).
Alternative : implémenter votre propre classe `Coulomb`.

### ArduinoJson (optionnel)

Nécessaire uniquement pour la persistance JSON de l'exemple `Exemples/avec_persistance/`
(`bblanchon/ArduinoJson @ ^6.20.0`). Alternative : votre propre sérialisation.

### Matériel requis

| Élément | Détail |
|---|---|
| Processeur | min. STM32 (32-bit) ; recommandé ESP32/ESP8266/Teensy/STM32H7 |
| Mémoire | ≥ 32 KB FLASH, ≥ 4 KB RAM |
| Framework | Arduino (C++11 minimum) — ESP32/ESP8266, STM32duino, Teensy 3.x/4.x, Due, Mega, Pico (Uno marginal) |
| Capteurs | 1) tension batterie (ADC ou I²C) ; 2) courant (shunt + ampli, ou INA226/INA219) ; 3) température (NTC/DS18B20, recommandé) |

### Matrice de versions

| Composant | Min | Recommandé | Testé max |
|-----------|-----|------------|-----------|
| Arduino IDE | 1.8.13 | 2.0+ | 2.3 |
| BatteryModels | **1.6.9** | 1.6.9+ | 1.6.9 |
| BatteryKalman | **1.7.4** | 1.7.4 | 1.7.4 |
| CoulombsAh (`Coulomb.h`) | 1.1 | 1.1+ | 1.1+ |
| ArduinoJson | 6.18 | 6.20+ | 7.0 |

### Dépannage

| Erreur | Cause | Solution |
|---|---|---|
| `BatteryModels.h: No such file` | BatteryModels non installé | Library Manager → installer v1.6.9+ |
| `detectChargeState undefined` | BatteryModels < v1.3 | Mettre à jour (API renommée en v1.2) |
| `Coulomb not found` | CoulombsAh non installé | Installer [Fo170/CoulombsAh](https://github.com/Fo170/CoulombsAh) v1.1+ |
| `out of memory` (Uno) | RAM insuffisante | Utiliser ESP32, ou réduire les buffers |

## 📋 Caractéristiques (v1.7.4)

### Kalman Filter
- **Extended Kalman Filter 2D** : Estime simultanément capacité + taux vieillissement
- **Process Noise Q** : Croissance d'incertitude entre mesures
- **Online R Estimation** : Adaptation automatique du bruit de mesure
- **Continuous Confidence** : Métrique [0,1] au lieu de phases discrètes
- **🐛 v1.5.3** : EKF 2D réellement complet (vieillissement appris), R adaptatif appliqué, transition FLOAT fonctionnelle, porte de stabilité REST (F8)
- **✨ v1.6.0** : configuration P/Q/R + seuils + REST_LONG + confiance **par technologie** (`model->getKalmanTuning()`), setters runtime, seuil ΔAh relatif, gating OCV (NiFe/Sodium)
- **✨ v1.7.0 (R6/R7)** : seuil de planéité REST_LONG adapté au **bruit de mesure** ; **profil REST_LONG par technologie** via `applyRecommendedRestLong()` (plomb 30 min / Li-ion 15 min / …)
- **✨ v1.7.0 (P1/P2)** : **mode pseudo-repos** opt-in (`enablePseudoRest()`) et **régression de capacité pondérée** (`getRegressionCapacity()`, …)
- **🐛 Corrections v1.7.0 (K1–K12)** : anneau de stabilité REST rotatif, REST_LONG stable, signe du vieillissement, underflow `millis()`, **apprentissage du vieillissement réellement effectif** (accumulation des cycles), **resynchronisation `last_Ah` à la restauration** (K8), **porte temporelle du cutoff basse tension** (K9), **tolérance au wrap `millis()`** (K11) — voir [`CHANGELOG.md`](CHANGELOG.md) (Annexe K/R)
- **✨ v1.7.1** : état P1/P2 **persistable** (`getRegressionRing()` / `setRegressionRing()` / `getRegressionMaxPoints()` / `setPseudoRestEvents()`)
- **🐛 v1.7.2** : fusion SoC **sans pondération par la confiance** (« le coulomb est la référence », `alpha = 1` en charge/décharge) et `soc_ref` transmis à `correctVoltageToOCV()` — plus de saut de SoC en phase Convergence, plus d'emballement `SoC_voltage = 100 %` sous charge
- **🐛 v1.7.4** : **pseudo-repos validé sur une DURÉE** (30 s mini) et plus seulement sur 2 échantillons — à 10 Hz cela valait 0,2 s, très inférieur à τ_pol (relaxation non terminée → `C_hat` biaisé)
- **🐛 v1.7.3** : **compteurs Ah réconciliés avec le SoC** — `Ah_total_charged`/`Ah_total_discharged` dérivent du `dAh` corrigé (rendement η) au lieu de `I` brut + `0,98` codé en dur (écart de 1,7-1,9× à 5 A) ; ajout de `getCoulombMeter()` pour exposer le η réellement appliqué

### Batterie
- **Technologie-agnostique** : fonctionne avec tout type batterie via `BatteryModel`
- **Détection automatique d'état** : BULK, ABSORPTION, FLOAT, REST via [BatteryModels](https://github.com/Fo170/BatteryModels)
- **Apprentissage du vieillissement** : dC/dcycle appris automatiquement (pas hard-codé)
- **Détection remplacement batterie** : Détection d'outliers + magnitude check
- **Cycle counting** : Suivi des cycles complets et partiels
- **Gestion des outliers** : 3-sigma test robuste
- **Référence OCV auto-adaptative** : porte REST_LONG réglée **par technologie** (repos min / fenêtre / seuil) et seuil de planéité **adapté au bruit** — voir [`doc/rest_long_tuning.md`](doc/rest_long_tuning.md)

### Persistance
- **État Kalman persistable** : SoCData + KalmanState2D
- **Support multi-backend** : EEPROM, LittleFS, Preferences, SD card, etc.
- La librairie **ne persiste pas elle-même** : `saveState()`/`loadState()` sont des stubs à implémenter côté utilisateur. Recommandé : la librairie [**Fo170/Persistance**](https://github.com/Fo170/Persistance) (EEPROM/FRAM/LittleFS), avec des registres par backend. Les exemples `Exemples/` illustrent le branchement.

## 📊 Phases d'apprentissage

| Phase | Nom | Précision | Description |
|-------|-----|-----------|-------------|
| 0 | Bootstrap | ±15-20% | Capacité inconnue, SoC basé sur OCV brut |
| 1 | Convergence rapide | ±10-15% | Première estimation grossière |
| 2 | Raffinement | ±5-8% | Précision croissante |
| 3 | Suivi vieillissement | ±3-5% | Précision maximale, suivi de dérive |

## 🐛 Évolutions et corrections — détail (quoi, pourquoi, formule)

> Chaque entrée donne **ce qui était faux**, **pourquoi** et la **formule exacte** avant/après.
> Sources en fin de document ([Références](#-références)).

### v1.7.4 — 2026-10-18 · pseudo-repos : critère de durée (B3)

**Ce qui était faux.** Une ancre OCV n'était validée que sur un **nombre d'échantillons**
(`PSEUDO_REST_MIN_SAMPLES = 2`), en supposant des échantillons espacés de plusieurs minutes
(commentaire d'origine : « 2 x 5 min = 10 min »). Un firmware appelant `update()` à **10 Hz**
validait donc l'ancre après **0,2 s** de repos.

**Pourquoi c'était faux.** La relaxation de polarisation du plomb a une constante de temps
τ_pol de plusieurs dizaines de secondes. À 0,2 s la tension terminale est encore loin de l'OCV :
l'ancre est surestimée en charge et sous-estimée en décharge → `C_hat` biaisé.

**Ce qui a changé.**

```
ancre valide  ⟺  (nb_échantillons ≥ pseudo_rest_min_samples)  ET  (durée ≥ pseudo_rest_min_ms)
```

`PSEUDO_REST_MIN_MS` = **30 000 ms** (nouveau) · `enablePseudoRest(..., min_duration_ms)` ·
`getPseudoRestMinMs()`. API additive ; `min_duration_ms = 0` restaure l'ancien comportement.

### v1.7.3 — 2026-10-18 · compteurs Ah réconciliés avec le SoC

**Ce qui était faux.** Les deux compteurs d'Ah publiés n'utilisaient pas le même courant que le
SoC, donc ne se réconciliaient plus avec lui :

```
Ah_total_discharged += (−I) · Δt_h            ← I BRUT
Ah_total_charged    +=   I  · Δt_h · 0,98     ← I BRUT + 0,98 codé en dur
SoC_coulomb         += (ΔAh_compteur / C_ref) · 100    ← ΔAh CORRIGÉ (η)
```

Depuis que `Coulomb` applique un rendement η, `ΔAh_compteur = I·Δt·η` : les compteurs et le SoC
divergaient d'un facteur **1/η** — soit **1,7× en décharge et 1,9× en charge à 5 A**. Ces compteurs
sont exposés en UI (`ah_chg` / `ah_dis`) : l'utilisateur voyait des valeurs qui contredisaient le
SoC et l'autonomie.

**Correctif.** Les deux compteurs dérivent de la **même source** que le SoC (`ΔAh` du compteur) :

```
ΔAh = Ah_compteur(k) − Ah_compteur(k−1)
si ΔAh < 0 :  Ah_total_discharged += −ΔAh
sinon      :  Ah_total_charged    +=  ΔAh
SoC_coulomb += (ΔAh / C_ref) · 100
```

Ajout de `getCoulombMeter()` pour que l'UI affiche le η **réellement appliqué** au lieu de
recalculer la loi depuis `BatteryModel` (qui diverge : η constant côté compteur vs η(I) de Peukert
côté modèle — corrigé en BatteryModels 1.6.10).

### v1.7.2 — 2026-10-18 · « le coulomb est la référence »

**Ce qui était faux.** `fuseSoC()` pondérait le SoC coulomb par la confiance du filtre. Quand la
confiance montait en phase Convergence, le poids du SoC **tension** augmentait → **saut de SoC**
(cas mesuré : 92,15 % en COARSE → 80,75 % en TRACK, soit ~11 % de saut sans aucune mesure réelle).

**Correctif.** `α` ne dépend plus de la confiance mais de l'**état de charge détecté** :

```
α = 0,00   si State_FLOAT        (ancre 100 %)
α = 0,02   si State_REST_LONG    (OCV très fiable)
α = 0,10   si State_REST         (OCV fiable)
α = 1,00   sinon (BULK/ABSORPTION/DISCHARGE)   → SoC fusionné = SoC coulomb

SoC_fusionné = α·SoC_coulomb + (1 − α)·SoC_voltage
```
Corollaire : `correctVoltageToOCV(..., soc_ref)` reçoit désormais `SoC_coulomb` → fin de
l'emballement `SoC_voltage = 100 %` sous charge. `KALMAN_COARSE_ALPHA_MIN` supprimé.

### v1.7.1 — 2026-10-17 · état P1/P2 persistable

Accesseurs `getRegressionRing()` / `setRegressionRing()` / `getRegressionMaxPoints()` /
`setPseudoRestEvents()` pour sauver et restaurer les observateurs P1/P2 dans le stockage hôte.

### v1.7.0 — 2026-10-17 · chasse aux bugs (K1–K12, R1–R7) + observateur passif

12 correctifs d'algorithme/état : anneau de stabilité REST rotatif, REST_LONG stable, signe du
vieillissement, underflow `millis()`, apprentissage du vieillissement réellement effectif
(accumulation des cycles), resynchronisation `last_Ah` à la restauration (K8), porte temporelle du
cutoff basse tension (K9), tolérance au wrap `millis()` (K11) ; seuil de planéité REST_LONG
auto-adaptatif au bruit (R6) et profil REST_LONG par technologie (R7) ; mode pseudo-repos (P1) et
régression de capacité (P2).

### v1.6.0 — configuration par technologie

P/Q/R, seuils et REST_LONG consommés depuis `model->getKalmanTuning()` (BatteryModels v1.4.0+).

### v1.5.3 — EKF 2D réellement complet

Vieillissement appris, R adaptatif appliqué, transition FLOAT fonctionnelle, porte de stabilité
REST (F8).

---

## 🔧 Principe de fonctionnement — algorithme EKF 2D

> 📄 Comparaison détaillée avec le Kalman général et les variantes batteries
> (EKF+ECM, AEKF, DEKF, UKF) : [`doc/comparaison_kalman.md`](doc/comparaison_kalman.md).

L'algorithme est un **filtre de Kalman étendu (EKF) 2D**. L'état estimé est :

$$
\mathbf{x} = \begin{bmatrix} C \\ a \end{bmatrix}, \qquad
C = \text{capacité (Ah)}, \quad a = \frac{dC}{d\text{cycle}} \;(\text{Ah/cycle})
$$

### Modèle de processus

À chaque intervalle de $n$ cycles écoulés depuis la dernière mesure, la capacité
dérive selon le taux de vieillissement appris $a$ :

$$
f(\mathbf{x}) = \begin{bmatrix} C - a\,n \\ a \end{bmatrix},
\qquad
\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} 1 & -n \\ 0 & 1 \end{bmatrix}
$$

### Modèle de mesure

La mesure est la **capacité observée** sur un segment entre deux références OCV
(FLOAT = 100 %, ou REST_LONG) :

$$
z = \frac{|\Delta Ah|}{|\Delta SoC| / 100}, \qquad
h(\mathbf{x}) = C, \qquad \mathbf{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

### Équations (prédiction / mise à jour)

**Prédiction**
$$
\hat{\mathbf{x}}^- = f(\hat{\mathbf{x}}), \qquad
\mathbf{P}^- = \mathbf{F}\,\mathbf{P}\,\mathbf{F}^{\!\top} + \mathbf{Q},
\qquad \mathbf{Q} = \mathrm{diag}(q_a\,n + q_C,\; q_a)
$$

**Mise à jour**
$$
\tilde{y} = z - C, \qquad
S = P^-_{00} + R, \qquad
\mathbf{K} = \begin{bmatrix} P^-_{00}/S \\ P^-_{10}/S \end{bmatrix}
$$
$$
\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}}^- + \mathbf{K}\tilde{y},
\qquad
\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\,\mathbf{P}^-
$$

Les paramètres $P$, $Q$, $R$, les seuils de segment, la porte REST_LONG et la confiance
sont réglés **par technologie** (`model->getKalmanTuning()`) avec la hiérarchie
setter runtime > modèle > macros > défaut.

### Extensions propres à la librairie

- **R adaptatif en ligne** : $R \leftarrow \alpha\,(\text{var}(y) - P_{00}) + (1-\alpha)R$,
  borné $[r_{min}, r_{max}]$ (bruit de mesure estimé depuis les innovations).
- **Confiance continue** : $\text{conf} = \tfrac{1}{2}\big(\min(1, n/10) + (1 - \min(1, P_{00}/P_{init}))\big) \in [0,1]$.
- **Rejet d'outliers** 3-σ + détection de remplacement batterie.
- **Fusion SoC** (filtre complémentaire, pas Kalman) : $SoC_{fused} = \alpha\,SoC_{coulomb} + (1-\alpha)\,SoC_{tension}$,
  avec $\alpha$ dépendant de l'état de charge. **Depuis la v1.7.2 il n'y a plus de pondération
  par la confiance** ($\alpha$ n'est plus multiplié par `kalman->confidence`) : le comptage
  coulomb est la **référence factuelle** et l'OCV n'ancre le SoC qu'au repos.

  | État | $\alpha$ | Sens |
  |---|---|---|
  | `State_FLOAT` | 0.00 | ancre « 100 % » (tension seule) |
  | `State_REST_LONG` | 0.02 | OCV très fiable (repos stabilisé) |
  | `State_REST` | 0.10 | OCV fiable (repos court) |
  | `State_BULK` / `State_ABSORPTION` / `State_DISCHARGE` / `State_UNKNOWN` | **1.00** | SoC fusionné = SoC coulomb |

  ⚠️ Ne **pas** réintroduire `alpha_base * confidence` : en phase Convergence (conf < 0.30)
  $\alpha$ tombait à ~0.19, donc la fusion repassait à ~81 % sur la tension → saut de SoC
  d'une dizaine de % quand la confiance remontait. `KALMAN_COARSE_ALPHA_MIN` a été supprimé
  avec ce changement.
- **Référence de l'OCV corrigé** : `update()` appelle
  `model->correctVoltageToOCV(V, I, T, dt, &r_int_eff, nullptr, data->SoC_coulomb)`.
  Le 7ᵉ argument (`soc_ref`) sert à dimensionner `R_soc`/`R_int` sur le SoC **coulomb** et non
  sur un SoC dérivé de la tension terminale (qui saturait `R_soc` à `R_soc_max` sous charge →
  emballement de `SoC_voltage` vers 100 %). **Requiert BatteryModels ≥ 1.6.9.**
- **Dérive de vieillissement (K7)** : les cycles sont **accumulés entre deux mesures**
  (`cycles_since_update`) et appliqués à la prédiction. Sans cette accumulation, $n \approx 0$
  à la mesure et $a$ n'est jamais appris (la covariance croisée $P_{10}$ reste nulle).

### Vérification de l'algorithme

Les équations ci-dessus ont été **recoupées sur plusieurs sources indépendantes** — elles
correspondent à l'EKF canonique (prédiction $\mathbf{P}^-=\mathbf{F}\mathbf{P}\mathbf{F}^{\!\top}+\mathbf{Q}$,
gain $\mathbf{K}=\mathbf{P}^-\mathbf{H}^{\!\top}\mathbf{S}^{-1}$, mise à jour
$\mathbf{P}=(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}^-$) :

| Source | Contenu |
|---|---|
| Wikipedia — [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) | Équations KF prédiction/mise à jour, Jacobiens, forme de Joseph |
| Wikipedia — [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) | EKF discret : $f,h$, $F_k=\partial f/\partial x$, $H_k=\partial h/\partial x$, équations predict/update |
| [kalmanfilter.net](https://www.kalmanfilter.net/kalman1d.html) | Dérivation KF 1D : état, covariance, gain, extrapolation |
| Wikipedia — [State of charge](https://en.wikipedia.org/wiki/State_of_charge) | Le Kalman filtering combine tension + coulomb-counting pour estimer le SoC |

**Conclusion** : le cœur EKF de BatteryKalman (prédiction, Jacobien
$\mathbf{F}=[[1,-n],[0,1]]$, gain 2D $\mathbf{K}=[P_{00},P_{10}]/S$, mise à jour d'état et de
covariance) est **mathématiquement correct** et conforme aux références. Les choix
d'implémentation (Q heuristique plutôt que $GG^{\!\top}$, bornage de $P$ à
$[p_{min}, p_{init}]$, prédiction déclenchée à la mesure) sont des variantes assumées, non des
erreurs. Le bug d'apprentissage du vieillissement (K7) est corrigé — voir
[`CHANGELOG.md`](CHANGELOG.md) (Annexe K/R).

## 🎛️ Tuning recommandé

Les réglages se font par technologie via `model->getKalmanTuning()` (BatteryModels v1.6.9+),
ou par les setters runtime (`setP/setQ/setR/setSegmentThresholds/setRestLong/setConfidence`).
Les macros compile-time restent un moyen global de forcer un réglage avant `#include`.

### Profils d'application (valeurs macro)

**Applications lentes (BMS stationnaires) :**
```cpp
#define Q_AGING_UNCERTAINTY   0.000001f  // Moins d'incertitude
#define R_SMOOTH_ALPHA        0.05f      // Lissage plus agressif
#define R_MIN                 0.2f       // Plus confiant aux mesures
```

**Applications rapides (véhicules) :**
```cpp
#define Q_AGING_UNCERTAINTY   0.000005f  // Plus d'incertitude
#define R_SMOOTH_ALPHA        0.2f       // Adapte vite
#define R_MAX                 50.0f      // Moins confiant
```

**Batteries instables :**
```cpp
#define Q_CAPACITY_DRIFT      0.05f      // Capacité peut dériver
#define BATTERY_CHANGE_THR    0.20f      // Seuil sensible
#define BATTERY_CHANGE_COUNT  2          // Détecte rapidement
```

### Profil REST_LONG par technologie (R7)

`battery.applyRecommendedRestLong()` applique automatiquement (déduit du nom de techno) :

| Technologie | `min_min` | `stable_min` | `stable_mv` |
|---|---|---|---|
| Plomb inondé/AGM/Gel/PbC | 30 | 60 | 30 mV |
| LiFePO4 (LFP) | 30 | 60 | 10 mV |
| Li-ion (NMC/NCA/LCO/LiPo/LMNO), LTO, NiMH/NiCd/NiZn, Sodium | 15 | 30 | 20 mV |
| Supercondensateur (EDLC) | 5 | 15 | 50 mV |

> ⚠️ **Cadence minimale** : avec `min_samples = 6`, il faut `dt ≤ stable_min/5`
> (fenêtre 30 min → `dt ≤ 6 min` ; 15 min → `dt ≤ 3 min`). Plus lent → abaisser
> `rest_long_stable_min_samples` via `setRestLong(...)`. Détails : [`doc/rest_long_tuning.md`](doc/rest_long_tuning.md).

## 📡 Observateur passif — installations sans repos (P1/P2)

> Cas des IoT qui **mesurent seulement** (V, I, T) et ne contrôlent ni charge ni décharge
> (ex. un Victron gère le système). Aucun test de capacité actif n'est possible : la capacité
> doit être extraite des **événements naturels**.

Deux fonctionnalités opt-in répondent à ce cas :

### P1 — Mode pseudo-repos (`enablePseudoRest()`)

Sur une installation à **décharge permanente** (ex. 0,45 A la nuit) et **charge PV** le jour,
les vrais repos n'existent pas. On exploite les **brefs instants à courant ~nul** comme **ancre
OCV basse** :

```cpp
battery.enablePseudoRest(/*I_max*/ 0.05f, /*min_samples*/ 2,
                         /*min_discharge_Ah*/ 2.0f, /*relax_tau_min*/ 0.0f);
```

- Détecte `|I| < I_max` pendant ≥ `min_samples` échantillons.
- N'accepte que les pseudo-repos en **décharge nette** (creux après la nuit), pas ceux de la charge.
- Optionnel : extrapolation de relaxation (`relax_tau_min > 0`) pour corriger la tension non
  encore équilibrée.

### P2 — Régression accumulée (`getRegressionCapacity()`)

Plutôt que d'estimer la capacité segment par segment (très bruité), on **accumule** les couples
`(ΔSoC, ΔAh)` et on ajuste une droite par l'origine : `ΔAh = C · ΔSoC/100`.

```cpp
float C = battery.getRegressionCapacity();   // capacité robuste (Ah)
uint8_t n = battery.getRegressionPoints();   // nb de points accumulés
float disp = battery.getRegressionDispersion(); // dispersion (%) des points
float dod  = battery.getRegressionMaxDoD();  // profondeur max observée (%)
```

**Pondération par la profondeur** : chaque point est pondéré par `ΔSoC²` (variance de
`C_i = ΔAh/(ΔSoC/100)` ∝ `1/ΔSoC²`). Conséquence : les **décharges profondes** dominent la
régression et **recalent** l'estimation, sans effacer l'historique des cycles doux.

**Principe** : chaque jour, l'ancre haute (FLOAT = 100 %) et l'ancre basse (pseudo-repos) donnent
un point. Le bruit s'annule sur N points (σ/√N). 100 % passif.

### 💾 Persistance de la régression (v1.7.1)

Le ring P2 (`reg_dsoc[]`/`reg_dah[]`) et le compteur d'événements P1 vivent en RAM : sans
sauvegarde, ils repartent de zéro à chaque reboot. La classe ne persiste pas elle-même
(`saveState()`/`loadState()` restent des stubs), mais expose l'état pour un stockage utilisateur :

```cpp
static float dsoc[BatteryKalman::getRegressionMaxPoints()];   // ou REG_MAX_POINTS
static float dah [BatteryKalman::getRegressionMaxPoints()];

// Sauvegarde (ex. FRAM) :
uint8_t n = battery.getRegressionRing(dsoc, dah);      // n = points valides
uint16_t ev = battery.getPseudoRestEvents();
// ... écrire n, ev, dsoc[0..n), dah[0..n) dans votre stockage ...

// Restauration (APRÈS begin()/enablePseudoRest()) :
battery.setRegressionRing(dsoc, dah, n);               // ordre indifférent
battery.setPseudoRestEvents(ev);
```

La régression étant invariante par permutation des points, l'ordre chronologique n'est pas
nécessaire. `getRegressionMaxPoints()` renvoie la taille compile-time du ring (`REG_MAX_POINTS`).

### 🎯 Recalage par décharge profonde (recommandé)

L'erreur sur la capacité varie en **~1/DoD** : un cycle à 5 % de profondeur donne ±30 % d'erreur
(20 mV de bruit ADC), un cycle à 50 % donne ±4 %. D'où la stratégie optimale :

> **Usage quotidien doux + une décharge profonde ponctuelle (ex. 1×/mois).**

La décharge profonde sert d'**ancre de recalage** ; grâce à la pondération `ΔSoC²`, elle corrige
l'estimation sans imposer une usure quotidienne à la batterie.

**Démonstration** (repro `doc/_repro_pseudo_rest.cpp`, capacité réelle 60 Ah) :

| Étape | C estimé | Écart |
|---|---|---|
| 6 cycles doux (10 %) | 50,7 Ah | −15,5 % |
| **+ 1 décharge profonde (50 %)** | **58,1 Ah** | **−3,2 %** |
| + 5 cycles doux de plus | 58,0 Ah | −3,3 % (stable) |

| DoD quotidien | Erreur C (ADC 20 mV) | Usure plomb |
|---|---|---|
| 5 % | ±32 % | nulle |
| 10 % | ±16 % | faible |
| 20 % | ±8 % | modérée |
| 50 % | ±4 % | élevée si quotidienne → **à réserver au ponctuel** |

**Cadence recommandée : recalage hebdomadaire.** Simulation (usage doux quotidien + 1 décharge
profonde 45 %/semaine, bruit ADC 20 mV) :

| Horizon | Écart C |
|---|---|
| Semaine 1-2 | ±5 % (convergence) |
| Semaines 5-20 | **±1 à ±4 %** (stable, oscille autour de −2 %) |

La fenêtre de régression de **32 points** est optimale (48/64 n'apportent rien et coûtent de la
RAM). Si une semaine de recalage est sautée, l'ancre profonde reste ~4 semaines dans la fenêtre.

### ⚠️ Limites d'observabilité (à connaître)

La précision dépend de l'**amplitude** du cycle et de la **qualité** des ancres :

| Situation | Conséquence |
|---|---|
| Cycle quotidien faible (ex. 5 % DoD) | `ΔSoC` petit → erreur amplifiée (1/ΔSoC). 20 mV d'erreur sur l'OCV plomb = **2 % de SoC** = **±30 % sur C** à 5 % de DoD. |
| Peu d'ancres (1-2/semaine) | La régression converge en **semaines/mois**, pas en jours. |
| Repos courts (< 30 min) | OCV non équilibré → biais (corrigé partiellement par l'extrapolation P1). |
| Rendement de charge | Les segments de **charge** sont exclus (rendement + maintien FLOAT faussent `ΔAh`). |

**Conclusion** : P1/P2 donnent une estimation **exploitable mais à confiance faible** pour ce
type d'installation. Pour une valeur fiable, seule une **décharge profonde contrôlée** (hors
portée d'un observateur passif) donne ±3-5 %.

## 🔌 Compatibilité & implémentation personnalisée
BatteryKalman **dépend uniquement de l'interface abstraite `BatteryModel`** — toute classe qui
l'implémente fonctionne.

```
BatteryKalman  ──dépend de──▶  Interface BatteryModel (abstraite)
                                     ▲
              ┌──────────────────────┼──────────────────────┐
        BatteryModels          Votre modèle          Autre lib / BMS
        (recommandé)            perso                compatible
```

### Implémenter son propre `BatteryModel`

Utile si vous avez un modèle batterie existant, des courbes OCV propriétaires, ou un BMS à
intégrer.

```cpp
#include <BatteryKalman.h>

class MonModele : public BatteryModel {
public:
    ChargeState detectChargeState(float V, float I, float C) override { /* BULK/FLOAT/REST/… */ }
    float correctVoltageToOCV(float V, float I, float T, float dt,
                              float* r_eff, float* v_pol) override { /* V = OCV - I·R - V_pol */ }
    float ocvToSoc(float V, float T) override { /* lookup OCV -> SoC */ }
    uint8_t getCellCount() override { return 4; }
    float getNominalCapacity() override { return 100.0f; }
    float getMinVoltage() override { return 10.0f; }
    float getMaxVoltage() override { return 16.8f; }
    const char* getTechnologyName() override { return "LiFePO4"; }
    bool isAutoDetect() override { return true; }
    bool isOcvReliable() override { return true; }
};
```

Exemple minimal (modèle linéaire) :

```cpp
class SimpleModel : public BatteryModel {
    ChargeState detectChargeState(float V, float I, float) override {
        if (I > 5.0f) return State_BULK;
        if (I < -5.0f) return State_DISCHARGE;
        if (I > -0.5f && I < 0.5f) return State_REST;
        return State_FLOAT;
    }
    float correctVoltageToOCV(float V, float I, float, float, float* r_eff, float*) override {
        if (r_eff) *r_eff = 0.01f; return V + I * 0.01f;
    }
    float ocvToSoc(float V, float) override {
        return constrain((V - 10.0f) / (16.8f - 10.0f) * 100.0f, 0.0f, 100.0f);
    }
    uint8_t getCellCount() override { return 4; }
    float getNominalCapacity() override { return 100.0f; }
    float getMinVoltage() override { return 10.0f; }
    float getMaxVoltage() override { return 16.8f; }
    const char* getTechnologyName() override { return "Simple"; }
    bool isAutoDetect() override { return false; }
    bool isOcvReliable() override { return false; }
};
```

### Checklist de compatibilité

```
☐ Classe dérivant de (ou implémentant) BatteryModel
☐ detectChargeState() -> ChargeState (State_BULK/FLOAT/REST/…)
☐ correctVoltageToOCV(V, I, T, dt, r_eff, v_pol)
☐ ocvToSoc(V, T)
☐ getCellCount() -> uint8_t
☐ getNominalCapacity() -> float (Ah)
☐ getMinVoltage() / getMaxVoltage() -> float
☐ getTechnologyName() -> const char*
☐ isAutoDetect() / isOcvReliable() -> bool
☐ Compile sans erreur et fonctionne avec BatteryKalman
```

### Résumé

| Aspect | Statut |
|--------|--------|
| BatteryModels obligatoire ? | ❌ Non (recommandé) |
| BatteryModels compatible ? | ✅ 100% |
| Implémentation perso ? | ✅ Possible |
| Autre librairie ? | ✅ Si elle implémente `BatteryModel` |
| Code BatteryKalman à changer ? | ❌ Non |
| Persistance compatible ? | ✅ `SoCData` + `KalmanState2D` |

## 🔄 Migration

### BatteryModels v1.1.x → v1.2.0 (renommages)

La v1.2.0 de BatteryModels a introduit des changements de terminologie (modélisation thermique
avancée). BatteryKalman a suivi :

| Avant (v1.1.x) | Après (v1.2.0+) |
|---|---|
| `detectMpptState()` | `detectChargeState()` |
| `MpptState` | `ChargeState` |
| `MPPT_UNKNOWN/DISCHARGE/BULK/ABSORPTION/FLOAT/REST/REST_LONG` | `State_*` (mêmes suffixes) |
| `battery.getMpptState()` / `getMpptStateStr()` | `battery.getChargeState()` / `getChargeStateStr()` |

> La v1.6.0 est **additive** : aucune migration requise. Nouveau : `getTuning()`, setters runtime
> (`setP/setQ/setR/setSegmentThresholds/setBatteryChange/setRestLong/setConfidence/resetTuning`)
> et consommation de `model->getKalmanTuning()` (BatteryModels v1.6.9+).
>
> La v1.7.0 est également **additive** : aucune migration requise. Nouveau :
> `applyRecommendedRestLong()` (R7), `enablePseudoRest()` (P1) et
> `getRegressionCapacity()`/`resetRegression()` (P2). Les correctifs K1–K12 changent
> l'état interne par défaut (`dC_dCycle = +0.0005`) — un état persisté reste lisible.

### v2.0.0 → v1.5 (`KalmanState` → `KalmanState2D`)

**Changement unique requis :**
```cpp
// Ancien
KalmanState kalmanState;

// Nouveau
KalmanState2D kalmanState;
```

**Persistance** — ajouter les nouveaux champs au JSON :
```cpp
doc["dC_dCycle"]   = kalmanState.dC_dCycle;
doc["P_CC"]        = kalmanState.P[0][0];
doc["P_Caging"]    = kalmanState.P[0][1];
doc["P_aging"]     = kalmanState.P[1][1];
doc["confidence"]  = kalmanState.confidence;
doc["R_estimated"] = kalmanState.R_estimated;
```

Voir `Exemples/avec_persistance/` pour la sérialisation complète.

### Nouvelles méthodes publiques (v1.5+)

```cpp
float getConfidence();        // [0, 1] — confiance du filtre
float getKalmanC();           // Ah — capacité estimée
float getKalmanAgingRate();   // Ah/cycle — taux vieillissement appris
float getKalmanP_CC();        // Variance de C
float getKalmanP_aging();     // Variance du taux aging
float getR_estimated();       // R adaptatif en ligne
float getR_measured();        // R courant utilisé par la mesure
```

## 📜 Historique v1.5 — design & release

> **Document historique** (fusion des anciens `IMPROVEMENTS.md` et `VERSION_1.5_RELEASE.md`).
> Les correctifs F1–F8 puis K1–K12/R1–R7 sont documentés dans [`CHANGELOG.md`](CHANGELOG.md).
> Sortie : **20 juillet 2026** · Version : **1.5** · État : production-ready.

La v1.5 a transformé BatteryKalman d'un filtre Kalman **1D scalaire** simplifié en un **EKF 2D**
théoriquement fondé, avec apprentissage automatique des paramètres.

### Améliorations clés (avant → après)

| Aspect | v2.0.0 | v1.5 (EKF 2D) |
|---|---|---|
| **État** | 1D (`C` seul) | 2D (`C` + `dC/dcycle`) |
| **Covariance** | scalaire `P` | matrice `P[2×2]` (avec covariance croisée) |
| **Process noise Q** | ❌ absent | ✓ `Q = diag(q_a·n + q_C, q_a)` |
| **R adaptatif** | heuristique `computeR()` | estimation en ligne (innovations) |
| **Aging** | post-hoc fixé (0.05%/cycle) | variable d'état **apprise** |
| **Prédiction temporelle** | ❌ non | ✓ oui (via cycles) |
| **Phases** | discrètes (hard-codées) | continues (data-driven) |
| **Confiance** | implicite | explicite `[0,1]` |

### Détails d'implémentation

- **État 2D** : `KalmanState2D { C_hat, dC_dCycle, P[2][2], R_estimated, R_measured, confidence, … }`
  remplace `KalmanState { C_hat, P, n_updates }`.
- **Process noise Q** : `predictKalman(delta_cycles)` fait croître `P` entre mesures (filtre moins
  surconfiant) ; `P` est plafonné à `p_init` pour éviter la divergence.
- **EKF 2D** : `applyKalmanUpdate2D()` avec Jacobien `H = [1, 0]`, gain 2D, mise à jour de la
  covariance croisée.
- **R en ligne** : `updateREstimate()` estime `R` depuis la variance des innovations (moyenne sur 5),
  lissage exponentiel (`R_SMOOTH_ALPHA`), borné `[R_MIN, R_MAX]`.
- **Confiance continue** : `conf = ½(min(1, n/10) + (1 − min(1, P/P_init)))`, mappée vers les phases
  legacy par `updatePhaseFromConfidence()`.

### Résultats attendus

| Métrique | v2.0.0 | v1.5 | Amélioration |
|----------|--------|------|--------------|
| Convergence | 3-5 cycles | 2-3 cycles | -33% |
| Accuracy | ±3-5% | ±3-5% | maintenu sur durée |
| Aging tracking | hard-codé | appris auto | adaptatif |
| Confiance | implicite | explicite [0,1] | observabilité |
| RAM | ~800 B | ~850 B | +6% |
| Flash | ~20 KB | ~22 KB | +10% |

### Breaking changes (v2.0.0 → v1.5)

- **Struct** : `KalmanState` → `KalmanState2D`.
- **Constructeur** : inchangé (4 paramètres).
- **API publique** : ~99% compatible, 6 nouveaux getters.
- **Persistance** : `saveState()`/`loadState()` à adapter pour `P[2×2]`, `dC_dCycle`, `confidence`.

### Notes de déploiement

1. **EEPROM/LittleFS** : adapter `saveState/loadState` pour `P[2][2]`.
2. **Rétro-compat** : un état v2.0.0 peut être chargé (`dC_dCycle` initialisé au défaut).
3. **RAM** : +50 octets (~6%) — acceptable en embarqué.
4. **Compilation** : C++11 minimum, aucun flag particulier.

## 🧪 Vérifications locales (sans matériel)

Des reproductions C++ autonomes (compilables avec `g++`) vérifient le comportement :

```bash
# Vérification de compilation (sans dépendances externes)
g++ -std=c++11 -I doc/_syntax -I src -fsyntax-only doc/_syntax/test_bk.cpp   # compilation

# Reproductions autonomes (sortie AVANT/APRES) — binaire dans /tmp
g++ -O0 -o /tmp/r doc/_repro_ring.cpp && /tmp/r                    # K1
g++ -O0 -o /tmp/r doc/_repro_restlong.cpp && /tmp/r                # K2/K3
g++ -O0 -o /tmp/r doc/_repro_window.cpp && /tmp/r                  # R1/R3
g++ -O0 -o /tmp/r doc/_repro_noise.cpp && /tmp/r                   # R6
g++ -O0 -o /tmp/r doc/_repro_aging.cpp && /tmp/r                   # K7
g++ -std=c++11 -I doc/_syntax -I src -o /tmp/r doc/_repro_restlong_profile.cpp && /tmp/r   # R7
g++ -std=c++11 -I doc/_repro_mocks -I src -o /tmp/r doc/_repro_restore.cpp && /tmp/r        # K8/K9
g++ -O0 -o /tmp/r doc/_repro_wrap.cpp && /tmp/r                    # K11
g++ -std=c++11 -I doc/_repro_mocks -I src -o /tmp/r doc/_repro_pseudo_rest.cpp && /tmp/r    # P1/P2
g++ -std=c++11 -I doc/_repro_mocks -I src -o /tmp/r doc/_repro_fusion.cpp && /tmp/r         # fusion SoC (v1.7.2)
```

`doc/_syntax/` contient des en-têtes stub (compile check sans dépendances) ;
`doc/_repro_mocks/` fournit des mocks fonctionnels (horloge Arduino, compteur coulomb,
`BatteryModel`) pour `_repro_restore.cpp`. Chaque repro imprime un verdict `AVANT`/`APRES`.

> ⚠️ Les mocks `doc/_syntax/BatteryModels.h` et `doc/_repro_mocks/BatteryModels.h` doivent
> déclarer la signature **7 arguments** de `correctVoltageToOCV` (avec `float soc_ref = NAN`),
> sinon `test_bk.cpp`, `_repro_restore.cpp` et `_repro_pseudo_rest.cpp` ne compilent plus.

### Version de la librairie

Le header déclare sa version dans une **source unique**, agrégée par un projet hôte
(par ex. une page `/version` qui affiche les versions de toutes les libs Fo170) :

```cpp
#define __BATTERYKALMAN_H_VERSION__ "1.7.4"
```

`library.json` doit rester aligné sur cette macro (elle fait foi).

## 🧮 Formules exactes (référence)

> Identiques au code (`src/BatteryKalman.h`). Convention : **`I > 0` = charge**.

### 1. SoC coulomb (intégration du compteur)

```
ΔAh        = Ah_compteur(k) − Ah_compteur(k−1)         [Ah]  (déjà corrigé de η par Coulomb)
SoC_coulomb ← clamp( SoC_coulomb + (ΔAh / C_ref)·100 , 0 ; 100 )   [%]
C_ref       = C_hat  si le Kalman est initialisé et C_hat > 1 Ah, sinon C_nominal
```

### 2. Fusion SoC (`fuseSoC`)

```
SoC_fusionné = α·SoC_coulomb + (1 − α)·SoC_voltage

α = 0,00  State_FLOAT
α = 0,02  State_REST_LONG
α = 0,10  State_REST
α = 1,00  BULK / ABSORPTION / DISCHARGE   (défaut)
```
> Depuis la v1.7.2, α **ne dépend plus de la confiance** : un changement de phase ne provoque plus
> de saut de SoC.

### 3. Compteurs Ah publiés (v1.7.3)

```
si ΔAh < 0 :  Ah_total_discharged += −ΔAh
sinon      :  Ah_total_charged    +=  ΔAh
```
(plus de `0,98` codé en dur : le rendement est porté par `Coulomb`)

### 4. Correction tension → OCV

```
V_OCV = correctVoltageToOCV(V_terminal, I, T, Δt, …, soc_ref = SoC_coulomb)
SoC_voltage = ocvToSoc(V_OCV, T)
```
La résistance et la polarisation sont retirées par `BatteryModel` (cf. README BatteryModels §3).
`SoC_voltage` sert d'**ancre** au repos et en fin de charge, jamais de référence en charge/décharge.

### 5. Cycles et vieillissement

```
Δcycles       = cycles_partiel(k) − cycles_partiel(k−1)
cycles_cumul += Δcycles                        (sinon Δcycles ≈ 0 et dC/dcycle n'est jamais appris)
```

### 5 bis. Pseudo-repos — ancre OCV courte (P1, durcie en v1.7.4)

Détection d'un repos court, puis validation de l'ancre **uniquement si la durée est suffisante** :

```
repos court  :  |I| < pseudo_rest_i_max                     (défaut 0,05 A)
début        :  premier échantillon consécutif sous le seuil (mémorise V_start et t0)
fin          :  |I| ≥ pseudo_rest_i_max

ancre VALIDE ⟺ (n_échantillons ≥ pseudo_rest_min_samples)  ET  (t_fin − t0 ≥ pseudo_rest_min_ms)

correction de relaxation (si pseudo_rest_tau_min > 0) :
  pente  = (V_fin − V_start) / (durée en minutes)
  corr   = clamp(pente · pseudo_rest_tau_min , ±PSEUDO_REST_MAX_CORR)
  V_ancre = V_fin + corr
```

| Paramètre | Défaut | Rôle |
|---|---|---|
| `pseudo_rest_i_max` | 0,05 A | Seuil de « courant nul » |
| `pseudo_rest_min_samples` | 2 | Nb d'échantillons consécutifs |
| **`pseudo_rest_min_ms`** | **30 000 ms** | **Durée minimale (v1.7.4)** — `0` = critère par échantillons seul |
| `pseudo_rest_tau_min` | 0 (désactivé) | τ de relaxation pour extrapoler l'OCV |
| `PSEUDO_REST_MAX_CORR` | 0,150 V | Garde-fou sur la correction |

> ⚠️ **Pourquoi la durée est indispensable** : à 10 Hz, `min_samples = 2` ne représente que
> **0,2 s**, alors que la relaxation de polarisation du plomb dure des dizaines de secondes.
> Sans le critère de durée, l'ancre est prise sur une tension encore loin de l'OCV.

### 6. Confiance (phase)

```
confiance < 0,30  → PHASE_COARSE
confiance < 0,60  → PHASE_REFINE
sinon             → PHASE_TRACK
confiance = 0     → PHASE_BOOTSTRAP (avant initialisation)
```

---

## 📚 Références

- **EKF** : Bar-Shalom, Li, Kirubarajan — *Estimation with Applications to Tracking and Navigation*, 2001.
- **Kalman Filtering for Battery** : Wei He et al. — *State-of-charge estimation for Li-ion batteries*, 2019.
- **Process Noise** : Welch & Bishop — *An Introduction to the Kalman Filter*, 2006.
- **Comparaison théorie/variantes** : [`doc/comparaison_kalman.md`](doc/comparaison_kalman.md) (sources Wikipedia KF/EKF, kalmanfilter.net, MathWorks, papiers AEKF/DEKF/UKF).
- **Réglage REST_LONG** : [`doc/rest_long_tuning.md`](doc/rest_long_tuning.md).

### Sources en ligne (vérifiées le 2026-10-18, HTTP 200)

| Sujet | Source | Usage dans cette lib |
|---|---|---|
| Filtre de Kalman | [Wikipedia — Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) | Prédiction/correction, gain, covariance |
| Filtre de Kalman étendu | [Wikipedia — Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) | Linéarisation `H`, structure du gain |
| État de charge | [Wikipedia — State of charge](https://en.wikipedia.org/wiki/State_of_charge) | Définitions SoC/DoD, comptage de coulombs vs OCV |
| Gestion de batterie | [Wikipedia — Battery management system](https://en.wikipedia.org/wiki/Battery_management_system) | Rôle de l'estimation SoC/SoH |
| Rendement faradique | [Wikipedia — Faraday efficiency](https://en.wikipedia.org/wiki/Faraday_efficiency) | Justification du rendement appliqué par `Coulomb` (v1.7.3) |
| Rendement coulombique plomb | [Battery University — BU-808c](https://batteryuniversity.com/article/bu-808c-coulombic-and-energy-efficiency-with-the-battery) | Ordre de grandeur η ≈ 0,90 (plomb) |
| Batterie plomb-acide | [Wikipedia — Lead–acid battery](https://en.wikipedia.org/wiki/Lead%E2%80%93acid_battery) | Comportement de charge, palier float |
| Comptage de cycles (Rainflow) | [Wikipedia — Rainflow-counting algorithm](https://en.wikipedia.org/wiki/Rainflow-counting_algorithm) | Observateur P2 / usure par cycles |

> ℹ️ Aucune conclusion de cette lib ne repose sur une source inaccessible : les URL testées en
> erreur (HTTP 4xx/5xx) ont été écartées.
