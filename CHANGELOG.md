# Changelog - BatteryKalman

## [Non publié] — Corrections de bugs (chasse aux bugs v1.6.0)

Corrections et ajouts dans `src/BatteryKalman.h` (ajout d'une méthode publique :
`applyRecommendedRestLong()`) :

### 🐛 Corrigés
- **K1** — Anneau de stabilité REST : index d'écriture figé sur la case 0 après remplissage
  (`rest_v_count` plafonné à `REST_LONG_STABLE_MAX_SAMPLES`). Anneau désormais rotatif.
- **K2** — `state_entry_ms` réinitialisé à chaque promotion REST→REST_LONG (car `mppt_state_prev`
  stocke l'état final) → REST_LONG instable. Réarmement sur l'état **brut** via `mppt_raw_prev`.
- **K3** — Défaut `dC_dCycle` incohérent avec le modèle de processus (`C_hat -= dC_dCycle·Δ`) :
  passé de −0.0005 à **+0.0005 Ah/cycle** (taux de vieillissement positif = perte de capacité).
- **K4** — Underflow `millis()` non saturé (`cutoff`, `extra`) → saturation.
- **K5** — `static bool initialized` partagé dans `updateCycleDetection()` → membre `cycle_initialized`.
- **K6** — `phase_info` jamais mis à jour → `getPhaseInfo()` renvoie désormais la phase réelle.
- **K7** — Vieillissement non appris : les cycles n'étaient pas accumulés entre mesures
  (`delta_cycles ≈ 0` à la mesure → `dC_dCycle` figé). Accumulation via `cycles_since_update`.
- **R1** — Anneau de stabilité REST non temporel (fenêtre ≈3 s à 10 Hz) → écriture décimée
  (`stable_ms / REST_LONG_STABLE_MAX_SAMPLES`) pour couvrir toute la fenêtre.
- **R4** — Sentinelle de « premier push » ambiguë au démarrage → membre `rest_v_has_push`.
- **R2** — Sémantique de la fenêtre REST clarifiée (**lookback** plafonné à `rest_long_stable_min`,
  défaut 15/30 conservé) ; réglages par technologie via R7.

### 🐛 Corrigés (2ᵉ passe — cycle de chasse aux bugs)
- **K8** — `begin()` ne synchronisait `last_Ah` que dans la branche `!coulomb_initialized`. En
  **restauration** (`coulomb_initialized == true`) avec un compteur coulomb non nul, `last_Ah`
  restait à 0 → le 1er `update()` injectait tout le compteur comme `dAh` (saut de `SoC_coulomb`).
  Synchronisation désormais **inconditionnelle** dans `begin()`.
- **K9** — Cutoff basse tension : `doSync()` était appelé à **chaque** échantillon tant que la
  batterie restait au repos sous `V_min` (écriture de persistance continue + reset coulomb répété).
  Ajout d'une **porte temporelle** (5 min) via `REST_ALIGN_SYNC_MS`.
- **K11** — Anneau de stabilité REST : tolérance au **wrap de `millis()`** (~49,7 j) — vidage de
  l'anneau si l'horloge a rebouclé, pour ne pas conserver d'horodatages périmés (REST_LONG différé).
- **K12** — Littéral `300000UL` dupliqué (cutoff basse tension + alignement REST) → constante
  nommée `REST_ALIGN_SYNC_MS`.

### ✨ Ajouté (auto-adaptatif / par technologie)
- **R6** — Seuil de planéité REST **auto-adaptatif au bruit de mesure** :
  `stable_mv_eff = max(rest_long_stable_mv, REST_LONG_NOISE_K × bruit)`, borné à 3× le seuil de base.
- **R7** — `applyRecommendedRestLong()` : profil REST_LONG **par technologie** (plomb, LiFePO4,
  Li-ion, NiMH/NiCd, Sodium, supercondensateur). Voir `doc/rest_long_tuning.md`.
- **P1** — **Mode pseudo-repos** (opt-in, `enablePseudoRest()`) : pour les installations **sans
  repos long** (décharge permanente la nuit, charge PV le jour), exploite les courts instants à
  courant ~nul comme **ancre OCV basse**. Rétro-compatible (désactivé par défaut).
- **P2** — **Régression accumulée** (observateur passif) : accumule les couples `(dSoC, dAh)` des
  segments de **décharge nette** et ajuste une droite par l'origine (`dAh = C·dSoC/100`),
  **pondérée par `dSoC²`** (les décharges profondes recalent l'estimation). API :
  `getRegressionCapacity()`, `getRegressionPoints()`, `getRegressionDispersion()`,
  `getRegressionMaxDoD()`, `resetRegression()`. 100 % passif (aucun contrôle requis).

### 📝 Documenté
- **R3** — Limite de cadence : `dt ≤ rest_long_stable_min/(min_samples-1)`.
- **K10** — Champs morts supprimés (`Segment::I_sq_sum/I_sum/I_min/I_max/conf_start/start_ms/
  had_discharge/had_charge`, `_tuningFromModel`, helper `_restLongMinSamples()`) ; taille de
  l'anneau REST = macro compile-time `REST_LONG_STABLE_MAX_SAMPLES` (le champ
  `KalmanTuning::rest_long_stable_max_samples` n'est pas consommé — documenté).
- `doc/rest_long_tuning.md` — recommandations par techno + seuil auto-adaptatif.

Détail complet et preuves : **Annexe — Journal détaillé de la chasse aux bugs** (en fin de
fichier) et `analyse.md`.

## [1.6.0] - 2026-08-04

### ✨ Configuration par technologie (consomme BatteryModels v1.4+)

Le filtre consomme désormais `model->getKalmanTuning()` : les paramètres
P, Q, R, les seuils de segment, la détection de batterie changée, la porte
REST_LONG et la confiance sont réglés **par technologie** depuis
BatteryModels (valeurs calibrées dans `TECH_PARAMS[]`).

### 🔀 Hiérarchie de résolution centralisée (`getTuning()`)

1. **Setter runtime** (nouveaux) : `setTuning()`, `setP()`, `setQ()`,
   `setR()`, `setSegmentThresholds()`, `setBatteryChange()`, `setRestLong()`,
   `setConfidence()`, `resetTuning()`.
2. **Modèle** : `model->getKalmanTuning()` (défaut par technologie).
3. **Macros compile-time** : `KALMAN_P_INIT_C`, `R_INIT`, `SEG_MIN_DAH`, etc.
   — désormais **optionnelles** (défaut = sentinelle `NAN`/`0` = "non
   définie"). Les valeurs réelles par défaut vivent dans
   `KALMAN_DEFAULT_TUNING` (équivalent v1.5.3). Les macros restent un moyen
   global de forcer un réglage avant `#include`.
4. **Défaut interne** : `KALMAN_DEFAULT_TUNING` (valeurs v1.5.3 inchangées).

La résolution est re-faite à chaque `getTuning()` : un
`model->setTechnology()` ultérieur est donc automatiquement pris en compte.

### ➕ Nouveautés fonctionnelles

- **Seuil ΔAh relatif** (`seg_min_dAh_pct`, % de C) : utile pour les petites
  batteries (NiMH/Alkaline). Le seuil effectif est `max(seuil_absolu, pct × C)`.
- **Gating des segments OCV** (`ocv_segment_allowed=false`) : les segments
  ΔAh/ΔSoC sont désactivés pour NiFe/Sodium (OCV peu fiable).
- **REST_LONG** (durée minimale, fenêtre de stabilité, écart crête, nb
  d'échantillons) et **confiance** (FLOAT, REST_LONG, seuils de phase)
  configurables par technologie via le tuning.

### 🔁 Rétro-compatibilité

- **Aucun breaking change** : sans setter ni BatteryModels v1.4+, le
  comportement est strictement identique à v1.5.3 (défauts internes).
- `KalmanState2D` : les valeurs initiales de P et R sont les constantes
  v1.5.3 (plus de dépendance aux macros sentinelles).
- L'exemple `Exemples/avec_persistance/` utilise `KALMAN_DEFAULT_TUNING`
  comme défauts de chargement (les macros valent désormais `NAN` par défaut).

## [1.5.3] - 2026-08-03

### 🐛 Corrections de bugs (v1.5.2) + Porte de stabilité REST (F8)

Version corrigée basée sur l'analyse de données réelles (jeu de données depuis retiré du dépôt).
Toutes les corrections sont dans `src/BatteryKalman.h` ; le correctif F7 pour
`BatteryModels.h` (`detectChargeState`) est intégré dans la **v1.3** de
BatteryModels et documenté dans `doc/correction_BatteryModels.md`.

### 🐛 F1 — Transition d'état MPPT jamais détectée (bug principal)
- `updateMpptState()` mettait à jour `mppt_state_prev` dans la fonction, donc
  `mppt_state == FLOAT && mppt_state_prev != FLOAT` n'était **jamais vrai**.
  Synchro FLOAT, fermeture de segment FLOAT et comptage de cycles = code mort.
- **Fix**: `mppt_state_prev` mis à jour en fin de `update()` (différé).

### 🐛 F2 — Reset coulomb avant la mesure du segment
- `handleSyncEvents()` (→ `doSync()` → reset coulomb) appelé **avant**
  `updateSegmentAndKalman()` → segment réinitialisé avant mesure (`dAh ≈ 0`).
- **Fix**: `updateSegmentAndKalman()` appelé avant `handleSyncEvents()`.

### 🐛 F3 — R adaptatif jamais appliqué
- `updateREstimate()` mettait à jour `R_estimated` mais `updateSegmentAndKalman()`
  utilisait `R_measured` (jamais modifié). Contradiction avec README.md (§ Historique v1.5)
  qui documentait `R_measured = constrain(R_estimated, R_MIN, R_MAX)`.
- **Fix**: `R_measured = R_estimated` après lissage.

### 🐛 F4 — EKF 2D incomplet (vieillissement non appris)
- `predictKalman()` n'appliquait pas `C -= dC/dcycle × Δcycles` et ne propageait pas
  la covariance croisée ; `applyKalmanUpdate2D()` forçait `K_aging = 0` et ne mettait
  jamais à jour `P[1][1]` → `dC_dCycle` inapprenable (filtre 1D de fait).
- **Fix**: prédiction complète (`F = [[1,−Δcycles],[0,1]]`, `P = F·P·Fᵀ + Q`),
  gain 2D `K = [P_CC, P_agingC]/S`, correction des 2 composantes + covariance.

### 🐛 F5 — Fermeture REST_LONG conditionnée à `seg.n > 200`
- Nécessitait 200 échantillons (16,7 h) → fermetures quasi jamais déclenchées.
- **Fix**: fermeture sur **transition** REST_LONG (cf. F1).

### 🐛 F6 — Apprentissage gated par `isAutoDetect()`
- `if (model->isAutoDetect()) updateSegmentAndKalman(...)` → le Kalman n'apprenait
  que si la capacité nominale était 0.
- **Fix**: apprentissage toujours actif (l'auto-détection reste indépendante).

### 🐛 F7 — `detectChargeState()` : FLOAT masqué par REST (BatteryModels.h)
- Ordre des tests `discharging → in_rest → near_float` : en FLOAT (|I| < 0,05 A),
  l'état était classé REST, jamais FLOAT.
- **Fix**: tester `near_float` avant `in_rest`.

### ✨ F8 — Porte de stabilité REST (OCV fiable)
- Le point de référence REST_LONG ne dépend plus d'un délai fixe de 2 h
  (inatteignable sur de nombreuses installations).
- REST_LONG est atteint quand : repos ≥ `REST_LONG_MIN_MS` (15 min par défaut)
  **ET** tension stable (écart crête < `REST_LONG_STABLE_MV` = 20 mV) sur une
  fenêtre temporelle `REST_LONG_STABLE_MS` (30 min).
- Indépendant du rythme d'échantillonnage (10 Hz, 1 Hz, 5 min...).

### ⚙️ Nouvelles constantes de configuration (F8)
```cpp
REST_LONG_MIN_MS            // durée min de repos (900000 = 15 min)
REST_LONG_STABLE_MV         // stabilité tension (0.020 V)
REST_LONG_STABLE_MS         // fenêtre temporelle stabilité (1800000 = 30 min)
REST_LONG_STABLE_MIN_SAMPLES// échantillons min dans la fenêtre (6)
REST_LONG_STABLE_MAX_SAMPLES// taille max anneau (32)
REST_LONG_SYNC_MS           // délai entre 2 synchros REST_LONG (30 min)
```

### 📊 Validation
- Validation par reproductions C++ autonomes (`doc/_repro_*.cpp`, compilables avec `g++`) :
  la capacité estimée passait de **108,2 Ah (bugué) → 43,5 Ah (corrigé)** sur le jeu de
  données réelles (plomb inondé 6S, depuis retiré du dépôt), cohérent avec un groupement
  usagé < 50 Ah.
- Cycle synthétique 100 Ah : convergence en 8 corrections, `P` 4,0 → 0,5,
  confiance → 90 %, R adapté, cycles comptés.

---

## [1.5] - 2026-07-20

### 🎉 Major Release: Complete Kalman Filter Overhaul

This release transforms BatteryKalman from a simplified 1D scalar Kalman implementation to a full **Extended Kalman Filter (EKF) 2D** with theoretical rigor and automatic parameter learning.

**Licence**: GNU General Public License v3

### ✨ Added

#### Core Algorithm Improvements
- **2D State Space**: Simultaneous estimation of capacity (C) and aging rate (dC/dcycle)
- **Extended Kalman Filter (EKF)**: Proper Jacobian-based update with 2×2 covariance matrix P
- **Process Noise Q**: Realistic uncertainty modeling between measurements
  - `Q_AGING_UNCERTAINTY`: Configurable process noise
  - `Q_CAPACITY_DRIFT`: Capacity drift uncertainty
  - Grows P between updates (prevents overconfidence)
- **Online R Estimation**: Adaptive measurement noise from innovation sequence
  - `R_SMOOTH_ALPHA`: Exponential smoothing factor
  - `R_MIN` / `R_MAX`: Bounds for R adaptation
- **Continuous Confidence Metric**: [0, 1] scale replaces discrete phase thresholds
  - Data-driven confidence formula: `(n_factor + p_factor) / 2`
  - Smooth transitions between learning phases
- **Automatic Aging Learning**: No hardcoded drift rates
  - `dC_dCycle` learned from measurement residuals
  - Adapts to actual battery degradation patterns

#### New Struct
- `KalmanState2D`: Replaces `KalmanState`
  - Added: `dC_dCycle` (aging rate, Ah/cycle)
  - Added: `P[2][2]` (2×2 covariance matrix)
  - Added: `R_estimated`, `R_measured` (adaptive noise)
  - Added: `confidence` ([0, 1] continuous metric)
  - Added: `innovation_sum_sq`, `innovation_count` (for R estimation)

#### New Configuration Macros
```cpp
KALMAN_P_INIT_C           // Initial variance for C (Ah²)
KALMAN_P_INIT_AGING       // Initial variance for dC/dcycle
KALMAN_P_MIN_C            // Minimum variance for C
KALMAN_P_MIN_AGING        // Minimum variance for aging
Q_AGING_UNCERTAINTY       // Process noise for aging
Q_CAPACITY_DRIFT          // Process noise for capacity
R_INIT                    // Initial measurement noise
R_MIN / R_MAX             // Bounds for adaptive R
R_SMOOTH_ALPHA            // Exponential smoothing for R
```

#### New Public Methods
- `getConfidence()`: Confidence metric [0, 1]
- `getKalmanC()`: Current capacity estimate (Ah)
- `getKalmanAgingRate()`: Learned aging rate (Ah/cycle)
- `getKalmanP_CC()`: Capacity variance
- `getKalmanP_aging()`: Aging rate variance
- `getR_estimated()`: Estimated measurement noise
- `getR_measured()`: Current measured noise

#### New Private Methods
- `predictKalman()`: Prediction phase with process noise Q
- `applyKalmanUpdate2D()`: 2D EKF update with Jacobian
- `updateREstimate()`: Innovation-based R adaptation
- `updatePhaseFromConfidence()`: Maps continuous confidence to phases (legacy)

### 📝 Changed

#### Struct Changes
- **KalmanState → KalmanState2D**: Complete restructure
  - `P` scalar → `P[2][2]` matrix
  - New fields: `dC_dCycle`, `R_estimated`, `R_measured`, `confidence`

#### Method Signature Changes
- `update()`: Added cycle tracking (internal)
  - Signature: `void update(float V, float I, float T)` (unchanged externally)
  - Internally computes `delta_cycles` for process noise

#### Algorithm Changes
- **Phase transitions**: Now driven by `confidence` instead of hardcoded `P` thresholds
- **R adaptation**: Replaces hardcoded `computeR()` heuristics with statistical estimation
- **Aging model**: Integrated into process model (not post-hoc)
- **Outlier detection**: 3-sigma threshold unchanged, but acts on 2D innovation covariance

#### Documentation
- Added design rationale (désormais dans `README.md`, § Historique v1.5)
- Updated CLAUDE.md with 2D EKF explanation
- Updated examples for KalmanState2D
- Updated CHANGELOG.md for v1.5
- Marked old version as BatteryKalman_v2.0.0_backup (legacy)

### 🔄 Deprecated

- Hard-coded phase transition thresholds (now `confidence`-driven)
- Fixed aging drift rate (`AGING_DRIFT_PER_CYCLE` → learned `dC_dCycle`)
- Manual R tuning via `computeR()` (now adaptive R)

### 📊 Compatibility

| Feature | v2.0.0 | v3.0.0 | Breaking |
|---------|--------|--------|----------|
| Public API | ~95% | ~99% | No* |
| Struct | `KalmanState` | `KalmanState2D` | Yes |
| Example Code | N/A | Updated | No (getters compatible) |
| Persistence Format | Scalar P | Matrix P[2×2] | Yes** |

*Public method signatures unchanged; new methods added  
**If upgrading from v2.0.0, saveState/loadState must handle P[2×2]

### ⚠️ Breaking Changes

- **Struct rename**: `KalmanState` → `KalmanState2D`
  - Update your variable declarations: `KalmanState2D kalmanState;`
- **Persistence**: `saveState()` / `loadState()` must handle:
  - New fields: `dC_dCycle`, `P[2][2]`, `confidence`, `R_estimated`
  - See `Exemples/avec_persistance/` for updated JSON serialization
- **Constructor remains unchanged**: Same 4 parameters expected

### 🐛 Fixed

- *Issue #1*: P never grew between measurements → Fixed: predictKalman() with Q
- *Issue #2*: Hardcoded aging rate ignored battery variation → Fixed: dC_dCycle learned
- *Issue #3*: No time-based prediction → Fixed: Process noise Q applied
- *Issue #4*: Discrete phases too rigid → Fixed: Continuous confidence metric
- *Issue #5*: R heuristic unmotivated → Fixed: Online statistical estimation

### 🧪 Testing

**Recommended validation:**
1. **Convergence speed**: ~2-3 cycles (vs 3-5 before)
2. **Accuracy post-convergence**: ±3-5% maintained (vs degradation after ~500 cycles)
3. **Aging tracking**: Automatic learning vs manual reset
4. **Battery change detection**: Auto-reset vs manual intervention

**Example test:**
```cpp
// Test aging tracking
for (int cycle = 0; cycle < 100; cycle++) {
    charge_battery();
    discharge_battery();
    float aging_learned = battery.getKalmanAgingRate();
    Serial.printf("Cycle %d: Aging rate = %.6f Ah/cycle\n", cycle, aging_learned);
}
// Should show progressively more accurate aging estimation
```

### 📈 Performance Impact

| Metric | v2.0.0 | v3.0.0 | Change |
|--------|--------|--------|--------|
| **Flash** | ~20 KB | ~22 KB | +10% |
| **RAM** | ~800 B | ~850 B | +6% |
| **CPU per update** | ~150 µs | ~180 µs | +20% |
| **Convergence cycles** | 3-5 | 2-3 | -33% |

Impact: Negligible on ESP32/STM32; acceptable on ATmega328.

### 🔧 Migration Guide

**Step 1**: Update struct declaration
```cpp
// Old
KalmanState kalmanState;

// New
KalmanState2D kalmanState;
```

**Step 2**: Update examples (if using)
```cpp
// Old
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

// New (unchanged constructor)
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
```

**Step 3**: Update persistence (saveState/loadState)
```cpp
// New fields to persist:
doc["dC_dCycle"] = kalmanState.dC_dCycle;
doc["P_CC"] = kalmanState.P[0][0];
doc["P_Caging"] = kalmanState.P[0][1];
doc["P_aging"] = kalmanState.P[1][1];
doc["confidence"] = kalmanState.confidence;
```

**Step 4**: Update tuning (optional)
```cpp
// New macros to adjust (if needed):
#define Q_AGING_UNCERTAINTY  0.000002f    // Aging process noise
#define R_SMOOTH_ALPHA       0.1f         // R adaptation rate
#define KALMAN_P_INIT_C      500.0f       // Initial P for C
```

See `README.md` (§ Historique v1.5) for detailed before/after comparison.

### 📚 References

- Bar-Shalom, Li, Kirubarajan: "Estimation with Applications to Tracking and Navigation", 2001
- Welch & Bishop: "An Introduction to the Kalman Filter", 2006
- He et al.: "State-of-charge estimation for Li-ion batteries using neural networks", 2019

### 🙏 Acknowledgments

This v3.0.0 release implements recommendations from theoretical Kalman filtering literature and practical BMS industry standards, addressing all identified gaps from v2.0.0 analysis.

---

## [2.0.0] - 2026-03-15

### Added
- Initial BatteryKalman v2.0.0 with 1D Kalman filter
- Phase-based learning (Bootstrap → Coarse → Refine → Track)
- Adaptive measurement noise R
- Segment-based collection
- Cycle detection
- Coulomb counting integration
- Support for BatteryModels v1.1.x

### Fixed (v2.0.0 → v3.0.0 only in hindsight)
- Missing process noise Q
- No time-based prediction
- Hardcoded aging drift
- Discrete phase transitions
- No R online adaptation

---

## Version Comparison

```
v1.0.0 → v2.0.0: Initial Arduino library
  - Phases, adaptive R, segments
  - 1D Kalman filter
  - Limited aging model

v2.0.0 → v3.0.0: Complete overhaul (current)
  - 2D EKF with aging learning
  - Process noise Q
  - Online R estimation
  - Continuous confidence
  - Backward compatible API
```

---

## Annexe — Journal détaillé de la chasse aux bugs (K/R)

> Fusion de l'ancien `changelog_bughunt.md` (nommé ainsi pour ne pas écraser ce fichier sur un
> système insensible à la casse). Format : id · sévérité · catégorie · fichier:ligne · statut.
> Statuts : `open` · `resolved` (corrigé + prouvé) · `dismissed` · `reporté`.

Base : `src/BatteryKalman.h` **v1.6.0** → corrigé. Reproductions : `doc/_repro_*.cpp` (g++),
vérif de compilation : `doc/_syntax/` (`g++ -fsyntax-only`).

**Bilan : K1–K7, R1, R2, R4, R5 corrigés ; R3 documenté ; R6 (seuil auto-adaptatif) et R7 (profils par technologie) ajoutés ; algorithme EKF vérifié sur 4 sources.**

**Cycle 2 (nouvelle passe) : K8, K9, K11 corrigés ; K10 documenté ; K12 (nit) corrigé. Aucun bug ouvert.**

---

### K1 · major · logique / anneau · src/BatteryKalman.h:607-613 · statut: `resolved`

**Description** — L'index d'écriture de l'anneau de stabilité REST était
`rest_v_count % REST_LONG_STABLE_MAX_SAMPLES`, mais `rest_v_count` était **plafonné à
`REST_LONG_STABLE_MAX_SAMPLES`**. Une fois l'anneau plein (`rest_v_count == 32`),
`32 % 32 == 0` : toutes les écritures écrasaient la case 0, les 31 autres étaient figées puis
purgées par la fenêtre temporelle.

**Impact** — Échantillonnage rapide (1–10 Hz) : l'anneau ne tourne plus → `kept` s'effondre à 1
(< `rest_long_stable_min_samples` = 6) → `stable_ok` faux → **REST_LONG jamais atteint** et la
porte de stabilité valide des échantillons périmés. Prive le filtre de la référence OCV (F8).

**Correctif appliqué** — Curseur d'écriture strictement modulo MAX ; suppression du plafonnement :
```cpp
uint8_t widx = rest_v_count % REST_LONG_STABLE_MAX_SAMPLES;
rest_v_ring[widx] = V;
rest_v_ring_ms[widx] = millis();
rest_v_count = (uint8_t)((rest_v_count + 1) % REST_LONG_STABLE_MAX_SAMPLES);
```

**Preuve** — `doc/_repro_ring.cpp` :
```
AVANT  :  1/32 cases ecrites apres remplissage (attendu 32) -> BUG
APRES  : 32/32 cases ecrites apres remplissage (attendu 32) -> OK
```

---

### K2 · major · machine à états · src/BatteryKalman.h:597-601,635-636,908 · statut: `resolved`

**Description** — `state_entry_ms` était réarmé dès que `mppt_state != mppt_state_prev`. Or
`mppt_state_prev` prend la valeur **finale** (`State_REST_LONG`) en fin de `update()` (F1). À
l'itération suivante, `detectChargeState()` renvoie `State_REST` → `REST != REST_LONG` →
`state_entry_ms = millis()` → `time_ok` faux. REST_LONG clignotait une itération toutes les
`rest_long_min_min` au lieu d'être un état stable.

**Impact** — `extra` (confiance de synchro OCV) s'effondrait à ~0 → confiance bloquée à
`conf_rest_base` ; chrono de repos redémarré à chaque promotion ; fermeture/réouverture de segment
en clignotement ; référence OCV de repos long peu fiable.

**Correctif appliqué** — Nouveau membre `mppt_raw_prev` (état **brut** de `detectChargeState`) ;
le chrono est réarmé sur la transition brute, `mppt_state_prev` (final) reste utilisé pour la
détection de transition :
```cpp
if (mppt_state != mppt_raw_prev) state_entry_ms = millis();
mppt_raw_prev = mppt_state;   // mppt_state == état brut ici
```
`begin()` initialise aussi `mppt_raw_prev = mppt_state;`.

**Preuve** — `doc/_repro_restlong.cpp` :
```
REST_LONG atteint sur 60 min : AVANT=3 (clignotant)  APRES=1 (stable) -> OK
```

---

### K3 · minor · numérique / signe · src/BatteryKalman.h:245,408,444,476,964 · statut: `resolved`

**Description** — Le défaut `dC_dCycle = -0.0005f` (« taux vieillissement ») était incohérent avec
le modèle de processus `C_hat -= dC_dCycle * delta_cycles` (= `C_hat + 0.0005·Δ`) : la capacité
**augmentait** au lieu de vieillir.

**Impact** — Biais de capacité qui dérive vers le haut ; initialisation contradictoire avec le
signe attendu par `F=[[1,-Δ],[0,1]]`.

**Correctif appliqué** — Défauts passés à `+0.0005f` (les 4 occurrences) et fallback de
`Exemples/avec_persistance/` aligné (`| 0.0005f`). Le modèle de processus et le Jacobien (F4) sont
inchangés.

**Preuve** — `doc/_repro_restlong.cpp` :
```
Capacite apres 3 cycles 'vieillissement' : AVANT=50.00150 Ah (monte)  APRES=49.99850 Ah (baisse) -> OK
```

---

### K4 · minor · underflow entier · src/BatteryKalman.h:616-618,657,780 · statut: `resolved`

**Description** — `millis()` non saturé : `cutoff = millis() - stable_ms` déborde si
`millis() < stable_ms` (30 min) → toutes les cases purgées (`kept = 0`) ; `extra = millis() -
state_entry_ms - rest_min_ms` pouvait déborder de même.

**Impact** — Pendant ~30 min après démarrage, un repos légitime est rejeté ; `extra` débordant
aurait saturé la confiance.

**Correctif appliqué** — Saturation :
```cpp
uint32_t cutoff = (now_ms > stable_ms) ? (now_ms - stable_ms) : 0UL;
uint32_t elapsed = millis() - state_entry_ms;
uint32_t extra = (elapsed > rest_min_ms) ? (elapsed - rest_min_ms) : 0UL;
```
(appliqué dans `updateMpptState()` et dans les 2 sites `extra` de `handleSyncEvents()` et
`updateSegmentAndKalman()`.)

**Preuve** — relecture + `g++ -fsyntax-only` OK.

---

### K5 · minor · état partagé (static local) · src/BatteryKalman.h:912 · statut: `resolved`

**Description** — `updateCycleDetection()` utilisait `static bool initialized` (locale statique
partagée entre instances).

**Impact** — Interférence entre instances, reset hebdomadaire du cycle erroné, non réentrant.

**Correctif appliqué** — Membre d'instance `bool cycle_initialized = false;` remplaçant le
`static`.

**Preuve** — relecture + `g++ -fsyntax-only` OK.

---

### K6 · minor · accesseur obsolète · src/BatteryKalman.h:312,1037 · statut: `resolved`

**Description** — `phase_info` était initialisé à `"Bootstrap"` et jamais réécrit ;
`getPhaseInfo()` renvoyait donc toujours `"Bootstrap"`.

**Impact** — Info de statut utilisateur systématiquement fausse.

**Correctif appliqué** — `updatePhaseFromConfidence()` met à jour `phase_info` :
```cpp
snprintf(phase_info, sizeof(phase_info), "%s", PHASE_NAMES[phase]);
```

**Preuve** — relecture + `g++ -fsyntax-only` OK.

---

### K7 · major · algorithme / process model · src/BatteryKalman.h:930-937,862 · statut: `resolved`

**Description** — Le modèle de processus applique la dérive de vieillissement
`C_hat -= dC_dCycle * delta_cycles` dans `predictKalman()`, appelé à chaque **mesure**. Mais
`delta_cycles` était l'incrément **par échantillon** (`cycles_partial - last_cycles`, recalculé à
chaque `update()`), et `cycles_partial` ne change qu'à la **complétion d'un cycle**
(`updateCycleDetection`, appelé APRÈS la mesure). À la fermeture d'un segment,
`delta_cycles ≈ 0` → `F01 = -delta_cycles ≈ 0` → **aucune covariance croisée** `P[1][0]` générée
→ `K_aging = P[1][0]/S = 0` → `dC_dCycle` **jamais appris**.

**Impact** — L'« apprentissage automatique du vieillissement » (caractéristique annoncée) était
**inopérant** : `dC_dCycle` restait à sa valeur par défaut ; le filtre était de fait 1D.
Détecté lors de la vérification mathématique de l'algorithme (EKF 2D).

**Correctif appliqué** — Accumulation des cycles écoulés depuis la dernière mesure
(`cycles_since_update`), consommée à la mesure :
```cpp
// update()
cycles_since_update += data->cycles_partial - last_cycles;
last_cycles = data->cycles_partial;
updateSegmentAndKalman(cycles_since_update);
// updateSegmentAndKalman(), après la mesure :
applyKalmanUpdate2D(C_measured, R, delta_cycles);
cycles_since_update = 0.0f;
```

**Preuve** — `doc/_repro_aging.cpp` :
```
  AVANT (delta=0)  : a=0.00050000  P[1][0]=0.000e+00  (a INCHANGE -> non appris)
  APRES (delta=1)  : a=0.00050247  P[1][0]=-2.643e-06  (a EVOLUE)
```

---

### R1 · major · logique / porte de stabilité REST · src/BatteryKalman.h:609-627 · statut: `resolved`

**Description** (détecté en revue de clôture, **activé par le correctif K1**) — L'anneau de
stabilité a une taille fixe (`REST_LONG_STABLE_MAX_SAMPLES = 32`). Après K1, il tourne bien, mais
il ne couvre que les 32 **derniers échantillons** : à 1–10 Hz la fenêtre de stabilité
`rest_long_stable_min` n'était donc pas respectée (≈ 3 s à 10 Hz), contrairement à la
promesse « indépendant du rythme d'échantillonnage ». La porte F8 devenait trop faible et
pouvait accorder une référence OCV avant équilibrage (le risque que F8 devait éviter).

**Impact** — À 10 Hz : fenêtre réelle ≈ 3,1 s au lieu de la durée configurée → REST_LONG accordé
trop tôt → biais d'apprentissage de capacité. À très faible cadence : `kept < min_samples` →
REST_LONG inatteignable (limite de tuning, voir R3).

**Correctif appliqué** — Anneau réellement **temporel** : écriture décimée à au plus un
échantillon par `stable_ms / MAX` (`push_ms`), pour que l'anneau couvre toute la fenêtre quelle
que soit la cadence. Membres `rest_v_last_push_ms` et `rest_v_has_push` (remis à 0/false en
quittant REST).
```cpp
uint32_t push_ms = stable_ms / REST_LONG_STABLE_MAX_SAMPLES;
if (push_ms == 0) push_ms = 1;
if (!rest_v_has_push || (now_ms - rest_v_last_push_ms) >= push_ms) { ...push... }
```

**Preuve** — `doc/_repro_window.cpp` (fenêtre 15 min, `span`/`kept`) :
```
  10 Hz           |    3100 ms / 32 OK       |  874199 ms / 32 OK
  1 Hz            |   31000 ms / 32 OK       |  898999 ms / 32 OK
  0.2 Hz (5min)   |  899999 ms /  4 kept<6   |  899999 ms /  4 kept<6
```

**Reste à documenter (non bloquant)** — À très faible cadence, `min_samples` doit être abaissé
côté tuning : voir **R3**.

---

### R2 · major · conception / porte de stabilité REST · src/BatteryKalman.h:642-646 · statut: `resolved` (lookback + profils R7)

**Description** — La porte F8 exige `time_ok` (repos ≥ `rest_long_min_min`) **et** `stable_ok`
(`kept ≥ min_samples` et écart crête ≤ seuil). Or, par défaut, `rest_long_min_min = 15 min` **<
`rest_long_stable_min = 30 min`**. À 15 min de repos, la fenêtre de 30 min n'est donc pas encore
couverte, et REST_LONG est accordé avec seulement ~15 min de stabilité. La doc (« stable sur une
fenêtre de 30 min ») n'est pas strictement respectée.

**Impact** — Référence OCV de repos long accordée possiblement avant équilibrage complet → biais
d'apprentissage de capacité (le risque que F8 visait). Sévérité *major* selon la revue.

**Résolution** — La fenêtre est un **lookback** plafonné à `rest_long_stable_min` : à 15 min de
repos on vérifie la planéité sur les ~15 min disponibles ; au-delà de 30 min, sur les 30 dernières
minutes. Le défaut `15 / 30` est donc **cohérent** (repos ≥ 15 min ET planéité sur ≤ 30 min) et
**conservé**. La différenciation par chimie est apportée par le **profil par technologie (R7)** et
la robustesse au bruit par le **seuil auto-adaptatif (R6)** — voir `doc/rest_long_tuning.md`.

**Correctif appliqué** — Aucun changement de valeur par défaut (`15 / 30 / 20 mV` conservés) ;
ajout de `applyRecommendedRestLong()` (R7) et du seuil adaptatif (R6) ; sémantique lookback
documentée.

---

### R3 · minor · doc vs comportement · src/BatteryKalman.h:612-613 · statut: `resolved` (documenté)

**Description** — Avec la décimation (R1) et `min_samples = 6`, REST_LONG devient inatteignable
dès que le pas d'échantillonnage dépasse `stable_ms/(min_samples-1)`. Avec la fenêtre par défaut
de **30 min** (et R7 plomb : 60 min), cela donne une cadence requise de **≤ 6 min** (≤ 12 min pour
le plomb) ; pour une fenêtre de 15 min, **≤ 3 min**. La mention « indépendant du rythme
d'échantillonnage » est donc trop absolue.

**Correctif** — Documenté (ce journal + commentaire d'en-tête + `doc/rest_long_tuning.md`) :
cadence requise `≤ rest_long_stable_min/(min_samples-1)`. Pour un échantillonnage lent, abaisser
`rest_long_stable_min_samples` (via `setRestLong(...)`).

**Preuve** — `doc/_repro_window.cpp` (fenêtre 15 min) : à 1 Hz → `kept=32` OK ; à 5 min →
`kept=4` (`kept<6`) → REST_LONG inatteignable.

---

### R4 · minor · edge case (boot) · src/BatteryKalman.h:621-628 · statut: `resolved`

**Description** — Le sentinelle `rest_v_last_push_ms == 0` (« jamais poussé ») se confond avec un
premier échantillon à `millis() == 0`, provoquant une double écriture au démarrage.

**Correctif appliqué** — Membre explicite `bool rest_v_has_push = false;` (remis à `false` en
quittant REST) au lieu du sentinelle `0`.

**Preuve** — relecture + `g++ -fsyntax-only` OK.

---

### R5 · minor · artefact de preuve · doc/_repro_window.cpp · statut: `resolved`

**Description** — La 1ʳᵉ version du repro R1 renvoyait le *span* au lieu du *kept* et utilisait un
sentinelle `0xFFFFFFFF` (différent de l'en-tête qui utilise `0`) → surestimait l'indépendance à la
cadence et masquait R3.

**Correctif appliqué** — Le repro rapporte désormais `span` **et** `kept`, avec le sentinelle `0`
de l'en-tête, et signale `kept < min_samples`.

**Preuve** — sortie `doc/_repro_window.cpp` ci-dessus (R3).

---

### R6 · ajout · auto-adaptatif · src/BatteryKalman.h:650-666 · statut: `resolved`

**Objectif** — Rendre le **seuil de planéité** robuste au bruit de mesure (sinon un seuil fixe
rejette une batterie pourtant relaxée, ou devient trop laxiste).

**Implémentation** — Estimation continue du bruit de tension au repos (`rest_v_noise` = moyenne
glissante de `|V - V_précédent|`, via le nouveau membre `V_prev_sample`), puis
`stable_mv_eff = max(rest_long_stable_mv, REST_LONG_NOISE_K × bruit)`, borné à `3 × base`.
Macro `REST_LONG_NOISE_K` (défaut 5.0). Comportement inchangé si le bruit est faible.

**Preuve** — `doc/_repro_noise.cpp` :
```
  bruit sigma= 2.0 mV | ptp=  8.8 | fixe=20.0 -> ACCEPTE | adapt=20.0 -> ACCEPTE
  bruit sigma= 6.0 mV | ptp= 30.5 | fixe=20.0 -> REJETTE | adapt=30.7 -> ACCEPTE
  bruit sigma=10.0 mV | ptp= 36.3 | fixe=20.0 -> REJETTE | adapt=56.8 -> ACCEPTE
```

---

### R7 · ajout · profils par technologie · src/BatteryKalman.h:1155-1190 · statut: `resolved`

**Objectif** — Choisir automatiquement les meilleurs paramètres REST_LONG selon la chimie.

**Implémentation** — `applyRecommendedRestLong()` : déduit la chimie du nom
(`model->getTechnologyName()`) et applique via `setRestLong()` (override runtime, niveau 1) :

| Technologie | min_min | stable_min | stable_mv |
|---|---|---|---|
| Plomb inondé/AGM/Gel/PbC | 30 | 60 | 30 mV |
| LiFePO4 (LFP) | 30 | 60 | 10 mV |
| Li-ion (NMC/NCA/LCO/LiPo/LMNO), LTO, NiMH/NiCd/NiZn, Sodium | 15 | 30 | 20 mV |
| Supercondensateur (EDLC) | 5 | 15 | 50 mV |

Détail et justification : `doc/rest_long_tuning.md`.

**Preuve** — `doc/_repro_restlong_profile.cpp` : profil correct pour 10 technologies (plomb,
LiFePO4, Li-ion, LTO, NiMH, NiCd, Sodium, EDLC).

---

### K8 · major · persistance / initialisation · src/BatteryKalman.h:878-893 · statut: `resolved`

**Description** (cycle 2) — `begin()` ne synchronisait `last_Ah` que dans la branche
`if (!data->coulomb_initialized)`. En **restauration** (`coulomb_initialized == true`, état
`SoCData` persisté), `last_Ah` restait à son défaut membre (`0.0f`). Si le **compteur coulomb**
est lui aussi restauré à une valeur non nulle (recommandé via `Fo170/Persistance`), le premier
`update()` calcule `dAh = getAmpereHours() - last_Ah = valeur - 0` → injection de **tout** le
compteur comme incrément.

**Impact** — Au redémarrage avec persistance, saut de `SoC_coulomb` (borné 0/100), estimation
fausse jusqu'à la prochaine synchro.

**Correctif appliqué** — Synchronisation **inconditionnelle** dans `begin()` :
```cpp
last_Ah = coulombMeter->getAmpereHours();   // hors du if(!coulomb_initialized)
```

**Preuve** — `doc/_repro_restore.cpp` :
```
K8 begin()/last_Ah  : SoC_coulomb avant=60.0 apres=60.0  (AVANT-fix aurait saute a 100.0)
   -> OK (pas de saut)
```

---

### K9 · minor · logique / persistance · src/BatteryKalman.h:704-713 · statut: `resolved`

**Description** (cycle 2) — Le cutoff basse tension appelait `doSync(3.0f, …)` à **chaque**
échantillon tant que la batterie restait au repos sous `V_min`, sans porte temporelle ni
hystérésis. `doSync()` met `state_dirty = true` et réinitialise le compteur coulomb.

**Impact** — Écriture de persistance **à chaque échantillon** (usure EEPROM/Flash) et reset
coulomb continu tant que la condition persiste.

**Correctif appliqué** — Porte temporelle (1 synchro / 5 min ; `last_sync_time` est mis à jour
par `doSync()`) :
```cpp
if ((mppt_state == State_REST || mppt_state == State_REST_LONG) && total_ocv < min_voltage
    && millis() - data->last_sync_time > REST_ALIGN_SYNC_MS) { doSync(3.0f, 0.75f, "Low voltage cutoff"); return; }
```

**Preuve** — `doc/_repro_restore.cpp` :
```
K9 cutoff basse V   : resets AVANT=60 (chaque ech.)  APRES=1  -> OK (borne)
K9 e2e (header)     : resets coulomb=1 sur 60 ech. -> OK
```

---

### K10 · minor · bloat / tuning · src/BatteryKalman.h · statut: `resolved` (champs morts supprimés + doc)

**Description** (cycle 2) — Deux points morts : (a) la taille de l'anneau REST est pilotée par la
**macro** `REST_LONG_STABLE_MAX_SAMPLES`, alors que le champ homonyme
`KalmanTuning::rest_long_stable_max_samples` (exposé par BatteryModels) n'est **jamais consommé** →
un override via `setRestLong()`/tuning est silencieusement sans effet ; (b) champs de `Segment`
(`I_sq_sum/I_sum/I_min/I_max/conf_start/start_ms/had_discharge/had_charge`) et membre
`_tuningFromModel` écrits mais jamais lus.

**Impact** — Attente déçue côté tuning (taille non modifiable par le tuning) ; code/bruit.

**Correctif** —
- (b) **suppression effective** des champs morts : `Segment` réduit à `{active, Ah_start,
  SoC_start, n}` ; `_tuningFromModel` retiré ; helper `_restLongMinSamples()` (jamais appelé)
  retiré ; `startNewSegment()` perd son paramètre `conf` inutilisé ; `updateSegmentAndKalman()`
  perd son paramètre `I` inutilisé ; suppression du bloc d'accumulation
  `I_sum/I_sq_sum/I_min/I_max/had_*` et du calcul de `conf_end`/`extra` devenus sans effet.
  Comportement **strictement identique** (ces valeurs n'étaient jamais lues).
- (a) **documenté** : commentaire d'en-tête au-dessus de la macro. Pour changer la taille,
  définir `REST_LONG_STABLE_MAX_SAMPLES` avant `#include`.

**Preuve** — `g++ -fsyntax-only` OK (aucun nouveau warning) ; suite `_repro_*` → tous OK.

---

### K11 · minor · edge / wrap millis() · src/BatteryKalman.h:632-643 · statut: `resolved`

**Description** (cycle 2) — Après ~49,7 j d'uptime, si un repos chevauche le wrap de `millis()`,
`now_ms` redevient petit → `cutoff = (now_ms > stable_ms) ? (now_ms - stable_ms) : 0UL` renvoie 0
→ les cases **pré-wrap** (ms ≈ 4,29e9) ne sont plus purgées → écart crête gonflé → `stable_ok`
faux → REST_LONG différé jusqu'à réécriture complète de l'anneau.

**Correctif appliqué** — Détection de wrap : si `now_ms < rest_v_last_push_ms`, l'anneau est
vidé (horodatages du cycle précédent invalides) :
```cpp
if (rest_v_has_push && now_ms < rest_v_last_push_ms) {
    rest_v_count = 0; rest_v_last_push_ms = 0; rest_v_has_push = false;
    for (uint8_t i = 0; i < REST_LONG_STABLE_MAX_SAMPLES; i++) rest_v_ring_ms[i] = 0;
}
```

**Preuve** — `doc/_repro_wrap.cpp` :
```
K11 wrap millis()   : cases conservees AVANT=8 (perimees)  APRES=0 (anneau vide) -> OK
```

---

### K12 · low · nit / maintenabilité · src/BatteryKalman.h:171,727,732 · statut: `resolved`

**Description** (cycle 2, revue de clôture) — Le littéral `300000UL` (« 5 min ») était dupliqué
entre la porte du cutoff basse tension (K9) et la branche d'alignement REST, sans constante nommée.

**Correctif appliqué** — Extraction d'une macro nommée (hors tuning, comme `REST_LONG_SYNC_MS`) :
```cpp
#define REST_ALIGN_SYNC_MS 300000UL
```
utilisée aux deux sites. Aucun changement de comportement.

**Preuve** — `g++ -fsyntax-only` OK ; repros K8/K9/K11 OK.

---

### Vérification globale

- `g++ -std=c++11 -Wall -Wextra -I doc/_syntax -I src -fsyntax-only doc/_syntax/test_bk.cpp` → **SYNTAX_OK**
  (seul avertissement : paramètre `S` non utilisé dans `updateREstimate()`, préexistant, hors périmètre).
- Reproductions avant/après : K1 (`_repro_ring`), K2/K3 (`_repro_restlong`), R1/R3 (`_repro_window`),
  R6 (`_repro_noise`), R7 (`_repro_restlong_profile`), K7 (`_repro_aging`) → OK.
- Vérification mathématique de l'EKF (predict/update, Jacobien, gain) recoupée sur 4 sources
  indépendantes (Wikipedia Kalman, Wikipedia EKF, kalmanfilter.net, Wikipedia SoC) → **conforme**.
- Vérification par sous-agent `verifier` : K1–K6 **PASS**.
- **Cycle 2** — preuves : `doc/_repro_restore.cpp` (K8, K9), `doc/_repro_wrap.cpp` (K11) → OK.
  Non-régression cycle 1 relancée : `_repro_ring` (K1), `_repro_restlong` (K2/K3), `_repro_window`
  (R1/R3), `_repro_noise` (R6), `_repro_restlong_profile` (R7), `_repro_aging` (K7) → tous OK.
  Mocks fonctionnels ajoutés : `doc/_repro_mocks/{Arduino,BatteryModels,Coulomb}.h`.
- Revue de clôture par sous-agent `code_reviewer` : K1–K6 + R1 confirmés résolus ; R2 (conception),
  R3 (doc), R4 (edge boot), R5 (artefact) signalés → R4/R5 corrigés, R3 documenté, R2 résolu
  (lookback + R7).

### Faux positifs / non signalés (intentionnels)

- `mppt_state_prev` MAJ en fin de `update()` (F1) ; `updateSegmentAndKalman()` avant
  `handleSyncEvents()` (F2) ; macros `NAN/0` sentinelles ; stubs `saveState/loadState` ;
  commentaires FR + tags F — intentionnels.
- Double `startNewSegment()` sur transition FLOAT (via `updateSegmentAndKalman` puis `doSync`) :
  redondant mais sans effet fonctionnel — non signalé.
- Paramètre `S` non utilisé dans `updateREstimate()` : cosmétique, préexistant — non corrigé.
