# analyse.md — BatteryKalman (bug-hunt)

> Contexte produit pour la revue de code. Fichier cible : **`src/BatteryKalman.h`** (v1.7.0,
> header-only). Journal des findings : **`CHANGELOG.md`** (Annexe K/R).

## 0. Cadrage

- **Périmètre** : `src/BatteryKalman.h` (toute la logique). Dépendance externe lue pour
  référence : `BatteryModel` (BatteryModels v1.6.0).
- **Hors périmètre** : docs, exemples, reproductions `doc/_repro_*.cpp`.
- **Définition d'un « bug »** : écart entre le comportement du code et son intention
  documentée (commentaires F/K/R, `CHANGELOG.md`),
  ou erreur de calcul/état conduisant à une estimation fausse, instable ou non déclenchée.

## 1. Architecture

```
BatteryKalman (EKF 2D)  ── utilise ──> BatteryModel (OCV, R, detectChargeState, getKalmanTuning)
        │                ── utilise ──> Coulomb      (intégration Ah : addMeasurement/getAmpereHours/getLastInterval)
        └── persiste (utilisateur) : SoCData + KalmanState2D
```

Un seul fichier : `src/BatteryKalman.h`. Include guard `BATTERY_KALMAN_H`. Dépend de
`BatteryModels.h` et `Coulomb.h`.

## 2. Types et structures clés

| Type | Rôle |
|---|---|
| `BatteryKalman` | Filtre. Constructeur `(SoCData*, KalmanState2D*, BatteryModel*, Coulomb*)`. |
| `SoCData` | État SoC persistable (SoC_coulomb/voltage/raw/fused, Ah totaux, cycles). |
| `KalmanState2D` | État EKF persistable : `C_hat`, `dC_dCycle`, `P[2][2]`, `R_estimated`, `R_measured`, `confidence`, `n_updates`, `outlier_streak`. |
| `KalmanTuning` | (BatteryModels) Réglages P/Q/R/seuils/REST_LONG/confiance par techno. |
| `LearningPhase` | `PHASE_BOOTSTRAP/COARSE/REFINE/TRACK` (dérivé de `confidence`). |
| `ChargeState` | (BatteryModels) `State_UNKNOWN/DISCHARGE/BULK/ABSORPTION/FLOAT/REST/REST_LONG`. |

## 3. Membres internes notables (état du filtre)

| Membre | Rôle | Piège |
|---|---|---|
| `mppt_state`, `mppt_state_prev` | État courant / précédent | `mppt_state_prev` MAJ **en fin de `update()`** (F1). |
| `state_entry_ms` | Instant d'entrée dans l'état courant | Sert à `time_ok` REST_LONG et à `extra` (confiance). |
| `rest_v_ring[]`, `rest_v_ring_ms[]`, `rest_v_count` | Anneau (tension, temps) pour la porte de stabilité REST (F8) | Index = `rest_v_count % MAX` ; **`rest_v_count` plafonné à MAX** → anneau figé (K1). |
| `seg` | Segment ΔAh/ΔSoC en cours | Fermé sur transition FLOAT/REST_LONG. |
| `cycle` | Suivi de cycle (SoC min/max, flags). | `static bool initialized` local (K5). |
| `phase_info` | Chaîne d'info de phase | Jamais mise à jour (K6). |
| `_tuningOverride/FromModel/Resolved`, `_tuningOverrideSet` | Hiérarchie de tuning | Résolu à chaque `getTuning()`. |

## 4. API / fonctions principales

- `begin()` : `loadState()` (stub), init coulomb si besoin, `updatePhaseFromConfidence()`.
- `update(V, I, T)` : point d'entrée par échantillon. Ordre interne :
  `updateDerivatives` → `correctVoltageToOCV` → `SoC_voltage` → `updateMpptState` →
  comptage coulomb → **`updateSegmentAndKalman` (AVANT sync, F2)** → `handleSyncEvents` →
  `fuseSoC` → `computeUncertainty` → `updateCycleDetection` → `mppt_state_prev = mppt_state` (F1).
- `updateMpptState()` : `detectChargeState()` + détection transition + porte de stabilité REST (F8).
- `handleSyncEvents()` : synchro FLOAT (SoC=100 %), REST_LONG (OCV), low-voltage cutoff, alignement REST.
- `updateSegmentAndKalman()` : collecte ΔAh/ΔSoC, calcule `C_measured`, appelle l'EKF.
- `predictKalman()`, `applyKalmanUpdate2D()`, `updateREstimate()` : cœur EKF 2D.
- `fuseSoC()` : fusion coulomb/tension selon l'état.
- Getters : `getSoC`, `getEffectiveCapacity`, `getConfidence`, `getKalmanC`, `getKalmanAgingRate`,
  `getR_measured`, `getChargeState`, `getLearningPhaseStr`, `getPhaseInfo` (K6).
- Tuning runtime : `getTuning`, `setP/Q/R/SegmentThresholds/BatteryChange/RestLong/Confidence`, `resetTuning`.

## 5. Invariants intentionnels (NE PAS signaler comme bugs)

- **F1** : `mppt_state_prev` MAJ en fin de `update()`.
- **F2** : `updateSegmentAndKalman()` appelé AVANT `handleSyncEvents()`.
- **F3** : `R_measured = R_estimated` après lissage.
- **F4** : EKF 2D complet (Jacobien `F=[[1,-Δ],[0,1]]`, gain 2D, covariance croisée).
- **F5** : fermeture de segment sur transition REST_LONG.
- **F6** : apprentissage Kalman toujours actif (indépendant de `isAutoDetect()`).
- **F7** : correctif `detectChargeState()` (côté BatteryModels v1.3).
- **F8** : porte de stabilité REST (anneau tension/temps) — **mais implémentation boguée, voir K1**.
- Macros `NAN/0` sentinelles testées par `MACRO == MACRO` ; défauts réels dans `KALMAN_DEFAULT_TUNING`.
- `saveState()`/`loadState()` stubs vides (persistance utilisateur).

## 6. Dépendances / outils de vérification

- Dépendances : `BatteryModels >= 1.4`, `CoulombsAh >= 1.1` (`Coulomb.h`).
- Pas de build/test/CI dans le dépôt.
- Vérification : compiler un exemple (`arduino-cli`/`pio`) **ou** reproductions C++ autonomes
  dans `doc/` (`_repro_*.cpp`, compilables avec `g++` ; `_repro_mocks/` pour K8/K9).

## 7. Findings (voir `CHANGELOG.md`, Annexe K/R, pour le détail)

| id | sévérité | résumé | fichier:ligne |
|---|---|---|---|
| K1 | major | Anneau de stabilité REST figé (index bloqué à 0 après remplissage) | src/BatteryKalman.h:607-610 |
| K2 | major | `state_entry_ms` réinitialisé à chaque promotion REST→REST_LONG → REST_LONG instable | src/BatteryKalman.h:597-598,631-632,902 |
| K3 | minor | Signe de `dC_dCycle` incohérent avec le modèle de processus | src/BatteryKalman.h:245,408,444,476,964 |
| K4 | minor | Underflow `millis()` (`cutoff`, `extra`) | src/BatteryKalman.h:615,653,776 |
| K5 | minor | `static bool initialized` partagé dans `updateCycleDetection()` | src/BatteryKalman.h:909 |
| K6 | minor | `phase_info` jamais mis à jour → `getPhaseInfo()` toujours « Bootstrap » | src/BatteryKalman.h:312,1032 |
| R1 | major | Anneau de stabilité REST non temporel (fenêtre ≈3 s à 10 Hz au lieu de 30 min) | src/BatteryKalman.h:609-627 |
| K8 | major | `begin()` ne synchronise `last_Ah` qu'à l'init → saut SoC en restauration | src/BatteryKalman.h:878-893 |
| K9 | minor | Cutoff basse tension : `doSync()` à chaque échantillon (persistance + reset coulomb continus) | src/BatteryKalman.h:704-713 |
| K10 | minor | `rest_long_stable_max_samples` (tuning) ignoré + champs `Segment` morts (supprimés) | src/BatteryKalman.h |
| K11 | minor | Wrap `millis()` dans `cutoff` de l'anneau REST → REST_LONG différé | src/BatteryKalman.h:632-643 |
| K12 | low | Littéral `300000UL` dupliqué (cutoff basse tension + alignement REST) | src/BatteryKalman.h:171,727,732 |

> **Cycle 2** : K8/K9/K11 corrigés, K10 (champs morts supprimés + doc), K12 (nit) corrigé. Preuves :
> `doc/_repro_restore.cpp`, `doc/_repro_wrap.cpp`. Non-régression cycle 1 OK.
> Voir `CHANGELOG.md` (Annexe K/R).
