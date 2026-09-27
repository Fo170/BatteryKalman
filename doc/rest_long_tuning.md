# Réglage REST_LONG par technologie + seuil auto-adaptatif

> BatterieKalman v1.7.0 — compléments R6/R7. Voir `../CHANGELOG.md` (Annexe K/R).

La référence OCV « repos long » (état `State_REST_LONG`) sert de point d'ancrage fiable
pour l'apprentissage de capacité. Sa qualité dépend de **trois** réglages :

| Réglage (`KalmanTuning`) | Rôle |
|---|---|
| `rest_long_min_min` | **Durée minimale de repos** avant d'autoriser un OCV (garde d'équilibrage). |
| `rest_long_stable_min` | **Fenêtre de planéité** : durée max sur laquelle on vérifie que la tension est plate. |
| `rest_long_stable_mv` | **Écart crête toléré** sur la fenêtre (le vrai garde-fou contre une relaxation en cours). |

Rappel : la fenêtre effective est `min(rest_long_min_min, rest_long_stable_min)`. Régler
`rest_long_min_min = rest_long_stable_min` rend la sémantique explicite ; les laisser à
`15 / 30` signifie « repos ≥ 15 min ET planéité sur les ≤ 30 dernières minutes » (lookback).

## 1. Recommandations par technologie (R7)

Appliquées automatiquement par `battery.applyRecommendedRestLong()` (déduit du nom de techno
renvoyé par `model->getTechnologyName()`), ou à reporter dans `TECH_PARAMS[].kalman` de
BatteryModels :

| Technologie | `min_min` | `stable_min` | `stable_mv` | Raison |
|---|---|---|---|---|
| Plomb inondé / AGM / Gel / PbC | **30** | **60** | **30 mV** | Relaxation lente (1–4 h) → repos long + tolérance plus large. |
| LiFePO4 (LFP) | **30** | **60** | **10 mV** | Plateau OCV très plat → repos long, seuil serré. |
| Li-ion (NMC/NCA/LCO/LiPo/LMNO) | 15 | 30 | 20 mV | Relaxation rapide. |
| LTO (Li-titanate) | 15 | 30 | 20 mV | Relaxation rapide. |
| NiMH / NiCd / NiZn / NiFe | 15 | 30 | 20 mV | Relaxation rapide. |
| Sodium-ion | 15 | 30 | 20 mV | OCV peu fiable (segments OCV désactivés côté tuning). |
| Supercondensateur (EDLC) | **5** | **15** | **50 mV** | Tension quasi linéaire, relaxation quasi nulle. |

> ⚠️ **Cadence minimale.** Avec `min_samples = 6`, il faut `dt ≤ stable_min / 5`
> (ex. fenêtre 30 min → `dt ≤ 6 min` ; fenêtre 15 min → `dt ≤ 3 min`). Pour un échantillonnage
> plus lent, abaisser `rest_long_stable_min_samples` via `setRestLong(...)`.

## 2. Seuil de planéité auto-adaptatif au bruit (R6)

Problème : un seuil **fixe** (`rest_long_stable_mv`) est soit trop strict (le bruit ADC gonfle
l'écart crête → REST_LONG jamais accordé), soit trop laxiste. BatteryKalman estime en continu le
**bruit de tension** au repos (moyenne glissante de `|V - V_précédent|`) et relâche le seuil :

```
stable_mv_eff = max(rest_long_stable_mv, REST_LONG_NOISE_K × bruit)   borné à 3 × rest_long_stable_mv
```

- `REST_LONG_NOISE_K` (macro compile-time, défaut `5.0`) : calibré pour que l'écart crête du seul
  bruit passe.
- Comportement inchangé si le bruit est faible (`stable_mv_eff = rest_long_stable_mv`).
- Preuve : `doc/_repro_noise.cpp` — à σ = 6–10 mV, le seuil fixe **rejette** une batterie pourtant
  relaxée ; le seuil adaptatif l'**accepte**.

## 3. Comment appliquer

```cpp
#include <BatteryModels.h>
#include <BatteryKalman.h>

BatteryModel model(TECH_FLOODED, 6, 0.0f);   // ex. plomb inondé
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);
battery.begin();

// Option A (recommandé) : profil automatique selon la techno
battery.applyRecommendedRestLong();

// Option B : réglage explicite (override runtime, prime sur le modèle)
battery.setRestLong(/*min_min*/ 30.0f, /*stable_mv*/ 0.030f, /*stable_min*/ 60.0f, /*min_samples*/ 6);
```

Hiérarchie de résolution : **setter runtime (A/B) > `model->getKalmanTuning()` > macros > défaut**.
Donc `applyRecommendedRestLong()`/`setRestLong()` **priment** sur les valeurs de BatteryModels ;
si tu préfères centraliser, reporte les valeurs du tableau §1 dans `TECH_PARAMS[].kalman` de
BatteryModels et n'appelle rien.

## 4. Récapitulatif des paramètres REST_LONG par défaut (BatteryKalman)

| Paramètre | Valeur par défaut | Emplacement |
|---|---|---|
| `rest_long_min_min` | 15 min | `KALMAN_DEFAULT_TUNING` |
| `rest_long_stable_min` | 30 min | `KALMAN_DEFAULT_TUNING` |
| `rest_long_stable_mv` | 0.020 V | `KALMAN_DEFAULT_TUNING` |
| `rest_long_stable_min_samples` | 6 | `KALMAN_DEFAULT_TUNING` |
| `rest_long_stable_max_samples` | 32 (compile-time) | macro `REST_LONG_STABLE_MAX_SAMPLES` |
| `REST_LONG_NOISE_K` | 5.0 (compile-time) | macro `REST_LONG_NOISE_K` |
