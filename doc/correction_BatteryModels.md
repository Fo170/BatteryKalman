# Correction `BatteryModels.h` — `detectChargeState()`

**Librairie** : [Fo170/BatteryModels](https://github.com/Fo170/BatteryModels)
**Version cible** : **1.3** (intègre ce correctif)
**Fichier corrigé** : `src/BatteryModels.h`
**Méthode corrigée** : `BatteryModel::detectChargeState()`
**Statut** : à intégrer dans la branche principale de la librairie GitHub

---

## 1. Contexte : pourquoi cette correction est nécessaire

`BatteryKalman` utilise les états de charge détectés par `detectChargeState()`
comme **points de référence** pour apprendre la capacité de la batterie :

| État | Rôle dans l'apprentissage |
|---|---|
| `State_FLOAT` | Point de référence « batterie pleine » → **SoC = 100 %** (confiance 95 %) |
| `State_REST_LONG` | Point de référence « OCV stabilisée » (repos long) |
| `State_REST` | Alignement doux du coulomb-counting (90/10) |

Si `State_FLOAT` n'est **jamais** détecté, le filtre Kalman ne reçoit
pratiquement aucun point de référence fiable et reste bloqué en phase
**Bootstrap** (capacité inconnue) pendant des mois.

C'est exactement ce qui a été observé sur les données réelles (jeu de
données depuis retiré du dépôt) : le filtre est resté 5 mois en Bootstrap,
incapable de fermer un seul segment de charge, faute de détection FLOAT.

---

## 2. Le bug

L'ordre des tests dans `detectChargeState()` était :

```cpp
if (discharging) return State_DISCHARGE;
if (in_rest) return State_REST;                                    // ← testé trop tôt
if (near_float && fabsf(current) < I_float_max) return State_FLOAT;
if (near_abs && charging) return State_ABSORPTION;
if (charging) return State_BULK;
return State_UNKNOWN;
```

avec :

```cpp
bool in_rest = (fabsf(current) < 0.05f);
bool near_float = (voltage >= V_float_ref - V_tol)
               && (voltage <= V_float_ref + V_tol * 0.5f);
```

### Pourquoi ça casse FLOAT

En **float**, la batterie est maintenue à sa tension de pleine charge
(≈ 14,2 V pour un 12 V) mais le courant de charge chute vers des valeurs
très faibles, souvent **sous 0,05 A** (|I| < 0,05 A).

Le test `if (in_rest) return State_REST;` est alors **vrai avant** même
que le test `near_float` soit évalué. La batterie en float est donc
systématiquement classée **`State_REST`**, jamais **`State_FLOAT`**.

Conséquence directe :
- `State_FLOAT` n'est jamais produit → pas de référence « SoC = 100 % » ;
- le point de référence le plus fiable du filtre (95 % de confiance)
  devient **inatteignable** ;
- `BatteryKalman` reste en Bootstrap (cf. bug F1/F5 dans `src/BatteryKalman.h`).

---

## 3. La correction

Il suffit de tester **`near_float` avant `in_rest`** :

```cpp
if (discharging) return State_DISCHARGE;
if (near_float && fabsf(current) < I_float_max) return State_FLOAT;
if (in_rest) return State_REST;
if (near_abs && charging) return State_ABSORPTION;
if (charging) return State_BULK;
return State_UNKNOWN;
```

Une tension dans la fenêtre FLOAT avec un courant faible (mais en dehors
de la décharge, `I > -0,05 A`) est désormais correctement classée
**`State_FLOAT`**, même quand `|I| < 0,05 A`.

### Pourquoi c'est sûr

- La **décharge** reste prioritaire : `I < -0,05 A` → `State_DISCHARGE`,
  quelle que soit la tension.
- Le test `near_float` exige toujours `fabsf(current) < I_float_max`
  (avec `I_float_max = max(0,6 A ; capacité × 2 %)`), donc un courant
  anormalement élevé à tension de float ne produit pas un faux FLOAT.
- `near_float` a une **hystérésis** symétrique par construction :
  la fenêtre est `[V_float_ref − V_tol ; V_float_ref + 0,5·V_tol]`.

---

## 4. Diff exact

```diff
         if (discharging) return State_DISCHARGE;
-        if (in_rest) return State_REST;
         if (near_float && fabsf(current) < I_float_max) return State_FLOAT;
+        if (in_rest) return State_REST;
         if (near_abs && charging) return State_ABSORPTION;
         if (charging) return State_BULK;
```

---

## 5. Validation

### 5.1 Test unitaire ciblé

Avec `BatteryModel model(TECH_FLOODED, 6, 0.0f)` (plomb inondé, 12 V,
float = 14,22 V) :

| Tension | Courant | Avant (buggé) | Après (corrigé) |
|---|---|---|---|
| 14,25 V | +0,01 A | `State_REST` ❌ | `State_FLOAT` ✅ |
| 14,25 V | +0,30 A | `State_FLOAT` | `State_FLOAT` |
| 12,50 V | 0,00 A | `State_REST` | `State_REST` |
| 12,20 V | −2,0 A | `State_DISCHARGE` | `State_DISCHARGE` |

### 5.2 Impact sur l'analyse réelle

La correction fait partie de l'ensemble F1–F8. À elle seule elle restaure
la **détection FLOAT** ; combinée aux correctifs de `BatteryKalman.h`
(transition FLOAT, fermeture de segment, EKF 2D...), la capacité estimée
passait de **108,2 Ah (buggé)** à **43,5 Ah (corrigé)** sur un groupement
de batteries plomb usagées annoncé < 50 Ah — estimation physiquement cohérente.
(Le jeu de données réel et le port Python de validation ont été retirés du dépôt.)

Voir le catalogue complet des corrections : [`../CHANGELOG.md`](../CHANGELOG.md) (Annexe K/R).

---

## 6. Numérotation

Cette correction est identifiée **F7** dans le catalogue des bugs :
- **F1–F6** : `src/BatteryKalman.h` v1.5.3 (transition d'état, ordre
  segment/sync, R adaptatif, EKF 2D, fermeture REST_LONG, apprentissage
  toujours actif).
- **F7** : `BatteryModels.h` — `detectChargeState()` (ce document).
- **F8** : `src/BatteryKalman.h` v1.5.3 — porte de stabilité REST (OCV fiable).

---

## 7. Version proposée pour le CHANGELOG de BatteryModels

```
## [1.3] - 2026-08-03

### 🐛 Fixed
- detectChargeState() : State_FLOAT masqué par State_REST.
  L'ordre des tests plaçait in_rest avant near_float, donc une batterie
  maintenue en float avec un courant < 0,05 A était classée REST au lieu
  de FLOAT. Le test near_float passe désormais avant in_rest, restaurant
  le point de référence "batterie pleine" (SoC = 100 %) pour BatteryKalman.
```
