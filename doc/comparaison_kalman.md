# Comparaison — Filtre de Kalman (général, batteries, BatteryKalman)

> Document d'analyse. Compare l'algorithme de Kalman **en général**, ses **variantes
> spécifiques aux batteries** (littérature), et le filtre réellement implémenté dans
> **BatteryKalman v1.7.0**, dans le contexte de la chaîne
> `CoulombsAh` + `BatteryModels` + `BatteryKalman` + `BatteryLifePredictor`.
> Toutes les affirmations comparatives sont sourcées en fin de document (§G).

---

## A. Kalman et EKF — la base théorique

### A.1 Filtre de Kalman (linéaire, discret)

Le filtre de Kalman (Kalman 1960) est l'estimateur **optimal** (au sens moindres carrés)
pour un système **linéaire** à bruits blancs gaussiens additifs. Modèle :

```
x_k = F·x_{k-1} + B·u_{k-1} + w_{k-1}      w ~ N(0, Q)
z_k = H·x_k + v_k                          v ~ N(0, R)
```

Deux phases : **prédiction** puis **correction** (mise à jour).

| Étape | Équation |
|---|---|
| Prédiction d'état | `x⁻ = F·x + B·u` |
| Prédiction de covariance | `P⁻ = F·P·Fᵀ + Q` |
| Innovation (résidu) | `y = z − H·x⁻` |
| Covariance d'innovation | `S = H·P⁻·Hᵀ + R` |
| Gain de Kalman | `K = P⁻·Hᵀ·S⁻¹` |
| Mise à jour d'état | `x = x⁻ + K·y` |
| Mise à jour de covariance | `P = (I − K·H)·P⁻` |

### A.2 Extended Kalman Filter (EKF)

Quand `f` et `h` sont **non linéaires**, l'EKF linéarise autour de l'estimation courante
(Taylor au 1er ordre) et remplace les matrices par les **jacobiens** :

```
x_k = f(x_{k-1}, u_{k-1}) + w
z_k = h(x_k) + v
F_k = ∂f/∂x |_{x̂, u}      H_k = ∂h/∂x |_{x̂}
```

Les équations predict/update sont **identiques** à celles du KF linéaire, avec `F_k` et `H_k`
évalués à chaque pas.

**Limites reconnues de l'EKF** :

- **Non optimal** en général (optimal seulement si le système est linéaire).
- **Divergence** possible si l'initialisation ou le modèle sont mauvais.
- La covariance `P` **sous-estime** typiquement la vraie covariance → filtre *inconsistent*
  sans ajout de « bruit stabilisant ».
- Ne représente pas les densités multimodales (capteur cubique, etc.).

**Alternatives** : iterated EKF (re-linéarisation), robust EKF (H∞ / Riccati « faux »),
**UKF** (transformée non parfumée, plus robuste/précis), Ensemble KF, particle filters.

### A.3 Point clé pour BatteryKalman

Si `f` et `h` sont **linéaires en l'état**, l'« EKF » se réduit à un **KF linéaire** (F/H
constants en structure, éventuellement variables dans le temps). La non-linéarité (courbe
OCV↔SoC) doit alors être traitée **en dehors** du filtre. C'est exactement le cas du projet
(§C) : le choix EKF vs UKF n'a donc aucun effet sur le résultat.

---

## B. Variantes spécifiques aux batteries (littérature)

| Variante | Idée | Ce qu'elle estime | Détail |
|---|---|---|---|
| **EKF + modèle ECM** (1RC / 2RC) | État `x=[SoC, V1(, V2)]`, entrée = courant, mesure = tension terminale ; `h(x)=V_OCV(SoC) − R0·i − V1(…)`, `H=[∂V_OCV/∂SoC, −1(, −1)]` | **SoC** (approche dominante) | §B.2.1 |
| **AEKF** (Adaptive EKF) | Met à jour `Q` (et parfois `R`) **en ligne** à partir de l'analyse des innovations | SoC robuste au bruit/modèle | §B.2.2 |
| **DEKF** (Dual EKF) | **Deux filtres** couplés : un pour l'**état** (SoC), un pour les **paramètres** (capacité, R) | **SoC + SOH** simultanés | §B.2.3 |
| **DUKF / multi-échelle** | UKF dual ; le filtre paramètre tourne **plus lentement** (le SOH varie très lentement vs le SoC) | SoC + capacité (SOH) | §B.2.4 |
| **UKF** | Sigma-points (pas de Jacobien), propage `2n+1` points via `f`/`h` | SoC (non-linéarités fortes) | §B.2.5 |
| **Kalman 1D (régression)** | Lissage de la pente de dégradation `capacité ≈ a·t + b` | **RUL / EOL** (durée de vie) | §B.2.6 |

**Deux courants** :

1. Le **SoC** est presque toujours estimé par un filtre **sur la tension** (EKF sur ECM).
2. La **capacité/SOH** est estimée par un filtre **paramétrique** (dual/joint) **ou** par une
   **régression** sur la dérive observée — pas comme composante d'état d'un EKF-SoC classique.

### B.2 Détail des algorithmes

#### B.2.1 EKF + modèle électrique équivalent (ECM 1RC / 2RC)

Modèle de circuit (Thévenin) : source OCV `V_OCV(SoC)`, résistance série `R0`, et une (ou
deux) branche(s) RC `(R1,C1)` `(R2,C2)` pour la polarisation.

**1RC — état `x = [SoC, V1]ᵀ`, entrée `u = i` :**

```
f(x,u) = [ SoC − (η·Δt / C_n)·i ,
           e^(−Δt/(R1·C1))·V1 + R1·(1 − e^(−Δt/(R1·C1)))·i ]ᵀ
h(x,u) = V_OCV(SoC) − R0·i − V1
```

Jacobiens :

```
F = [[ 1 , 0 ],
     [ 0 , e^(−Δt/(R1·C1)) ]]                    (∂f/∂SoC pour V1 = 0)

H = [ ∂V_OCV/∂SoC , −1 ]
```

**2RC** : on ajoute l'état `V2` avec sa propre équation RC ; `H = [∂V_OCV/∂SoC, −1, −1]`.

Boucle complète (identique au KF) :

```
Prédiction : x⁻ = f(x,u)            P⁻ = F·P·Fᵀ + Q
Innovation : y = V_mesuré − h(x⁻,u) S  = H·P⁻·Hᵀ + R
Gain       : K = P⁻·Hᵀ/S
Update     : x ← x⁻ + K·y           P ← (I − K·H)·P⁻
```

`∂V_OCV/∂SoC` provient de la pente locale de la table OCV (petit sur plateau → convergence
lente, cas LFP). C'est **l'algorithme de référence pour le SoC**.

#### B.2.2 AEKF (Adaptive EKF) — `Q`/`R` en ligne

Mêmes équations que B.2.1, mais `Q` (et/ou `R`) ne sont plus constants : ils sont estimés
depuis la séquence d'innovations (fenêtre glissante de `N` pas).

```
Innovation moyenne    : ȳ = (1/N)·Σ y_i
Cov. innovation (est.) : C_v = (1/N)·Σ y_i·y_iᵀ
Estimation de R        : R̂ = C_v − H·P⁻·Hᵀ        (borné [R_min, R_max])
Estimation de Q        : Q̂ = K·C_v·Kᵀ             (approximation courante)
Mise à jour            : Q ← Q̂ (lissage EMA),  R ← R̂
```

Variantes : AEKF « à la divergence » (n'adapte `Q` que si l'innovation dépasse un seuil),
filtre de Sage-Husa (estimation conjointe état + bruits). **But** : robustesse au modèle et
au bruit réel. BatteryKalman en reprend **seulement la partie `R`** (§C.3).

#### B.2.3 DEKF (Dual EKF) — SoC + SOH

**Deux filtres couplés** partageant la même mesure (tension) :

- **Filtre d'état** : `x = [SoC, V1, …]`, paramètres figés à `θ̂`.
- **Filtre de paramètre** : `θ = [C_n, R0, R1, …]`, état figé à `x̂`.

```
Filtre état      : prédit x avec f(x,θ̂,u), corrige avec H_x = ∂h/∂x
Filtre paramètre : prédit θ (souvent θ⁻ = θ, marche aléatoire), corrige avec H_θ = ∂h/∂θ
```

Échange à chaque pas : le filtre état utilise `θ̂` pour calculer `h` et `H_x` ; le filtre
paramètre utilise `x̂` (souvent l'état prédit) pour calculer `H_θ`. Le filtre paramètre a
**beaucoup plus petit `Q`** (les paramètres évoluent lentement). C'est **l'architecture
dominante pour le SoC + SOH simultanés**.

#### B.2.4 DUKF / multi-échelle (deux échelles de temps)

Identique au DEKF mais avec un **UKF** dans chaque filtre, et surtout des **rythmes
différents** : le filtre **paramètre (SOH)** tourne à une période **macro** `T_macro`
(ex. toutes les minutes/heures), le filtre **état (SoC)** à la période **micro** `Δt`.
Justification : le SOH varie de plusieurs ordres de grandeur plus lentement que le SoC, donc
la mesure de capacité accumulée sur une fenêtre macro est bien plus informative.

```
micro (chaque Δt)   : update SoC      (UKF état)
macro (chaque T_macro) : update θ = [C_n, R]  (UKF paramètre)
```

#### B.2.5 UKF (Unscented KF) — alternative non linéaire

Au lieu de linéariser (EKF), on propage `2n+1` **sigma-points** à travers `f`/`h` :

```
Points : χ₀ = x̂ ,  χ_i = x̂ ± √((n+λ)·P)     (colonnes de la racine de P)
Poids  : W₀^m, W_i^c  (λ = α²(n+κ) − n)
Predict : χ⁻ = f(χ)  →  x⁻ = Σ W_i^m χ⁻_i ,  P⁻ = Σ W_i^c (χ⁻_i − x⁻)(χ⁻_i − x⁻)ᵀ + Q
Update  : Z = h(χ⁻)  →  ẑ = Σ W_i^m Z_i ,  S = Σ W_i^c (Z_i−ẑ)(…)ᵀ + R
          P_xz = Σ W_i^c (χ⁻_i−x⁻)(Z_i−ẑ)ᵀ ,  K = P_xz·S⁻¹
```

Plus robuste/précis que l'EKF sur les non-linéarités fortes, **sans Jacobien**. Ne change
rien pour BatteryKalman (filtre linéaire, §A.3).

#### B.2.6 Kalman 1D de dégradation (RUL / EOL)

Modèle de tendance linéaire : `capacité(%) = a·t + b`. État `x = [a, b]ᵀ` (pente, offset).

```
F = [[1, 0],
     [0, 1]]          (a et b constants → marche aléatoire par Q)
H = [ t , 1 ]         (mesure = capacité observée à l'instant t)
Q = diag(q_a, q_b)    R = r_mesure
```

Après convergence : `σ(a) = √P₀₀` donne l'incertitude de la pente ; `t_EOL = (80 − b)/a`.
C'est **exactement** ce que fait le **Kalman 1D** de `BatteryLifePredictor` (§E) — et **non**
BatteryKalman.

---

## C. Ce que fait réellement BatteryKalman

### C.1 Nature du filtre

BatteryKalman **n'est pas un EKF-SoC classique** : c'est un **estimateur récursif de
paramètres**.

- **État** : `x = [C, a]` avec `C` = capacité (Ah) et `a = dC/dcycle` (Ah/cycle, >0 = perte).
  **Le SoC n'est pas un état du filtre.**
- **Modèle de processus** : `C ← C − a·n`, `a ← a` (n = cycles écoulés depuis la dernière
  mesure) → `F = [[1, −n], [0, 1]]`.
- **Mesure** : `z = |ΔAh| / (|ΔSoC|/100)` = **capacité observée** sur un *segment* borné par
  deux références OCV (FLOAT = 100 %, ou REST_LONG). `h(x) = C`, `H = [1, 0]`.
- **Équations** (conformes au KF/EKF canonique) :
  ```
  Prédiction : x⁻ = f(x)          P⁻ = F·P·Fᵀ + Q,  Q = diag(q_a·n + q_C, q_a)
  Innovation : y = z − C          S  = P⁻₀₀ + R
  Gain       : K = [P⁻₀₀/S , P⁻₁₀/S]ᵀ
  Update     : x ← x⁻ + K·y       P ← (I − K·H)·P⁻
  ```

### C.2 Le SoC est hors filtre (filtre complémentaire)

Le SoC fusionné n'est **pas** produit par le Kalman mais par un **filtre complémentaire** :

```
SoC_fused = α·SoC_coulomb + (1 − α)·SoC_tension
```

où `α` dépend de l'état de charge détecté par `BatteryModels` (FLOAT/REST_LONG → tension ;
BULK/DISCHARGE → coulomb). C'est un équivalent fonctionnel de la « correction OCV » des BMS,
mais **non pondéré par les incertitudes** (pas de gain de Kalman).

### C.3 Adaptations « AEKF-lite » et robustesse

- **R adaptatif en ligne** : `R ← α·(var(innovation) − P₀₀) + (1−α)·R`, borné `[r_min, r_max]`
  (estimé depuis les innovations) → proche d'un **AEKF** sur `R`.
- **Rejet d'outliers 3-σ** + **détection de remplacement batterie** (n outliers + magnitude).
- **Confiance continue** `conf = ½(min(1, n/10) + (1 − min(1, P₀₀/P_init))) ∈ [0,1]`.

### C.4 Variantes d'implémentation assumées

- **Q heuristique** `diag(q_a·n + q_C, q_a)` au lieu de `G·Q·Gᵀ`.
- **Bornage de P** à `[p_min, p_init]` (stabilisant, mais **brise la consistance statistique** —
  exactement le défaut signalé par la littérature EKF).
- **Prédiction événementielle** : `predictKalman()` est déclenché **à la mesure** (fermeture de
  segment), pas à chaque pas de temps.

### C.5 Pipeline complet (état actuel, v1.7.0)

**Ordre interne de `update(V, I, T)`** (V = tension pack) :

1. `dt` depuis `coulombMeter->getLastInterval()` ; lissage `dVdt` (`updateDerivatives`).
2. `V_cell_ocv = correctVoltageToOCV(V,I,T,dt) / getCellCount()` (modèle Thévenin + thermique) ;
   `SoC_voltage = ocvToSoc(...)`.
3. `updateMpptState()` : `detectChargeState()` → **machine à états** (§C.5.1).
4. Intégration coulomb : `SoC_coulomb += dAh / C_eff × 100`.
5. `updateSegmentAndKalman()` — **AVANT** `handleSyncEvents()` (F2) : mesure de capacité (§C.5.2).
6. `handleSyncEvents()` : resynchros FLOAT / REST_LONG / cutoff basse tension (§C.5.3).
7. `SoC_fused = fuseSoC()` (filtre complémentaire, §C.2) ; incertitude ; `updateCycleDetection()`.
8. `mppt_state_prev = mppt_state` **en fin** de `update()` (F1).

#### C.5.1 Machine à états de charge

`detectChargeState()` (BatteryModels) renvoie `UNKNOWN/DISCHARGE/BULK/ABSORPTION/FLOAT/REST`.
BatteryKalman **promeut** `REST → REST_LONG` via une **porte de stabilité** (F8) : repos depuis
≥ `rest_long_min_min` **et** tension plate (écart crête ≤ seuil) sur une fenêtre glissante
`rest_long_stable_min`, alimentée par un **anneau temporellement décimé** (taille compile-time
`REST_LONG_STABLE_MAX_SAMPLES`). Le seuil de planéité est **auto-adaptatif au bruit** de mesure
(R6) et le profil (durées/seuil) est **réglable par technologie** (R7) — voir
[`rest_long_tuning.md`](rest_long_tuning.md).

#### C.5.2 Mesure de capacité (segments)

Un **segment** s'ouvre sur une référence (FLOAT = 100 %, ou REST_LONG) et se ferme à la
référence suivante. Il fournit la mesure de l'EKF :
```
z = |ΔAh| / (|ΔSoC|/100)   avec ΔAh = |Ah(fermeture) − Ah(ouverture)|,  ΔSoC = |SoC_fin − SoC_début|
```
Rejet si `ΔAh < seuil` (absolu et/ou % de C), `ΔSoC < min_dsoc[phase]`, ou `z ∉ [1, 2000]` Ah.
Le **gating OCV** (`ocv_segment_allowed=false`) désactive les segments pour NiFe/Na-ion.

#### C.5.3 Resynchronisations

- **FLOAT** (transition) → `SoC = 100 %` (conf 95 %).
- **REST_LONG** (transition, throttlé) → `SoC = SoC_voltage`, confiance croissante avec le repos.
- **Cutoff basse tension** → `SoC = 3 %` (porte temporelle 5 min, K9).
- **Alignement REST** → léger rappel vers `SoC_voltage` (porte 5 min).

#### C.5.4 Comptage de cycles & vieillissement

Un cycle partiel est compté à la transition **FLOAT** après décharge (`DoD = SoC_max − SoC_min`,
`cycles_partial += DoD/100`). Les cycles écoulés **entre deux mesures** sont accumulés
(`cycles_since_update`, K7) puis consommés par `predictKalman()` → c'est ce qui rend
`dC_dCycle` **observable** (sinon `F₀₁ = −Δ ≈ 0`, pas de covariance croisée `P₁₀`).

#### C.5.5 Hiérarchie de tuning

`getTuning()` résout, du plus fort au plus faible : **setter runtime > `model->getKalmanTuning()`
> macros compile-time > `KALMAN_DEFAULT_TUNING`**. `applyRecommendedRestLong()` applique un profil
REST_LONG par chimie (niveau 1). Les macros de tuning sont des **sentinelles** (`NAN`/`0`) testées
par `MACRO == MACRO` ; les vrais défauts vivent dans `KALMAN_DEFAULT_TUNING`.

#### C.5.6 Correctifs appliqués (F / K / R)

- **F1–F8** : transition d'état (`mppt_state_prev` en fin de `update`), ordre mesure/sync,
  `R_measured`, EKF 2D complet, fermeture REST_LONG, apprentissage hors `isAutoDetect`,
  ordre de `detectChargeState` (BatteryModels), porte de stabilité REST.
- **K1–K12** : anneau REST rotatif (K1), chrono sur état **brut** (K2), signe `dC_dCycle` (K3),
  saturation `millis()` (K4), `cycle_initialized` d'instance (K5), `phase_info` (K6),
  accumulation des cycles (K7), `last_Ah` toujours synchronisé en `begin()` (K8), cutoff basse
  tension temporisé (K9), taille d'anneau compile-time documentée (K10), tolérance au wrap
  `millis()` (K11), constante `REST_ALIGN_SYNC_MS` (K12).
- **R1–R7** : fenêtre REST réellement temporelle (R1), sémantique lookback (R2), plancher de
  cadence documenté (R3), sentinelle de boot (R4), artefact de preuve (R5), seuil de planéité
  adaptatif au bruit (R6), profils REST_LONG par technologie (R7).

Détail et preuves : [`../CHANGELOG.md`](../CHANGELOG.md) (Annexe K/R).

---

## D. Tableau comparatif synthétique

| Aspect | Canonique (EKF + ECM) | Batterie (DEKF / multi-échelle) | **BatteryKalman v1.7.0** |
|---|---|---|---|
| État du filtre | `[SoC, V1, V2]` | état **+** paramètres (2 filtres) | **`[C, dC/dcycle]`** |
| Mesure | tension terminale | tension / capacité | **capacité dérivée de segments OCV** |
| Non-linéarité dans le filtre | oui (OCV↔SoC) | oui | **non** (hors filtre) |
| SoC | état Kalman | état Kalman | **filtre complémentaire séparé** |
| SOH / capacité | parfois | filtre dual lent | **2ᵉ état du même filtre** |
| Q | `G·Q·Gᵀ` | `G·Q·Gᵀ` | **heuristique `diag(q_a·n+q_C, q_a)`** |
| P | libre (consistant) | libre | **borné `[p_min, p_init]`** |
| R | fixe ou adaptatif | adaptatif | **adaptatif (AEKF-lite)** |
| Prédiction | chaque pas | chaque pas | **événementielle (à la mesure)** |
| Dépendance points de référence | faible (tension continue) | moyenne | **forte (FLOAT / REST_LONG requis)** |

**Conclusion** : BatteryKalman occupe une position **originale** — un **estimateur de
paramètre unique** alimenté par des segments OCV, là où la littérature emploie soit un EKF-SoC
sur la tension, soit un **DEKF** séparant état (SoC) et paramètre (capacité). Ses choix
(Q heuristique, P borné, prédiction événementielle) sont des **variantes assumées**, pas des
erreurs, et expliquent le besoin de tuning par technologie.

---

## E. Rôle des 4 librairies dans la chaîne

```
CoulombsAh (Coulomb)        → intègre I·dt          (fournit ΔAh, mesure brute)
BatteryModels               → OCV / Thévenin / thermique, detectChargeState, getKalmanTuning()
BatteryKalman               → EKF 2D : capacité + vieillissement ; fusion SoC (complémentaire)
BatteryLifePredictor        → EFC / Miner / Rainflow / sulfatation + Kalman 1D (pente → RUL/EOL)
```

| Librairie | Rôle | Kalman ? |
|---|---|---|
| **CoulombsAh** | Compteur de charge (`Coulomb`, `AmpereHeure`), intégration `I·dt` via `micros()` | non |
| **BatteryModels** | Modèle physique (OCV, Thévenin, thermique), `detectChargeState()` (crée les **points de référence**), `getKalmanTuning()` (P/Q/R par chimie) | non (fournit les paramètres) |
| **BatteryKalman** | **EKF 2D** capacité + vieillissement ; fusion SoC par filtre complémentaire | **oui (2D)** |
| **BatteryLifePredictor** | Prédiction de vie : EFC, Miner, Rainflow, sulfatation + **Kalman 1D** sur la dérive de capacité | **oui (1D, RUL)** |

**Point d'attention** : il y a **deux filtres de Kalman distincts** dans la chaîne — le **2D**
de BatteryKalman (capacité/vieillissement) et le **1D** de BatteryLifePredictor (pente de
dégradation → EOL). La prédiction de durée de vie **n'est pas** dans BatteryKalman.

---

## F. Écarts assumés et pistes

1. **Naming** : « EKF » est cosmétique ici — le filtre est **linéaire** en l'état ; la
   non-linéarité est déportée (construction de la mesure + fusion complémentaire).
2. **Dépendance aux points de référence** : sans FLOAT/REST_LONG, aucune mesure → Bootstrap.
   C'est intrinsèque à l'approche OCV (limitation classique).
3. **Consistance statistique** : bornage de P + Q heuristique → `P` peu interprétable ; les
   « σ » affichés le sont donc avec prudence.
4. **SoC non pondéré** : le filtre complémentaire n'utilise pas les incertitudes relatives
   coulomb/tension (pas de gain optimal).
5. **Piste littérature — DEKF** : séparer un filtre **état** (SoC, sur la tension) et un filtre
   **paramètre** (capacité) **à rythme lent** éviterait le bornage heuristique de `P` et
   donnerait un SoC pondéré optimalement. C'est l'architecture dominante pour le **SoC + SOH**
   simultanés.

---

## G. Références

### Théorie Kalman / EKF

- Wikipedia — [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) (équations predict/update, Jacobiens, forme de Joseph).
- Wikipedia — [Extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter) (EKF discret : `f`, `h`, `F_k`, `H_k` ; limites, UKF, variantes robustes).
- Wikipedia — [State of charge](https://en.wikipedia.org/wiki/State_of_charge) (le Kalman filtering combine tension + coulomb-counting).
- kalmanfilter.net — [Kalman Filter 1D](https://www.kalmanfilter.net/kalman1d.html) (dérivation KF 1D : état, covariance, gain).
- Welch & Bishop, *An Introduction to the Kalman Filter* (2006) — référence pédagogique classique.

### Kalman appliqué aux batteries (SoC)

- MathWorks — [SOC Estimator (Kalman Filter)](https://www.mathworks.com/help/simscape-battery/ref/socestimatorkalmanfilter.html) (état `[SoC, V1(, V2)]`, `F`/`H` discrets, ECM 1RC/2RC).
- IEEE — [State-of-Charge Estimation Method for Lithium-Ion Batteries](https://ieeexplore.ieee.org/document/10223036) (Yun, 2023) — EKF temps réel.
- MDPI — [An Extended Kalman Filter Design for State-of-Charge Estimation](https://www.mdpi.com/2313-0105/9/12/583) (Zhou, 2023) — EKF variationnel.
- LJML — [State Of Charge estimation based on EKF](https://researchonline.ljmu.ac.uk/id/eprint/20919/1/2022141168.pdf) (comparaison EKF / UKF, modèle 2RC).
- Okra Solar — [Building a Lightweight State of Charge Algorithm](https://www.okrasolar.com/blog/building-a-lightweight-state-of-charge-algorithm) (EKF + ECM pour l'embarqué).

### Adaptive / Dual / Unscented KF (SoC + SOH)

- Stanford (Onori) — [State of Charge Estimation Using Extended Kalman Filters](https://pangea.stanford.edu/ERE/pdf/OnoriPDF/Conferences/47.pdf) (EKF + **AEKF** : Q adapté par analyse des innovations ; résistance + capacité).
- IEEE — [SoC and SoH estimation (Dual EKF)](https://ieeexplore.ieee.org/document/8916734) (Azis, 2019) — **DEKF** : 1er EKF = SoC, 2ᵉ = paramètres.
- Iowa State (Mil & Zambreno) — [Estimating SoC and SoH of Li-Ion Batteries](https://www.ece.iastate.edu/~zambreno/assets/pdf/MilZam15A.pdf) — **Dual UKF multi-échelle** (SOH lent).
- PHM Society — [SoC and SoH Estimation for Li-Ion (DEKF + ESC)](https://papers.phmsociety.org/index.php/phme/article/download/4032/2458) (Lee, 2024).
- MDPI — [Joint Estimation of SOC and SOH Based on Kalman Filter](https://www.mdpi.com/2673-3951/6/3/100) (Qin, 2025).
- ScienceDirect — [SoH estimation using dual adaptive UKF](https://www.sciencedirect.com/science/article/abs/pii/S2352152X24011423) (Fahmy, 2024) — **DAUKF** + Coulomb counting.

### Filtres du projet

- `src/BatteryKalman.h` (v1.7.0) — EKF 2D capacité + vieillissement ; `README.md` § *Principe de fonctionnement*.
- [Fo170/BatteryModels](https://github.com/Fo170/BatteryModels) — OCV/Thévenin/thermique, `detectChargeState`, `getKalmanTuning`.
- [Fo170/CoulombsAh](https://github.com/Fo170/CoulombsAh) — compteur de charge (`Coulomb.h`).
- [Fo170/BatteryLifePredictor](https://github.com/Fo170/BatteryLifePredictor) — EFC/Miner/Rainflow + **Kalman 1D** (RUL/EOL).
