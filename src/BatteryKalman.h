/**
 * @file BatteryKalman.h
 * @brief Filtre de Kalman étendu (EKF 2D) pour l'estimation de l'état de charge batterie
 *
 * Améliorations v1.5:
 * - État 2D: capacité (C) + taux vieillissement (dC/dcycle)
 * - Process noise Q basé sur l'aging
 * - EKF avec Jacobien 2x2
 * - Estimation en ligne de R (adaptive noise)
 * - Confiance continue (pas de phases discrètes)
 * - Hysteresis sur transitions
 *
 * v1.5.2 — CORRECTIONS DE BUGS (voir CHANGELOG en fin de fichier):
 * - F1: transition d'état MPPT correctement détectée (mppt_state_prev différé)
 * - F2: fermeture de segment mesurée AVANT reset coulomb (ordre update())
 * - F3: R adaptatif réellement appliqué (R_measured mis à jour)
 * - F4: EKF 2D complet (prediction aging + mise à jour dC_dCycle et P[1][1])
 * - F5: fermeture REST_LONG par transition (plus de seuil n>200 bloquant)
 * - F6: apprentissage Kalman indépendant de isAutoDetect()
 * - F7: (BatteryModels.h) ordre des tests detectChargeState corrigé
 *
 * v1.5.3 — F8: porte de stabilité REST (OCV fiable)
 * - Le point de référence REST_LONG ne dépend plus d'un délai fixe de 2h.
 *   REST_LONG est atteint quand: repos >= REST_LONG_MIN_MS ET tension stable
 *   (écart crête < REST_LONG_STABLE_MV) sur une fenêtre temporelle
 *   REST_LONG_STABLE_MS (indépendant du rythme d'échantillonnage).
 * - Réglages via constantes: REST_LONG_MIN_MS (15 min), REST_LONG_STABLE_MV,
 *   REST_LONG_STABLE_MS, REST_LONG_STABLE_MIN_SAMPLES,
 *   REST_LONG_STABLE_MAX_SAMPLES, REST_LONG_SYNC_MS.
 *
 * v1.6.0 — CONFIGURATION PAR TECHNOLOGIE (consomme BatteryModels v1.4+)
 * - getTuning() : hiérarchie de résolution centralisée
 *     1. override runtime (setters)  >  2. model->getKalmanTuning()
 *     >  3. macros compile-time  >  4. défaut interne (valeurs v1.5.3).
 * - Nouveaux setters runtime : setTuning(), setP(), setQ(), setR(),
 *   setSegmentThresholds(), setBatteryChange(), setRestLong(),
 *   setConfidence(), resetTuning().
 * - Seuil ΔAh relatif (seg_min_dAh_pct) pour petites batteries (NiMH/Alkaline).
 * - Gating des segments OCV (ocv_segment_allowed=false) pour NiFe/Sodium.
 * - REST_LONG (durées, stabilité) et confiance configurables par technologie.
 * - Aucun breaking change : sans setter ni BatteryModels v1.4+, le
 *   comportement est identique à v1.5.3.
 *
 * v1.7.0 — CHASSE AUX BUGS (K1–K12, R1–R7) + OBSERVATEUR PASSIF (P1/P2)
 * - K1–K12/R1–R7 : anneau de stabilité REST rotatif et temporel, REST_LONG
 *   stable (réarmement sur l'état brut, tolérance au wrap de millis()),
 *   vieillissement réellement appris (accumulation des cycles), signe du
 *   taux de vieillissement, synchro last_Ah à la restauration, porte
 *   temporelle du cutoff basse tension, seuil de planéité REST auto-adaptatif
 *   au bruit (R6) et profil REST_LONG par technologie (R7).
 * - P1 : mode pseudo-repos (opt-in) pour installations sans repos long.
 * - P2 : régression accumulée (observateur passif de capacité).
 * - Voir doc/rest_long_tuning.md et CHANGELOG.md (Annexe K/R).
 *
 * P1 — MODE PSEUDO-REPOS (opt-in) : pour les installations sans repos long
 * (décharge permanente la nuit, charge PV le jour), exploite les courts instants
 * à courant ~nul comme ancre OCV basse. C = Ah_déchargés / (100 − SoC_repos)/100,
 * combinée à l'ancre haute FLOAT (=100 %) et au comptage Ah. Activer via
 * enablePseudoRest() (désactivé par défaut : comportement v1.6.0 inchangé).
 *
 * P2 — RÉGRESSION ACCUMULÉE (observateur passif) : accumule les couples
 * (dSoC, dAh) de chaque segment valide et ajuste une droite par l'origine
 * (dAh = C·dSoC/100), pondérée par dSoC² (les décharges profondes recalent).
 * API : getRegressionCapacity(), getRegressionPoints(), getRegressionDispersion(),
 * getRegressionMaxDoD(), resetRegression().
 * 100 % passif (ne requiert aucun contrôle de la charge/décharge).
 *
 * Licence: GNU General Public License v3
 * Version: 2.0.0
 * Date: Octobre 2026
 *
 * v2.0.0 — QUALITÉ DES SEGMENTS (rupture de la circularité de `C_hat`)
 * ---------------------------------------------------------------
 * PROBLÈME. La fermeture d'un segment sur transition FLOAT posait
 * `SoC_end = 100.0f` — une HYPOTHÈSE, pas une mesure. `detectChargeState()`
 * déclare FLOAT à `I < max(0,6 ; C·0,02)` près de `V_float_per_cell` : la
 * batterie peut n'être qu'à 95 %. Cette hypothèse entrait dans `C_measured`,
 * donc dans `C_hat`, donc dans `SoC_coulomb`, donc dans la fermeture suivante.
 * Boucle fermée sur sa propre erreur. Mesure du 19/10/2026 : `C_hat` = 26,07 Ah
 * là où le cycle réel implique ≈ 37,5 Ah (erreur 17,3 points de SoC).
 *
 * CORRECTIF. Un segment porte désormais la NATURE de ses deux ancres, et cette
 * nature détermine l'incertitude de la mesure :
 *
 *     u_ancre [points de SoC] :  REST_LONG 1,0 | pseudo-repos 2,0 | FLOAT 5,0
 *     σ_C     = C_ref · u_ancre / dSoC          [Ah]   (propagation standard)
 *     R_ancre = σ_C²                            [Ah²]
 *     R       = max(R_measured, R_ancre)
 *
 * Une seule loi, trois constantes, plus aucune valeur magique par cas
 * particulier (l'ancien `PSEUDO_REST_R_FACTOR = 4` disparaît).
 *
 * Un segment dont le score de qualité (loi identique, cf. `SegmentValidator`
 * côté projet) tombe sous le seuil est REJETÉ avant d'atteindre l'EKF. Un
 * segment qui contredit la capacité indépendante de plus de 40 % est rejeté
 * s'il est ancré sur une hypothèse, mais ACCEPTÉ s'il est ancré sur une mesure
 * — dans ce cas c'est le modèle d'usure qui est suspect, et le drapeau
 * `wearSuspect` le signale.
 *
 * L'initialisation sur une ancre hypothétique reste AUTORISÉE (sinon `C_hat`
 * resterait à 0 et le bug §M de la doc projet — FLOAT détecté à 2 A près,
 * SoC forcé à 100 % — se réactiverait), mais avec `P = R_ancre` : le premier
 * segment ancré sur une mesure reçoit alors un gain quasi unitaire et corrige
 * en une seule fois.
 *
 * RAM : +12 octets sur `Segment` (3 floats), +24 octets d'état.
 * API : additive uniquement (`setCapacityPrior`, `setSegmentGuard`,
 * `getLastSegmentQuality`, `getSegmentRejected`, `getLastAnchorKind`).
 */

#ifndef BATTERY_KALMAN_H
#define BATTERY_KALMAN_H

// Version de la librairie (source UNIQUE — agrégée par src/liste_lib.h)
#define __BATTERYKALMAN_H_VERSION__ "2.4.0"

#include <Arduino.h>
#include <math.h>
#include "BatteryModels.h"
#include "CoulombsAh.h"
#include "SegmentValidator.h"

// ============================================================
// SURCHARGE COMPILE-TIME (OPTIONNELLE) — v1.6.0
// ============================================================
// Toutes les macros sont optionnelles et GLOBALES : elles écrasent le
// tuning résolu (niveau 3 de la hiérarchie) si (et seulement si) elles
// sont définies par l'utilisateur avant #include. Par défaut = sentinelle
// (NAN pour les float, 0 pour les entiers) = "non définie" → le réglage du
// modèle (niveau 2) s'applique. Les défauts réels vivent dans
// KALMAN_DEFAULT_TUNING (niveau 4 = valeurs v1.5.3).
// Exceptions: REST_LONG_STABLE_MAX_SAMPLES (taille compile-time de l'anneau)
// et REST_LONG_SYNC_MS (limiteur de synchro, hors tuning).

#ifndef KALMAN_P_INIT_C
#define KALMAN_P_INIT_C         NAN
#endif
#ifndef KALMAN_P_INIT_AGING
#define KALMAN_P_INIT_AGING     NAN
#endif
#ifndef KALMAN_P_MIN_C
#define KALMAN_P_MIN_C          NAN
#endif
#ifndef KALMAN_P_MIN_AGING
#define KALMAN_P_MIN_AGING      NAN
#endif

// Process noise (Q)
#ifndef Q_AGING_UNCERTAINTY
#define Q_AGING_UNCERTAINTY     NAN
#endif
#ifndef Q_CAPACITY_DRIFT
#define Q_CAPACITY_DRIFT        NAN
#endif

// Measurement noise (R) - estimation en ligne
#ifndef R_INIT
#define R_INIT                  NAN
#endif
#ifndef R_MIN
#define R_MIN                   NAN
#endif
#ifndef R_MAX
#define R_MAX                   NAN
#endif
#ifndef R_SMOOTH_ALPHA
#define R_SMOOTH_ALPHA          NAN
#endif

// Phases d'apprentissage (legacy, conservées)
#ifndef PHASE1_P_THRESHOLD
#define PHASE1_P_THRESHOLD      30.0f
#endif
#ifndef PHASE2_P_THRESHOLD
#define PHASE2_P_THRESHOLD      3.0f
#endif

// Seuils segments
#ifndef SEG_MIN_DAH
#define SEG_MIN_DAH             NAN
#endif
#ifndef SEG_MIN_DSOC_P0
#define SEG_MIN_DSOC_P0         NAN
#endif
#ifndef SEG_MIN_DSOC_P1
#define SEG_MIN_DSOC_P1         NAN
#endif
#ifndef SEG_MIN_DSOC_P2
#define SEG_MIN_DSOC_P2         NAN
#endif
#ifndef SEG_DSOC_HIGH_CONF
#define SEG_DSOC_HIGH_CONF      NAN
#endif

// Seuils de batterie changée
#ifndef BATTERY_CHANGE_THR
#define BATTERY_CHANGE_THR      NAN
#endif
#ifndef BATTERY_CHANGE_COUNT
#define BATTERY_CHANGE_COUNT    0
#endif

// ============================================================
// REFÉRENCE "REPOS" (OCV fiable) — F8 v1.5.3
// ============================================================
// Le point de référence REST_LONG n'est plus un simple délai fixe de 2h
// (inatteignable sur de nombreuses installations: il faut que la batterie
// soit réellement stabilisée). On utilise une PORTE DE STABILITÉ temporelle :
//   REST_LONG est atteint quand on est en REST depuis REST_LONG_MIN_MS
//   ET que la tension est stable (écart crête max < REST_LONG_STABLE_MV)
//   sur une fenêtre glissante de REST_LONG_STABLE_MS.
// Indépendant du rythme d'échantillonnage (10 Hz, 1 Hz, 5 min...).

#ifndef REST_LONG_MIN_MS
#define REST_LONG_MIN_MS        0UL         // sentinelle (ms) — remplacé par rest_long_min_min (min) du tuning
#endif
#ifndef REST_LONG_STABLE_MV
#define REST_LONG_STABLE_MV     NAN         // écart tension crête (V) toléré sur la fenêtre
#endif
#ifndef REST_LONG_STABLE_MS
#define REST_LONG_STABLE_MS     0UL         // sentinelle (ms) — remplacé par rest_long_stable_min (min) du tuning
#endif
#ifndef REST_LONG_STABLE_MIN_SAMPLES
#define REST_LONG_STABLE_MIN_SAMPLES 0      // sentinelle — nb min d'échantillons du tuning
#endif
#ifndef REST_LONG_STABLE_MAX_SAMPLES
#define REST_LONG_STABLE_MAX_SAMPLES 32     // taille max de l'anneau (sécurité mémoire, compile-time)
#endif
// K10 (documentation): la taille de l'anneau REST est pilotée EXCLUSIVEMENT par
// cette macro compile-time. Le champ homonyme `KalmanTuning::rest_long_stable_max_samples`
// (exposé par BatteryModels) n'est PAS consommé ici : le définir via setRestLong()/tuning
// n'a aucun effet. Pour changer la taille, définir REST_LONG_STABLE_MAX_SAMPLES avant #include.
#ifndef REST_LONG_SYNC_MS
#define REST_LONG_SYNC_MS       1800000UL   // délai entre 2 synchros REST_LONG (hors tuning)
#endif
#ifndef STATE_CONFIRM_MS
// v1.8.1 (#44) — durée pendant laquelle un nouvel état doit se maintenir avant
// de réarmer `state_entry_ms`. Couvre le bruit de courant (±20 mA crête) face
// au deadband de détection (±50 mA) : un aller-retour parasite d'un échantillon
// ne remet plus la minuterie à zéro.
#define STATE_CONFIRM_MS         3000UL   // 3 s
#endif

#ifndef REST_FUSE_START_MS
// v1.8.0 (D1) — début de la fusion tension↔coulomb en State_REST.
// Avant ce délai, la polarisation n'est pas relaxée : le coulomb seul fait foi.
#define REST_FUSE_START_MS       300000UL   // 5 min
#endif
#ifndef REST_FUSE_FULL_MS
// v1.8.0 (D1) — fin de la rampe : au-delà, la fusion atteint son poids nominal
// (alpha = 0,10, soit 90 % tension). Aligné sur τ_hyst ≈ 2000 s + marge.
#define REST_FUSE_FULL_MS        1800000UL  // 30 min
#endif
#ifndef REST_FUSE_MAX_DELTA_PCT
// v1.8.0 (D1 bis) — déplacement maximal du SoC publié par rapport au coulomb,
// en points de %, lors d'une fusion partielle. Empêche tout « saut » de SoC.
#define REST_FUSE_MAX_DELTA_PCT  2.0f
#endif

#ifndef REST_ALIGN_SYNC_MS
#define REST_ALIGN_SYNC_MS      300000UL    // délai d'alignement SoC au repos (hors tuning) — sert aussi de porte au cutoff basse tension (K9)
#endif
#ifndef REST_LONG_NOISE_K
#define REST_LONG_NOISE_K       5.0f        // R6: facteur d'adaptation du seuil de planéité au bruit de mesure
#endif

// ============================================================
// MODE PSEUDO-REPOS (P1) — ancre OCV courte, opt-in
// ============================================================
// Pour les installations SANS repos long (décharge permanente la nuit, charge
// PV le jour) : on exploite les courts instants à courant ~nul comme ancre OCV
// basse. Combinée à l'ancre haute FLOAT (=100 %) et au comptage Ah, elle permet
// d'estimer la capacité : C = Ah_decharges / (100 - SoC_repos)/100.
// Opt-in : désactivé par défaut (activer via enablePseudoRest()).
#ifndef PSEUDO_REST_CURRENT_MAX
#define PSEUDO_REST_CURRENT_MAX 0.05f       // |I| < seuil (A) = courant "nul"
#endif
#ifndef PSEUDO_REST_MIN_SAMPLES
#define PSEUDO_REST_MIN_SAMPLES 2           // nb min d'échantillons consécutifs
#endif
#ifndef PSEUDO_REST_MIN_MS
// ⚠️ v1.7.4 (B3) — DURÉE minimale de pseudo-repos, en plus du nombre d'échantillons.
// L'ancienne conception supposait des échantillons espacés de plusieurs minutes
// (cf. commentaire « 2 x 5 min = 10 min »), mais un firmware qui appelle update()
// à 10 Hz validait une ancre OCV après **0,2 s** de repos — bien trop court pour
// que la relaxation de polarisation soit terminée (τ_pol de plusieurs dizaines de
// secondes sur le plomb) ⇒ OCV surestimée en charge, sous-estimée en décharge, et
// C_hat biaisé. 30 s est un plancher conservateur.
#define PSEUDO_REST_MIN_MS      30000UL
#endif
// ⚠️ v2.0.0 : `PSEUDO_REST_R_FACTOR` et `PSEUDO_REST_SOC_SIGMA` ont été RETIRÉS.
// L'incertitude d'ancre est désormais portée par `ANCHOR_SOC_SIGMA_*` (ci-dessous)
// et s'applique à TOUTES les natures d'ancre, plus seulement au pseudo-repos.
// Un code appelant encore ces macros ne compilerait plus — c'est voulu : il
// utilisait une valeur arbitraire (×4) au lieu d'une propagation d'incertitude.
#ifndef PSEUDO_REST_TAU_MIN
#define PSEUDO_REST_TAU_MIN     0.0f        // P1: constante de temps de relaxation (min) pour extrapoler l'OCV (0 = pas d'extrapolation, conservateur)
#endif
#ifndef PSEUDO_REST_MAX_CORR
#define PSEUDO_REST_MAX_CORR    0.150f      // P1: correction de relaxation max (V), garde-fou
#endif
#ifndef PSEUDO_REST_MIN_AH
#define PSEUDO_REST_MIN_AH      2.0f        // P1: Ah de décharge minimum pour valider une ancre pseudo-repos (~1/3 de la décharge nocturne)
#endif

// ============================================================
// v2.0.0 — NATURE ET INCERTITUDE DES ANCRES DE SoC
// ============================================================
// Un segment relie DEUX ancres de SoC. Son résultat
//
//     C_measured = dAh / (dSoC / 100)          [Ah]
//
// n'est exact que si les deux ancres le sont. Or elles ne le sont pas :
//
//   REST_LONG      SoC_end = data->SoC_voltage   → MESURÉ (OCV relaxée)
//   pseudo-repos   SoC_end = ocvToSoc(V_relax)   → MESURÉ, moins précis
//   FLOAT          SoC_end = 100.0f              → HYPOTHÈSE
//
// `detectChargeState()` déclare FLOAT à `I < max(0,6 ; C·0,02)` près de
// `V_float_per_cell` : le courant peut encore valoir 0,5 A, la polarisation
// n'est pas relaxée, et la batterie n'est pas nécessairement pleine. L'écart
// réel entre 100 % et le SoC vrai est BORNÉ PAR LE BAS de la fenêtre de
// détection, pas par une mesure.
//
// Incertitude retenue, en POINTS de SoC (pas en % relatif) :
//   1,0 pt   REST_LONG     : OCV relaxée, table OCV à ±1 pt près
//   2,0 pt   pseudo-repos  : relaxation incomplète (PSEUDO_REST_MIN_MS = 30 s
//                            contre τ_pol de plusieurs minutes)
//   5,0 pt   FLOAT         : hypothèse « batterie pleine » sur une fenêtre de
//                            courant de 2 % de C
//
// La loi d'erreur est ensuite la MÊME pour les trois :
//    σ_C = C_ref · u_ancre / dSoC      [Ah]
// Une ancre à ±5 pt sur 40 pt de course donne σ_C = 12,5 % de C_ref ; la même
// ancre sur 5 pt de course donne σ_C = 100 % de C_ref — le segment n'informe
// plus sur rien, ce que la validation de qualité rejette.
#ifndef ANCHOR_SOC_SIGMA_REST
#define ANCHOR_SOC_SIGMA_REST       1.0f
#endif
#ifndef ANCHOR_SOC_SIGMA_PSEUDO
#define ANCHOR_SOC_SIGMA_PSEUDO     2.0f
#endif
#ifndef ANCHOR_SOC_SIGMA_FLOAT
#define ANCHOR_SOC_SIGMA_FLOAT      5.0f
#endif
#ifndef ANCHOR_SOC_SIGMA_UNKNOWN
#define ANCHOR_SOC_SIGMA_UNKNOWN  100.0f   // ancre non identifiée ⇒ inexploitable
#endif

// Seuil de la garde croisée capacité apprise / capacité indépendante (usure).
// Au-delà, les deux estimations ne peuvent pas décrire la même batterie.
#ifndef SEG_CROSS_GUARD_REL
#define SEG_CROSS_GUARD_REL         0.40f
#endif

// Score de qualité minimal pour accepter un segment (cf. SegmentValidator).
// ⚠️ Le calcul des facteurs vit dans `src/SegmentValidator.h` du PROJET : la
//    librairie ne dépend pas du projet, elle ne reçoit que le score.
#ifndef SEG_QUALITY_MIN
#define SEG_QUALITY_MIN             0.50f
#endif

/**
 * @enum AnchorKind
 * @brief Nature de l'ancre de SoC qui ferme (ou ouvre) un segment.
 *
 * `ANCHOR_NONE` = aucune ancre identifiée : le segment ne peut pas être fermé.
 */
enum AnchorKind : uint8_t {
    ANCHOR_NONE            = 0,
    ANCHOR_OCV_MESURE      = 1,   // REST_LONG : SoC_end = SoC_voltage
    ANCHOR_PSEUDO_REST     = 2,   // fin de pseudo-repos : ocvToSoc(V_relax)
    ANCHOR_FLOAT_HYPOTHESE = 3    // transition FLOAT : SoC_end = 100 % (hypothèse)
};

/** @brief Nom lisible d'une nature d'ancre (UI / logs). */
inline const char* anchorKindStr(uint8_t k) {
    switch (k) {
        case ANCHOR_OCV_MESURE:      return "OCV";
        case ANCHOR_PSEUDO_REST:     return "pseudo";
        case ANCHOR_FLOAT_HYPOTHESE: return "FLOAT";
        default:                     return "-";
    }
}

// ============================================================
// P2 — RÉGRESSION ACCUMULÉE (observateur passif)
// ============================================================
// Pour un IoT passif (ne contrôle ni charge ni décharge) : au lieu d'estimer la
// capacité segment par segment (très bruité), on accumule les couples
// (dSoC, dAh) de chaque segment valide et on ajuste une droite par l'origine :
//     dAh = C * dSoC/100   ->   pente = capacité
// Le bruit s'annule sur N points (sigma / sqrt(N)). 100% passif.
#ifndef REG_MAX_POINTS
#define REG_MAX_POINTS          32          // taille de l'anneau de points (compile-time)
#endif
#ifndef REG_MIN_DSOC
#define REG_MIN_DSOC            3.0f        // dSoC minimal (%) pour accepter un point
#endif

// (02/10/2026) Le plancher KALMAN_COARSE_ALPHA_MIN a été retiré : la fusion
// suit désormais « le coulomb est la référence » (alpha=1 en charge/décharge,
// alpha faible au repos), donc conf=0 ne peut plus ramener la fusion sur la
// tension seule. Cf. fuseSoC().

// ============================================================
// DÉFAUT INTERNE (niveau 4 de la hiérarchie) — valeurs v1.5.3
// ============================================================
static const KalmanTuning KALMAN_DEFAULT_TUNING = {
    500.0f,     // p_init_C
    0.10f,      // p_min_C
    0.000001f,  // p_init_aging
    0.0000001f, // p_min_aging
    0.000002f,  // q_aging_uncertainty
    0.01f,      // q_capacity_drift
    4.0f,       // r_init
    0.5f,       // r_min
    100.0f,     // r_max
    0.1f,       // r_smooth_alpha
    0.20f,      // seg_min_dAh_abs
    0.0f,       // seg_min_dAh_pct (désactivé : seuil absolu seul)
    {3.0f, 5.0f, 8.0f},   // seg_min_dsoc[3]
    15.0f,      // seg_dsoc_high_conf
    true,       // ocv_segment_allowed
    0.30f,      // battery_change_thr
    3,          // battery_change_count
    15.0f,      // rest_long_min_min (min)
    0.020f,     // rest_long_stable_mv (V)
    30.0f,      // rest_long_stable_min (min) — fenêtre recommandée plomb ; 15 min pour Li/NiMH
    6,          // rest_long_stable_min_samples
    32,         // rest_long_stable_max_samples
    0.95f,      // conf_float
    0.70f,      // conf_rest_base
    0.15f,      // conf_rest_span
    0.30f,      // conf_phase_coarse
    0.60f       // conf_phase_refine
};

// ============================================================
// PHASES D'APPRENTISSAGE (Legacy - pour compatibilité)
// ============================================================
enum LearningPhase : uint8_t {
    PHASE_BOOTSTRAP  = 0,
    PHASE_COARSE     = 1,
    PHASE_REFINE     = 2,
    PHASE_TRACK      = 3
};

static const char* const PHASE_NAMES[] = {
    "Bootstrap",
    "Convergence rapide",
    "Raffinement",
    "Suivi vieillissement"
};

// ============================================================
// DONNÉES SoC (état à persister)
// ============================================================
struct SoCData {
    float SoC_coulomb = 50.0f;
    float SoC_voltage = 50.0f;
    float SoC_voltage_raw = 50.0f;
    float SoC_fused = 50.0f;
    float SoC_uncertainty = 10.0f;
    bool  soc_is_raw = true;

    float Ah_total_charged = 0.0f;
    float Ah_total_discharged = 0.0f;

    float cycles_partial = 0.0f;
    uint32_t cycles_full = 0;
    float DoD_accumulated = 0.0f;

    uint32_t last_sync_time = 0;
    bool coulomb_initialized = false;
};

// ============================================================
// ÉTAT KALMAN 2D (à persister)
// ============================================================
struct KalmanState2D {
    // État: [C_hat, dC_dCycle]
    float    C_hat = 0.0f;              // Capacité estimée (Ah)
    float    dC_dCycle = 0.0005f;      // Taux vieillissement (Ah/cycle, >0 = perte)

    // Covariance P[2x2] = [[P_CC, P_Caging], [P_agingC, P_aging]]
    // (valeurs v1.5.3 — remplacées à begin()/resetLearning() par le tuning)
    float    P[2][2] = {{500.0f, 0},
                        {0, 0.000001f}};

    // Estimation en ligne de R
    // (valeurs v1.5.3 — remplacées à begin()/resetLearning() par le tuning)
    float    R_estimated = 4.0f;      // R estimé depuis innovations
    float    R_measured = 4.0f;       // R actuel pour mesure (F3: désormais mis à jour)

    uint16_t n_updates = 0;             // Nombre mises à jour
    bool     initialized = false;
    uint8_t  outlier_streak = 0;

    // Confiance continue [0, 1]
    float    confidence = 0.0f;         // 0=inconnu, 1=très confiant

    // Historique innovations pour estimation R
    float    innovation_sum_sq = 0.0f;
    uint16_t innovation_count = 0;
};

// ============================================================
// CLASSE PRINCIPALE
// ============================================================
class BatteryKalman {
private:
    SoCData*        data;
    KalmanState2D*  kalman;
    BatteryModel*   model;
    Ah_GD*          ahMeter;   // v1.9.0 : compteur fusionnel CoulombsAh

    ChargeState mppt_state = State_UNKNOWN;
    ChargeState mppt_state_prev = State_UNKNOWN;
    ChargeState mppt_raw_prev = State_UNKNOWN;
    uint32_t  state_entry_ms = 0;
    float     temp_c_last = 25.0f;   // v1.8.1 : dernière T mesurée (°C)
    // v1.8.1 (#44) : confirmation d'état avant réarmement de la minuterie.
    uint32_t  _stateCandidateSince = 0;
    ChargeState _stateCandidate = State_UNKNOWN;
    // F8: suivi de stabilité de tension pour la détection REST_LONG
    float     rest_v_ring[REST_LONG_STABLE_MAX_SAMPLES] = {0.0f};
    uint32_t  rest_v_ring_ms[REST_LONG_STABLE_MAX_SAMPLES] = {0};
    uint8_t   rest_v_count = 0;
    uint32_t  rest_v_last_push_ms = 0;
    bool      rest_v_has_push = false;
    float     rest_v_noise = 0.0f;      // R6: estimation du bruit de tension (EMA de |ΔV|) au repos
    bool      rest_stable = false;
    float     dVdt = 0.0f;
    float     V_prev = 0.0f;
    float     I_prev = 0.0f;
    float     V_prev_sample = 0.0f;     // R6: V de l'échantillon précédent (bruit de mesure)
    float     I_prev_sample = 0.0f;     // v2.0.0: I de l'échantillon précédent (bruit de mesure)

    // P1: mode pseudo-repos (ancre OCV courte, opt-in)
    bool      pseudo_rest_enabled = false;
    float     pseudo_rest_i_max = PSEUDO_REST_CURRENT_MAX;
    uint8_t   pseudo_rest_min_samples = PSEUDO_REST_MIN_SAMPLES;
    uint32_t  pseudo_rest_min_ms = PSEUDO_REST_MIN_MS;   // v1.7.4 (B3) : durée mini
    float     pseudo_rest_min_ah = PSEUDO_REST_MIN_AH;
    float     pseudo_rest_tau_min = PSEUDO_REST_TAU_MIN;
    uint8_t   pseudo_rest_count = 0;    // échantillons consécutifs à |I| < seuil
    float     pseudo_rest_v = 0.0f;     // tension extrapolée (OCV estimé)
    float     pseudo_rest_v_start = 0.0f;
    uint32_t  pseudo_rest_t0_ms = 0;
    bool      pseudo_rest_ended = false;// le pseudo-repos vient de se terminer (1 tour)
    uint16_t  pseudo_rest_events = 0;   // compteur (observabilité)

    // P2: régression accumulée (observateur passif)
    float     reg_dsoc[REG_MAX_POINTS] = {0.0f};
    float     reg_dah[REG_MAX_POINTS] = {0.0f};
    uint8_t   reg_count = 0;
    uint8_t   reg_head = 0;

    LearningPhase phase = PHASE_BOOTSTRAP;

    struct Segment {
        bool     active = false;
        uint8_t  anchor_start = ANCHOR_NONE;   // v2.0.0 : nature de l'ancre d'ouverture
        uint8_t  anchor_end   = ANCHOR_NONE;   // v2.0.0 : nature de l'ancre de fermeture
        int64_t  Uc_start = 0;      // v1.9.0 : micro-coulombs EXACTS
        float    SoC_start = 0.0f;
        uint32_t n = 0;
        // v2.0.0 : observables de qualité, accumulées sur la durée du segment.
        // `dV_ema`/`dI_ema` sont des moyennes exponentielles de |ΔV| et |ΔI|
        // entre deux échantillons consécutifs — c'est le bruit du capteur, pas
        // l'amplitude du signal (qui parcourt 1 V sur une charge complète et
        // n'a donc aucun sens comme critère de propreté).
        float    dV_ema = 0.0f;     // V
        float    dI_ema = 0.0f;     // A
        float    T_start = 0.0f;    // °C — pour la dérive thermique
        float    T_last = 0.0f;     // °C
    } seg;

    // v2.0.0 — état de qualité / garde croisée
    SegmentQuality _segQ;                            // dernier score détaillé
    uint8_t  _segReason       = 0;                  // SegReason (0 = OK)
    uint8_t  _lastAnchor      = ANCHOR_NONE;        // ancre de la dernière fermeture
    uint16_t _segRejected     = 0;                  // segments rejetés (cumul)
    uint16_t _segAccepted     = 0;                  // segments acceptés (cumul)
    bool     _initFromHypothesis = false;           // C_hat initialisé sur une hypothèse
    bool     _wearSuspect     = false;              // une ancre mesurée contredit l'usure
    float    _capacityPrior   = 0.0f;               // Ah — capacité indépendante (usure)
    float    _crossGuardRel   = SEG_CROSS_GUARD_REL;
    float    _lastDSoC        = 0.0f;               // % — dSoC du dernier segment fermé
    SegmentValidator::Limits _segLimits;            // seuils de qualité

    // ── v2.1.0 (F-03) — PEUKERT SUR LA CONVERSION Ah→SoC ────────────────
    // La capacité qui convertit les Ah en SoC n'était PAS dépendante du courant :
    // `SoC_coulomb += (dAh / C_hat) × 100` avec `C_hat` constant. Or la capacité
    // RÉELLEMENT restituable dépend du régime (loi de Peukert) : à 0,56 A de
    // décharge nocturne, une batterie dont `C_hat` = 26,07 Ah restitue
    // 26,07·(1,3/0,56)^0,4 = 36,5 Ah. Le SoC descendait donc 40 % trop vite
    // chaque nuit, puis l'ancre FLOAT le remettait à 100 % : c'est exactement
    // la signature « +29 Ah entrés, SoC qui baisse » relevée sur 8 jours.
    //
    // ⚠️ Ce n'est PAS un rendement. `η` (CoulombFct) pondère le COURANT avant
    // intégration — il reste la seule chose qui touche au comptage. Peukert ne
    // modifie QUE le diviseur de la conversion Ah→SoC. Les deux notions sont
    // distinctes : les confondre reproduirait le défaut corrigé en v5.24.
    bool     _peukertOn       = false;              // activé par le projet (plomb)
    float    _peukertN        = 1.0f;               // exposant (> 1 = actif)
    float    _peukertIref     = 1.3f;               // A — courant de référence
    float    _peukertTau      = 300.0f;             // s — lissage du courant
    // v5.41 (4ᵉ passe) — RÉGIME DE DÉCHARGE + RATIO PUBLIÉ LISSÉ.
    // Le modèle est volontairement SIMPLE, parce que trois versions successives
    // ont montré que chaque mécanisme ajouté créait un nouveau transitoire :
    //   • un seul état de régime : `_peukertIema` = |I| lissé **pendant la
    //     décharge uniquement**, RÉ-AMORCÉ sur le front montant (première
    //     décharge après une pause). Sans ce ré-amorçage, une décharge de 5 A
    //     suivie d'un repos puis d'une décharge de 0,5 A laissait le régime à
    //     5 A : le ratio valait 0,69 au lieu de 1,47 pendant 24 min (−53 % sur
    //     « Ah disponibles »).
    //   • `_peukertCRatio` = EMA de la cible → la grandeur PUBLIÉE. Elle est
    //     continue par construction, donc `Ah`, `aut_h`, `peuk_ratio` et
    //     `c_soc` ne sautent jamais.
    // ⚠️ La CONTINUITÉ ne dispense pas de l'EXACTITUDE : le régime doit être
    // ré-amorcé, sinon la valeur est fausse (mais lisse) pendant des minutes.
    float    _peukertIema      = 0.0f;   // A — |I| lissé pendant la décharge
    bool     _peukertIemaInit  = false;  // régime jamais amorcé
    bool     _peukertWasDis   = false;   // ÉTAT « en décharge » (hystérésis, cf. N21)
    bool     _peukertRatioInit = false;  // ratio jamais amorcé
    float    _peukertCRatio    = 1.0f;   // publié : C_soc / C_hat

    // Seuils du régime (A) — réglables par `setSoCPeukertThresholds()`.
    //   • `_peukertIActive` : en dessous, il n'y a pas de régime de décharge
    //     identifiable. À 0,12 A une décharge dure 10 jours et la loi de Peukert
    //     n'est plus mesurable ; la formule saturait de toute façon (×2 dès
    //     0,23 A). ⚠️ Coût assumé : une veille permanente à 0,12 A perd sa
    //     correction, soit ~5,5 pt de SoC par 24 h (mesuré).
    //   • `_peukertINoise` : seuil au-dessus duquel on considère qu'on n'est
    //     PLUS en décharge (courant positif = charge, ou bruit au repos).
    //     Il n'est donc PAS redondant avec `_peukertIActive` : c'est la borne de
    //     SORTIE de l'hystérésis (cf. N21).
    float    _peukertIActive = 0.15f;
    float    _peukertINoise  = 0.02f;

    // Constante de temps du lissage du RATIO PUBLIÉ (s). Plus courte que
    // `_peukertTau` (300 s, le régime) : le ratio doit suivre en quelques
    // minutes, sans sauter.
    static constexpr float PEUKERT_RATIO_TAU_S = 60.0f;

    float R_int_eff = 0.0f;
    bool  state_dirty = false;
    bool  cycle_initialized = false;
    int64_t last_Uc = 0;   // v1.9.0 : µC exacts (le float32 perdait les petits deltas)

    // Buffers de diagnostic (getters getPhaseInfo/getLastSyncInfo/getLastLearnInfo).
    // Surchargeables par -D pour les cibles à RAM serrée : le projet Coulometre
    // (ESP8266, correctif OOM du 02/10/2026) les trime à 24 o car il n'utilise
    // pas ces getters. Défauts = tailles d'origine (pas de troncature).
#ifndef KALMAN_PHASE_INFO_LEN
#define KALMAN_PHASE_INFO_LEN     80
#endif
#ifndef KALMAN_SYNC_INFO_LEN
#define KALMAN_SYNC_INFO_LEN      64
#endif
#ifndef KALMAN_LEARN_INFO_LEN
#define KALMAN_LEARN_INFO_LEN     80
#endif
    char  phase_info[KALMAN_PHASE_INFO_LEN] = "Bootstrap";
    char  last_sync_info[KALMAN_SYNC_INFO_LEN] = "aucune";
    char  last_learn_info[KALMAN_LEARN_INFO_LEN] = "aucun";
    float alpha_fused = 0.5f;

    struct CycleHistory {
        float SoC_max = 0.0f;
        float SoC_min = 100.0f;
        bool was_charging = false;
        bool was_discharging = false;
        uint32_t cycle_start_time = 0;
    } cycle;

    float last_cycles = 0.0f;           // Pour calcul delta_cycles
    float cycles_since_update = 0.0f;   // K7: cycles accumulés depuis la dernière mesure Kalman

    // ============================================================
    // TUNING PAR TECHNOLOGIE (v1.6.0)
    // ============================================================
    // Hiérarchie de résolution (du plus fort au plus faible) :
    //   1. override runtime (setters)          -> _tuningOverride
    //   2. model->getKalmanTuning()            -> valeur locale de résolution
    //   3. macros compile-time (optionnelles)  -> KALMAN_*_MACRO
    //   4. défaut interne                      -> KALMAN_DEFAULT_TUNING
    // _tuningResolved est re-résolu à chaque getTuning() (léger) : les
    // changements de technologie du modèle sont donc pris en compte.
    KalmanTuning _tuningOverride;
    KalmanTuning _tuningResolved;
    bool _tuningOverrideSet = false;

    void _tuningResolve() {
        KalmanTuning t = KALMAN_DEFAULT_TUNING;
        if (_tuningOverrideSet) {
            // setter runtime (niveau 1) : remplace tout, y compris les macros
            t = _tuningOverride;
        } else if (model) {
            // niveau 2 : réglages du modèle
            t = model->getKalmanTuning();
            // macros compile-time (niveau 3) > modèle (niveau 2)
            _applyMacroOverrides(t);
        } else {
            // macros compile-time (niveau 3) > défaut interne (niveau 4)
            _applyMacroOverrides(t);
        }
        _tuningResolved = t;
    }

    void _applyMacroOverrides(KalmanTuning& t) const {
        if (KALMAN_P_INIT_C == KALMAN_P_INIT_C)       t.p_init_C = KALMAN_P_INIT_C;
        if (KALMAN_P_INIT_AGING == KALMAN_P_INIT_AGING) t.p_init_aging = KALMAN_P_INIT_AGING;
        if (KALMAN_P_MIN_C == KALMAN_P_MIN_C)         t.p_min_C = KALMAN_P_MIN_C;
        if (KALMAN_P_MIN_AGING == KALMAN_P_MIN_AGING) t.p_min_aging = KALMAN_P_MIN_AGING;
        if (Q_AGING_UNCERTAINTY == Q_AGING_UNCERTAINTY) t.q_aging_uncertainty = Q_AGING_UNCERTAINTY;
        if (Q_CAPACITY_DRIFT == Q_CAPACITY_DRIFT)     t.q_capacity_drift = Q_CAPACITY_DRIFT;
        if (R_INIT == R_INIT)                         t.r_init = R_INIT;
        if (R_MIN == R_MIN)                           t.r_min = R_MIN;
        if (R_MAX == R_MAX)                           t.r_max = R_MAX;
        if (R_SMOOTH_ALPHA == R_SMOOTH_ALPHA)         t.r_smooth_alpha = R_SMOOTH_ALPHA;
        if (SEG_MIN_DAH == SEG_MIN_DAH)               t.seg_min_dAh_abs = SEG_MIN_DAH;
        if (SEG_MIN_DSOC_P0 == SEG_MIN_DSOC_P0)       t.seg_min_dsoc[0] = SEG_MIN_DSOC_P0;
        if (SEG_MIN_DSOC_P1 == SEG_MIN_DSOC_P1)       t.seg_min_dsoc[1] = SEG_MIN_DSOC_P1;
        if (SEG_MIN_DSOC_P2 == SEG_MIN_DSOC_P2)       t.seg_min_dsoc[2] = SEG_MIN_DSOC_P2;
        if (SEG_DSOC_HIGH_CONF == SEG_DSOC_HIGH_CONF) t.seg_dsoc_high_conf = SEG_DSOC_HIGH_CONF;
        if (BATTERY_CHANGE_THR == BATTERY_CHANGE_THR) t.battery_change_thr = BATTERY_CHANGE_THR;
        if (BATTERY_CHANGE_COUNT != 0)                t.battery_change_count = BATTERY_CHANGE_COUNT;
        if (REST_LONG_STABLE_MV == REST_LONG_STABLE_MV) t.rest_long_stable_mv = REST_LONG_STABLE_MV;
        if (REST_LONG_MIN_MS != 0UL)                  t.rest_long_min_min = (float)(REST_LONG_MIN_MS / 60000UL);
        if (REST_LONG_STABLE_MS != 0UL)               t.rest_long_stable_min = (float)(REST_LONG_STABLE_MS / 60000UL);
        if (REST_LONG_STABLE_MIN_SAMPLES != 0)        t.rest_long_stable_min_samples = REST_LONG_STABLE_MIN_SAMPLES;
    }

    // Helpers ms (conversions depuis le tuning)
    uint32_t _restLongMinMs() {
        const KalmanTuning& t = getTuning();
        return (uint32_t)(t.rest_long_min_min * 60000.0f);
    }
    uint32_t _restLongStableMs() {
        const KalmanTuning& t = getTuning();
        return (uint32_t)(t.rest_long_stable_min * 60000.0f);
    }

    // ============================================================
    // MÉTHODES PRIVÉES - KALMAN 2D
    // ============================================================

    void predictKalman(float delta_cycles) {
        if (!kalman->initialized) return;
        if (delta_cycles < 0.0f) delta_cycles = 0.0f;

        // --- F4 CORRECTION: prediction d'état avec vieillissement ---
        // C_new = C_old - dC_dCycle * delta_cycles
        kalman->C_hat -= kalman->dC_dCycle * delta_cycles;

        // Jacobien de transition F = [[1, -delta_cycles], [0, 1]]
        // P = F * P * F^T
        float F01 = -delta_cycles;
        float P00 = kalman->P[0][0];
        float P01 = kalman->P[0][1];
        float P10 = kalman->P[1][0];
        float P11 = kalman->P[1][1];

        // F * P
        float FP00 = P00 + F01 * P10;
        float FP01 = P01 + F01 * P11;
        float FP10 = P10;
        float FP11 = P11;

        // (F * P) * F^T
        float P00_new = FP00 + FP01 * F01;
        float P01_new = FP01;
        float P10_new = FP10 + FP11 * F01;
        float P11_new = FP11;

        // Process noise Q (v1.6.0: par technologie)
        const KalmanTuning& t = getTuning();
        float Q_C = t.q_aging_uncertainty * delta_cycles + t.q_capacity_drift;
        float Q_aging = t.q_aging_uncertainty;

        kalman->P[0][0] = min(P00_new + Q_C, t.p_init_C);
        kalman->P[0][1] = P01_new;
        kalman->P[1][0] = P10_new;
        kalman->P[1][1] = min(P11_new + Q_aging, t.p_init_aging);
    }

    void applyKalmanUpdate2D(float C_measured, float R, float delta_cycles) {
        if (!kalman->initialized) {
            kalman->C_hat = C_measured;
            kalman->dC_dCycle = 0.0005f;  // Valeur par défaut
            kalman->P[0][0] = R;
            kalman->P[1][1] = getTuning().q_aging_uncertainty;
            kalman->initialized = true;
            kalman->n_updates = 1;
            kalman->confidence = 0.2f;
            updatePhaseFromConfidence();
            state_dirty = true;
            return;
        }

        // Prédiction (grow P due to process noise + aging)
        predictKalman(delta_cycles);

        // Calcul innovation (résidu)
        float innovation = C_measured - kalman->C_hat;

        // Innovation variance: S = H*P*H' + R
        // H = [1, 0] car measurement = C_hat uniquement
        float S = kalman->P[0][0] + R;
        if (S <= 0) S = R;

        // Détection outlier 3-sigma
        const KalmanTuning& t = getTuning();
        float sigma = sqrtf(S);
        if (fabsf(innovation) > 3.0f * sigma) {
            kalman->outlier_streak++;

            // Batterie changée? (n outliers + magnitude check)
            if (kalman->outlier_streak >= t.battery_change_count &&
                fabsf(innovation) > t.battery_change_thr * kalman->C_hat) {
                kalman->C_hat = C_measured;
                kalman->dC_dCycle = 0.0005f;
                kalman->P[0][0] = R * 2.0f;
                kalman->P[1][1] = t.q_aging_uncertainty * 5.0f;
                kalman->n_updates = 3;
                kalman->confidence = 0.1f;
                kalman->outlier_streak = 0;
                snprintf(last_learn_info, sizeof(last_learn_info),
                         "Battery replaced: C=%.1fAh", C_measured);
                state_dirty = true;
                return;
            }

            // Outlier: double P (mais pas trop)
            kalman->P[0][0] *= 2.0f;
            kalman->P[1][1] = min(kalman->P[1][1] * 1.5f, t.q_aging_uncertainty * 10.0f);
            updatePhaseFromConfidence();
            state_dirty = true;
            return;
        }

        kalman->outlier_streak = 0;

        // --- F4 CORRECTION: gain complet 2D ---
        // K = P*H' / S  où H = [1, 0]
        float K_C = kalman->P[0][0] / S;
        float K_aging = kalman->P[1][0] / S;

        // State update: x = x + K * innovation
        kalman->C_hat += K_C * innovation;
        kalman->dC_dCycle += K_aging * innovation;

        // Covariance update: P = (I - K*H) * P
        float P_CC_new = (1.0f - K_C) * kalman->P[0][0];
        float P_Ca_new = (1.0f - K_C) * kalman->P[0][1];
        float P_aC_new = kalman->P[1][0] - K_aging * kalman->P[0][0];
        float P_aa_new = kalman->P[1][1] - K_aging * kalman->P[0][1];

        kalman->P[0][0] = max(getTuning().p_min_C, P_CC_new);
        kalman->P[0][1] = P_Ca_new;
        kalman->P[1][0] = P_aC_new;
        kalman->P[1][1] = max(getTuning().p_min_aging, P_aa_new);

        kalman->n_updates++;

        // Mise à jour estimation R (innovation-based)
        updateREstimate(innovation, S);

        // Confiance croît avec n_updates et P décroît
        float n_factor = min(1.0f, (float)kalman->n_updates / 10.0f);
        float p_factor = 1.0f - min(1.0f, kalman->P[0][0] / getTuning().p_init_C);
        kalman->confidence = (n_factor + p_factor) * 0.5f;

        updatePhaseFromConfidence();

        snprintf(last_learn_info, sizeof(last_learn_info),
                 "EKF: C=%.1fAh dC=%.6f conf=%.0f%% P=%.2f",
                 kalman->C_hat, kalman->dC_dCycle, kalman->confidence*100.0f, kalman->P[0][0]);

        state_dirty = true;
    }

    void updateREstimate(float innovation, float S) {
        // Estimation en ligne de R depuis sequence d'innovations
        // Assum: innovation ~ N(0, S) où S = P + R
        // Donc R_estimate = var(innovation) - P

        kalman->innovation_sum_sq += innovation * innovation;
        kalman->innovation_count++;

        if (kalman->innovation_count >= 5) {  // Moyenner sur 5 innovations
            float innovation_var = kalman->innovation_sum_sq / kalman->innovation_count;
            const KalmanTuning& t = getTuning();
            float R_est = max(t.r_min, innovation_var - kalman->P[0][0]);

            // Lissage exponentiel
            kalman->R_estimated = t.r_smooth_alpha * R_est +
                                  (1.0f - t.r_smooth_alpha) * kalman->R_estimated;
            kalman->R_estimated = constrain(kalman->R_estimated, t.r_min, t.r_max);

            // --- F3 CORRECTION: appliquer R adaptatif aux futures mesures ---
            kalman->R_measured = kalman->R_estimated;

            // Reset for next window
            kalman->innovation_sum_sq = 0.0f;
            kalman->innovation_count = 0;
        }
    }

    void updatePhaseFromConfidence() {
        // Legacy: mapper confiance en phases discrètes
        const KalmanTuning& t = getTuning();
        if (!kalman->initialized) {
            phase = PHASE_BOOTSTRAP;
        } else if (kalman->confidence < t.conf_phase_coarse) {
            phase = PHASE_COARSE;
        } else if (kalman->confidence < t.conf_phase_refine) {
            phase = PHASE_REFINE;
        } else {
            phase = PHASE_TRACK;
        }
        snprintf(phase_info, sizeof(phase_info), "%s", PHASE_NAMES[phase]);
    }

    void updateDerivatives(float V, float I, float dt_s) {
        if (dt_s <= 0.0f || V_prev <= 0.0f) {
            V_prev = V; I_prev = I; return;
        }
        float alpha_f = dt_s / (20.0f + dt_s);
        dVdt = alpha_f * ((V - V_prev) / dt_s) + (1.0f - alpha_f) * dVdt;
        V_prev = V;
        I_prev = I;
    }

    // P1: détection des courts instants à courant ~nul (pseudo-repos).
    // Sur une installation sans repos long (décharge de nuit, charge PV de jour),
    // ces instants fournissent une ancre OCV basse approximative. On retient la
    // DERNIÈRE tension du repos (la plus relaxée) et on signale sa fin.
    void updatePseudoRest(float V, float I) {
        pseudo_rest_ended = false;
        if (!pseudo_rest_enabled) return;

        if (fabsf(I) < pseudo_rest_i_max) {
            if (pseudo_rest_count == 0) {
                pseudo_rest_v_start = V;
                pseudo_rest_t0_ms = millis();
            }
            pseudo_rest_count++;
            pseudo_rest_v = V;
        } else {
            // v1.7.4 (B3) : une ancre OCV n'est valide que si le repos a duré
            // assez longtemps POUR QUE la relaxation soit terminée. Le seul
            // comptage d'échantillons est insuffisant : à 10 Hz, 2 échantillons
            // = 0,2 s, alors que τ_pol vaut des dizaines de secondes sur le plomb.
            uint32_t elapsed = millis() - pseudo_rest_t0_ms;
            bool long_enough = (elapsed >= pseudo_rest_min_ms);
            if (pseudo_rest_count >= pseudo_rest_min_samples && long_enough) {
                // Extrapolation de relaxation : la tension remonte vers l'OCV
                // (V_inf = V + pente x tau). Corrige le biais "repos court".
                if (pseudo_rest_count >= 2 && elapsed > 0) {
                    float slope = (pseudo_rest_v - pseudo_rest_v_start) / (elapsed / 60000.0f);
                    float corr = slope * pseudo_rest_tau_min;
                    if (corr >  PSEUDO_REST_MAX_CORR) corr =  PSEUDO_REST_MAX_CORR;
                    if (corr < -PSEUDO_REST_MAX_CORR) corr = -PSEUDO_REST_MAX_CORR;
                    pseudo_rest_v += corr;
                }
                pseudo_rest_ended = true;
                pseudo_rest_events++;
            }
            pseudo_rest_count = 0;
        }
    }

    void updateMpptState(float V, float I) {
        float C_ref = getEffectiveCapacity();
        mppt_state = model->detectChargeState(V, I, C_ref);

        // --- F1 CORRECTION: mppt_state_prev n'est PLUS mis à jour ici ---
        // Il ne reflète plus la transition de ce même échantillon. La détection
        // de transition (FLOAT/REST_LONG) dans handleSyncEvents,
        // updateSegmentAndKalman et updateCycleDetection devient possible.
        // mppt_state_prev sera mis à jour en FIN de update().
        // ⚠️ v1.8.1 (#44) — RÉARMEMENT AVEC HYSTÉRÉSIS.
        // Avant, `state_entry_ms` repartait à zéro au MOINDRE changement d'état
        // brut. Or `detectChargeState()` utilise un deadband dur de ±0,05 A
        // alors que le courant de repos réel de cette installation vaut 0,035 A :
        // un SEUL échantillon bruité à −0,055 A faisait basculer REST→Discharge
        // et remettait la minuterie à zéro. Conséquences :
        //   • la rampe de fusion (REST_FUSE_START_MS) ne s'activait JAMAIS ;
        //   • REST_LONG (même minuterie) n'était jamais atteint, donc le doSync
        //     « Long rest OCV » ne partait jamais non plus.
        // Nouvelle règle : l'état doit rester DIFFÉRENT pendant
        // STATE_CONFIRM_MS avant de réarmer la minuterie. Un aller-retour
        // parasite (bruit) ne réinitialise donc plus rien.
        if (mppt_state != mppt_raw_prev) {
            if (mppt_state != _stateCandidate) {
                _stateCandidate      = mppt_state;
                _stateCandidateSince = millis();
            } else if (millis() - _stateCandidateSince >= STATE_CONFIRM_MS) {
                state_entry_ms   = _stateCandidateSince;
                mppt_raw_prev    = mppt_state;
            }
        } else {
            // Retour à l'état stable : on abandonne la candidature en cours.
            _stateCandidate = mppt_state;
        }

        // --- F8: porte de stabilité pour REST -> REST_LONG ---
        // On alimente un anneau (temps, tension); REST_LONG n'est atteint
        // que si (a) repos >= rest_long_min_min (tuning), et (b) tension
        // stable sur la fenêtre rest_long_stable_min (écart crête <
        // rest_long_stable_mv). Durées config par technologie (v1.6.0).
        if (mppt_state == State_REST) {
            uint32_t stable_ms = _restLongStableMs();
            uint32_t now_ms = millis();

            // K11 CORRECTION: tolérance au wrap de millis() (~49,7 j). Si l'horloge
            // a rebouclé depuis la dernière écriture, l'anneau contient des
            // horodatages du cycle précédent (ms ≈ 4,29e9) qui ne seraient plus
            // purgés par le cutoff → écart crête gonflé → REST_LONG différé. On
            // vide alors l'anneau pour repartir d'une fenêtre propre.
            if (rest_v_has_push && now_ms < rest_v_last_push_ms) {
                rest_v_count = 0;
                rest_v_last_push_ms = 0;
                rest_v_has_push = false;
                for (uint8_t i = 0; i < REST_LONG_STABLE_MAX_SAMPLES; i++) rest_v_ring_ms[i] = 0;
            }

            // K1 + R1: anneau réellement temporel. On n'écrit qu'au plus un
            // échantillon par stable_ms/MAX afin que l'anneau (taille fixe)
            // couvre toute la fenêtre stable_ms quel que soit le rythme
            // d'échantillonnage (sinon, à 1-10 Hz, il ne couvrait que ~3 s).
            // R3: la cadence doit rester <= stable_ms/(min_samples-1) pour que
            // l'anneau contienne assez d'échantillons (sinon REST_LONG jamais atteint).
            uint32_t push_ms = stable_ms / REST_LONG_STABLE_MAX_SAMPLES;
            if (push_ms == 0) push_ms = 1;
            if (!rest_v_has_push || (now_ms - rest_v_last_push_ms) >= push_ms) {
                uint8_t widx = rest_v_count % REST_LONG_STABLE_MAX_SAMPLES;
                rest_v_ring[widx] = V;
                rest_v_ring_ms[widx] = now_ms;
                rest_v_count = (uint8_t)((rest_v_count + 1) % REST_LONG_STABLE_MAX_SAMPLES);
                rest_v_last_push_ms = now_ms;
                rest_v_has_push = true;
            }

            // Conserver uniquement les échantillons dans la fenêtre temporelle
            // et calculer l'écart crête (ptp) sur ceux-ci.
            uint32_t cutoff = (now_ms > stable_ms) ? (now_ms - stable_ms) : 0UL;
            float v_min = 9999.0f, v_max = -9999.0f;
            uint8_t kept = 0;
            for (uint8_t i = 0; i < REST_LONG_STABLE_MAX_SAMPLES; i++) {
                if (rest_v_ring_ms[i] == 0) continue;
                if (rest_v_ring_ms[i] < cutoff) { rest_v_ring_ms[i] = 0; continue; }
                v_min = min(v_min, rest_v_ring[i]);
                v_max = max(v_max, rest_v_ring[i]);
                kept++;
            }
            const KalmanTuning& t = getTuning();

            // R6 (auto-adaptatif): seuil de planéité relatif au bruit de mesure.
            // Si le bruit (ADC/capteur) gonfle l'écart crête, on relâche le seuil
            // (borné à 3x le seuil de base) au lieu de bloquer REST_LONG.
            if (V_prev_sample > 0.0f) {
                float dV = fabsf(V - V_prev_sample);
                rest_v_noise = (rest_v_noise <= 0.0f) ? dV : (0.95f * rest_v_noise + 0.05f * dV);
            }
            float stable_mv_eff = t.rest_long_stable_mv;
            float noise_floor = REST_LONG_NOISE_K * rest_v_noise;
            if (noise_floor > stable_mv_eff) {
                stable_mv_eff = min(noise_floor, 3.0f * t.rest_long_stable_mv);
            }

            bool time_ok = (millis() - state_entry_ms >= _restLongMinMs());
            bool stable_ok = (kept >= t.rest_long_stable_min_samples)
                          && (v_max - v_min <= stable_mv_eff);
            rest_stable = stable_ok;

            if (time_ok && stable_ok) {
                mppt_state = State_REST_LONG;
            }
        } else {
            rest_v_count = 0;
            rest_v_last_push_ms = 0;
            rest_v_has_push = false;
            for (uint8_t i = 0; i < REST_LONG_STABLE_MAX_SAMPLES; i++) rest_v_ring_ms[i] = 0;
            rest_stable = false;
        }
    }

    void handleSyncEvents(float V_cell_ocv, float temp_c_meas = 25.0f) {
        float total_ocv = V_cell_ocv * model->getCellCount();
        const KalmanTuning& t = getTuning();

        // F1: ces transitions sont maintenant réellement détectées
        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            doSync(100.0f, t.conf_float, "Float charge", ANCHOR_FLOAT_HYPOTHESE);
            return;
        }

        if (mppt_state == State_REST_LONG && millis() - data->last_sync_time > REST_LONG_SYNC_MS) {
            uint32_t rest_min_ms = _restLongMinMs();
            uint32_t elapsed = millis() - state_entry_ms;
            uint32_t extra = (elapsed > rest_min_ms) ? (elapsed - rest_min_ms) : 0UL;
            float conf = t.conf_rest_base + t.conf_rest_span * min(1.0f, extra / (float)rest_min_ms);
            doSync(data->SoC_voltage, conf, "Long rest OCV", ANCHOR_OCV_MESURE);
            return;
        }

        // v1.8.1 (#51) — `total_ocv` est un OCV À LA TEMPÉRATURE DE MESURE (cf.
        // BatteryModels 1.6.11), alors que `getMinVoltage()` est défini à 25 °C.
        // Comparer les deux décale le cutoff de dVocv_dT·(T−25) (≈ −45 mV pack à
        // 5 °C). On ramène donc l'OCV à 25 °C avant comparaison.
        float min_voltage = model->getMinVoltage();
        {
            // removeThermalCorrectionOCV attend une tension PAR CELLULE
            float per_cell = model->removeThermalCorrectionOCV(
                                 total_ocv / model->getCellCount(), temp_c_meas)
                             * model->getCellCount();
            if (isfinite(per_cell) && per_cell > 0.0f) total_ocv = per_cell;
        }
        // K9 CORRECTION: porte temporelle sur le cutoff basse tension. Sans elle,
        // doSync() était appelé à CHAQUE échantillon tant que la batterie restait
        // au repos sous V_min → state_dirty permanent (écriture persistance en
        // continu) et reset coulomb répété. On limite à une synchro par 5 min
        // (last_sync_time est mis à jour par doSync()).
        if ((mppt_state == State_REST || mppt_state == State_REST_LONG) && total_ocv < min_voltage
            && millis() - data->last_sync_time > REST_ALIGN_SYNC_MS) {
            doSync(3.0f, 0.75f, "Low voltage cutoff", ANCHOR_OCV_MESURE);
            return;
        }

        // ⚠️ v1.8.1 (D1 ter) — CETTE RÉALIGNEMENT EST SUPPRIMÉ.
        // Il faisait `SoC_coulomb = 0,9·SoC_coulomb + 0,1·SoC_voltage` toutes les
        // REST_ALIGN_SYNC_MS (= 300 000 ms = exactement REST_FUSE_START_MS),
        // c'est-à-dire qu'il modifiait DIRECTEMENT le SoC coulomb en contournant
        // complètement `fuseSoC()`. Conséquence : la rampe progressive et le
        // plafond ±2 pt introduits en 1.8.0 étaient neutralisés — le « saut » de
        // SoC n'était pas supprimé, il était DÉPLACÉ de SoC_fused vers
        // SoC_coulomb (mesuré le 19/10 : −0,91 pt toutes les 5 min, soit −5,5 pt
        // sur 30 min, sans rampe ni plafond, et sans poser `state_dirty`).
        //
        // La fusion tension↔coulomb en REST est désormais assurée UNIQUEMENT par
        // `fuseSoC()`, qui porte la rampe temporelle et le plafond. Le coulomb
        // reste ainsi la référence factuelle (principe du 02/10/2026).
        //
        // ⚠️ Le cutoff basse tension (bloc précédent) reste, lui, un `doSync()`
        // explicite : c'est une protection, pas un recalage progressif.
        (void)0;
    }

    /**
     * @param anchor v2.0.0 : nature de l'ancre posée par cette synchro. Elle
     *               détermine l'incertitude du segment qui DÉMARRE ici (le SoC
     *               de départ) et celle du segment qui se fermera dessus.
     *               Par défaut ANCHOR_NONE = « origine inconnue » (synchro
     *               manuelle) : le segment hérite alors de l'incertitude
     *               maximale, donc d'un poids quasi nul.
     */
    void doSync(float new_SoC, float confidence, const char* reason,
                uint8_t anchor = ANCHOR_NONE) {
        data->SoC_coulomb = new_SoC;
        data->SoC_fused = new_SoC;
        data->last_sync_time = millis();
        // v1.9.0 : plus de reset du compteur. Le compteur est un odomètre
        // monotone ; c'est `last_Uc` qui sert de référence pour le delta.
        // (Avant, `reset()` était nécessaire uniquement pour éviter la perte de
        // précision d'un float32 qui grandit — problème résolu par l'int64.)
        last_Uc = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;

        snprintf(last_sync_info, sizeof(last_sync_info),
                 "%s conf=%.0f%%", reason, confidence * 100.0f);
        startNewSegment(new_SoC, anchor);
        state_dirty = true;
    }

    float fuseSoC() {
        if (phase == PHASE_BOOTSTRAP) {
            alpha_fused = 0.0f;
            data->soc_is_raw = true;
            data->SoC_voltage_raw = (V_prev > 1.0f)
                ? model->ocvToSoc(V_prev, 25.0f)
                : data->SoC_voltage;
            return data->SoC_voltage_raw;
        }

        data->soc_is_raw = false;

        // ⚠️ PRINCIPE « LE COULOMB EST LA RÉFÉRENCE » (02/10/2026) :
        // Le comptage coulomb est FACTUEL (mesuré), il ne doit jamais être
        // remplacé par une estimation tension sous courant. La tension/OCV ne
        // sert d'ancre qu'au REPOS (équilibre) et en FLOAT (100 %). En charge
        // (BULK/ABSORPTION) et en décharge, alpha=1 → SoC_fused = SoC_coulomb.
        // (Avant : alpha_base × conf en COARSE → conf=0 rendait la fusion =
        //  tension pure, d'où saut de SoC. On ne pondère plus par la confiance.)
        float alpha_base;
        switch (mppt_state) {
            case State_FLOAT:      alpha_base = 0.00f; break;  // ancre 100 %
            case State_REST_LONG:  alpha_base = 0.02f; break;  // OCV très fiable (repos)
            case State_REST:
                // ⚠️ v1.8.0 (D1) — fusion PROGRESSIVE selon la durée du repos.
                // Avant, alpha_base valait 0,10 DÈS la première seconde de
                // State_REST : le SoC publié devenait 90 % piloté par la tension
                // alors que la polarisation n'était pas relaxée (τ_pol ≈ 12 s,
                // τ2 = 300 s, τ_hyst = 2000 s ⇒ il faut ~4 h pour l'équilibre).
                // Mesure du 19/10/2026 à 09:00 (repos de 15 min à 0,035 A après
                // 14 h de décharge) : SoC_fused = 0,1·61,6 + 0,9·51,5 = 52,5 %
                // au lieu de 61,6 % → saut de −9,1 points sur le SoC publié, et
                // l'autonomie (getAhAvailable) suivait (−2,35 Ah).
                //
                // Nouvelle pondération, continue en durée :
                //   [0 ; REST_FUSE_START[   : alpha = 1,00 (coulomb seul)
                //   [START ; REST_FUSE_FULL[ : rampe linéaire 1,00 → 0,10
                //   ≥ REST_FUSE_FULL        : alpha = 0,10 (OCV considérée fiable)
                {
                    uint32_t rest_ms = millis() - state_entry_ms;
                    if (rest_ms < REST_FUSE_START_MS) {
                        alpha_base = 1.00f;
                    } else if (rest_ms < REST_FUSE_FULL_MS) {
                        float t = (float)(rest_ms - REST_FUSE_START_MS)
                                / (float)(REST_FUSE_FULL_MS - REST_FUSE_START_MS);
                        alpha_base = 1.00f - (1.00f - 0.10f) * t;
                    } else {
                        alpha_base = 0.10f;
                    }
                }
                break;
            default:               alpha_base = 1.00f; break;  // BULK/ABSORPTION/DISCHARGE → coulomb
        }
        alpha_fused = alpha_base;

        float fused = alpha_fused * data->SoC_coulomb + (1.0f - alpha_fused) * data->SoC_voltage;

        // ⚠️ v1.8.0 (D1 bis) — PLAFOND de correction par cycle.
        // Même avec la rampe ci-dessus, une ancre OCV aberrante (polarisation
        // résiduelle, offset de mesure) peut déplacer le SoC publié de plusieurs
        // points. On borne le DÉPLACEMENT à REST_FUSE_MAX_DELTA_PCT points par
        // rapport au coulomb : le SoC ne peut plus « sauter », il converge.
        // (Seules les fusions partielles sont plafonnées : en FLOAT alpha=0 est
        // une ancre dure voulue, en BULK/DISCHARGE alpha=1 est un no-op.)
        if (alpha_fused > 0.0f && alpha_fused < 1.0f) {
            float delta = fused - data->SoC_coulomb;
            if (delta >  REST_FUSE_MAX_DELTA_PCT) fused = data->SoC_coulomb + REST_FUSE_MAX_DELTA_PCT;
            if (delta < -REST_FUSE_MAX_DELTA_PCT) fused = data->SoC_coulomb - REST_FUSE_MAX_DELTA_PCT;
            if (fused < 0.0f)   fused = 0.0f;
            if (fused > 100.0f) fused = 100.0f;
        }

        return fused;
    }

    float computeUncertainty() {
        float base;
        switch (phase) {
            case PHASE_BOOTSTRAP: base = 15.0f; break;
            case PHASE_COARSE:    base = 10.0f; break;
            case PHASE_REFINE:    base =  6.0f; break;
            case PHASE_TRACK:     base =  3.0f; break;
            default:              base = 15.0f;
        }
        if (mppt_state == State_BULK || mppt_state == State_DISCHARGE) base += 2.0f;
        return base;
    }

    // ------------------------------------------------------------
    // v2.0.0 — LOIS PURES DE QUALITÉ (testables sans machine à états)
    // ------------------------------------------------------------
public:
    /**
     * @brief Incertitude d'une ancre de SoC, en POINTS de SoC.
     *
     * Une ancre non identifiée reçoit 100 points : sur une batterie réelle,
     * 100 points d'incertitude signifient « aucun SoC connu », donc un segment
     * inexploitable. C'est volontairement rédhibitoire plutôt que silencieux.
     */
    static float anchorSoCUncertainty(uint8_t kind) {
        switch (kind) {
            case ANCHOR_OCV_MESURE:      return ANCHOR_SOC_SIGMA_REST;
            case ANCHOR_PSEUDO_REST:     return ANCHOR_SOC_SIGMA_PSEUDO;
            case ANCHOR_FLOAT_HYPOTHESE: return ANCHOR_SOC_SIGMA_FLOAT;
            default:                     return ANCHOR_SOC_SIGMA_UNKNOWN;
        }
    }

    /**
     * @brief Écart-type de la capacité déduite d'un segment, en Ah.
     *
     *     C_measured = dAh / (dSoC/100)
     *     σ_C = C_measured · (u_ancre / dSoC)      [Ah]
     *
     * Démonstration : u_ancre est une erreur absolue sur le SoC en points, donc
     * l'erreur relative sur dSoC vaut u/dSoC, et C_measured est inversement
     * proportionnel à dSoC — d'où la même erreur relative sur C.
     *
     * ⚠️ On scale sur `C_ref` (capacité de RÉFÉRENCE), pas sur `C_measured` :
     *    une mesure aberrante gonflerait sa propre barre d'erreur et se
     *    protégerait du rejet. `C_ref` vient de l'extérieur du segment.
     *
     * @param cRefAh   Capacité de référence (Ah), > 0
     * @param dsocPct  Course de SoC du segment (points), > 0
     * @param kind     Nature de l'ancre de fermeture
     * @return σ_C en Ah ; +Inf si les entrées interdisent le calcul
     */
    static float anchorCapacitySigma(float cRefAh, float dsocPct, uint8_t kind) {
        if (!isfinite(cRefAh) || !isfinite(dsocPct)) return INFINITY;
        if (cRefAh <= 0.0f || dsocPct <= 0.0f) return INFINITY;
        return cRefAh * (anchorSoCUncertainty(kind) / dsocPct);
    }

    /**
     * @brief Variance de mesure `R` apportée par l'ancre, en Ah².
     * @see anchorCapacitySigma
     */
    static float anchorCapacityVariance(float cRefAh, float dsocPct, uint8_t kind) {
        float s = anchorCapacitySigma(cRefAh, dsocPct, kind);
        if (!isfinite(s)) return INFINITY;
        return s * s;
    }

    /** @brief Issues de la garde croisée capacité apprise / capacité indépendante. */
    enum GuardResult : uint8_t {
        GUARD_OK              = 0,  // les deux estimations sont compatibles
        GUARD_REJECT          = 1,  // segment ancré sur hypothèse : rejeté
        GUARD_WEAR_SUSPECT    = 2   // segment ancré sur MESURE : l'usure est suspecte
    };


    /**
     * @brief Garde croisée : le segment contredit-il la capacité indépendante ?
     *
     *     gap = |C_measured − C_prior| / C_prior
     *
     * RÈGLE ASYMÉTRIQUE — et pourquoi elle doit l'être :
     *   • ancre HYPOTHÈSE (FLOAT) → le segment n'a pas d'autorité : rejet.
     *     Sans cela, `C_hat` resterait tiré par une supposition à 100 %.
     *   • ancre MESURÉE (OCV) → le segment a l'autorité : on l'accepte et on
     *     marque le MODÈLE D'USURE comme suspect. Un rejet inconditionnel
     *     figerait `C_hat` sur une valeur fausse pour toujours — c'est
     *     exactement le piège à éviter (`capacity_current` est initialisée à
     *     72 Ah × (1 − 50 %) sans aucune mesure).
     *
     * @param cMeasAh   Capacité déduite du segment (Ah)
     * @param cPriorAh  Capacité indépendante (Ah) ; <= 0 ⇒ garde inactive
     * @param tolRel    Tolérance relative (0,40 = 40 %)
     * @param kind      Nature de l'ancre de fermeture
     */
    static uint8_t crossGuard(float cMeasAh, float cPriorAh, float tolRel,
                              uint8_t kind) {
        if (!isfinite(cMeasAh) || !isfinite(cPriorAh)) return GUARD_REJECT;
        if (!(cPriorAh > 0.0f)) return GUARD_OK;          // pas de référence
        if (!(tolRel > 0.0f)) return GUARD_OK;            // garde désactivée
        float gap = fabsf(cMeasAh - cPriorAh) / cPriorAh;
        if (gap <= tolRel) return GUARD_OK;
        return (kind == ANCHOR_FLOAT_HYPOTHESE) ? GUARD_REJECT : GUARD_WEAR_SUSPECT;
    }

    // ------------------------------------------------------------
    // Segments (privés)
    // ------------------------------------------------------------
private:
    /**
     * @brief Ouvre un nouveau segment à partir du SoC courant.
     * @param SoC_start    SoC de l'ancre d'ouverture (%)
     * @param anchor_start Nature de cette ancre (v2.0.0)
     */
    void startNewSegment(float SoC_start, uint8_t anchor_start = ANCHOR_NONE) {
        seg.active       = true;
        seg.anchor_start = anchor_start;
        seg.anchor_end   = ANCHOR_NONE;
        seg.Uc_start     = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
        seg.SoC_start    = SoC_start;
        seg.n            = 0;
        seg.dV_ema       = 0.0f;
        seg.dI_ema       = 0.0f;
        seg.T_start      = temp_c_last;
        seg.T_last       = temp_c_last;
    }

    /**
     * @brief Ferme un segment rejeté et comptabilise le rejet.
     * @param SoC_end      SoC de l'ancre de fermeture (nouvelle ouverture)
     * @param anchor_end   Nature de cette ancre
     * @param quality      Score de qualité [0;1] (0 si non calculé)
     * @param reason       Motif (SegReason, 0 = OK)
     */
    void rejectSegment(float SoC_end, uint8_t anchor_end, float quality, uint8_t reason,
                       float dsoc_pct = 0.0f) {
        if (_segRejected < 0xFFFF) _segRejected++;
        _segQ.score = quality;
        _segReason  = reason;
        _lastAnchor = anchor_end;
        _lastDSoC   = dsoc_pct;
        startNewSegment(SoC_end, anchor_end);
    }

    void updateSegmentAndKalman(float delta_cycles, float V = 0.0f, float I = 0.0f) {
        if (!seg.active) {
            if (mppt_state == State_FLOAT) {
                startNewSegment(data->SoC_fused, ANCHOR_FLOAT_HYPOTHESE);
            } else if (mppt_state == State_REST_LONG) {
                startNewSegment(data->SoC_fused, ANCHOR_OCV_MESURE);
            }
            return;
        }

        seg.n++;

        // v2.0.0 : bruit du capteur sur la durée du segment. |ΔV| et |ΔI| entre
        // deux échantillons consécutifs — c'est le bruit, pas l'amplitude du
        // signal (la tension parcourt ~1 V sur une charge complète : son écart
        // crête ne dit rien de sa propreté).
        //
        // ⚠️ REJET D'ÉCHELON. Un changement d'état (DISCHARGE → REST → BULK)
        //    fait sauter le courant de plusieurs ampères en UN échantillon.
        //    Ce n'est pas du bruit, c'est le procédé. Sans ce garde-fou, la
        //    moyenne exponentielle absorbait le saut et déclarait le segment
        //    « courant bruité » à chaque transition — donc rejetait des
        //    segments parfaitement valides. On n'alimente l'estimateur que si
        //    l'écart reste dans l'ordre de grandeur du bruit déjà observé.
        if (V > 0.0f && V_prev_sample > 0.0f) {
            float dV  = fabsf(V - V_prev_sample);
            float lim = 4.0f * seg.dV_ema + 0.002f;      // 2 mV de plancher
            if (seg.n <= 1 || dV <= lim) {
                seg.dV_ema = (seg.n <= 1) ? dV : (0.95f * seg.dV_ema + 0.05f * dV);
            }
        }
        {
            float dI  = fabsf(I - I_prev_sample);
            float lim = 4.0f * seg.dI_ema + 0.010f;      // 10 mA de plancher
            if (seg.n <= 1 || dI <= lim) {
                seg.dI_ema = (seg.n <= 1) ? dI : (0.95f * seg.dI_ema + 0.05f * dI);
            }
        }
        seg.T_last = temp_c_last;

        float SoC_end = 0.0f;
        bool close = false;
        bool pseudo_close = false;   // P1: fermeture sur fin de pseudo-repos
        uint8_t anchor_end = ANCHOR_NONE;

        // F1: transitions détectées (plus de code mort)
        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            // v2.0.0 : HYPOTHÈSE, plus une mesure. La nature est mémorisée pour
            // que l'incertitude de l'ancre entre dans R (cf. anchorCapacityVariance).
            SoC_end = 100.0f; close = true;
            anchor_end = ANCHOR_FLOAT_HYPOTHESE;
        } else if (mppt_state == State_REST_LONG && mppt_state_prev != State_REST_LONG) {
            // F5 + F8: fermeture par transition REST_LONG (porte de stabilité)
            SoC_end = data->SoC_voltage;
            close = true;
            anchor_end = ANCHOR_OCV_MESURE;
        } else if (pseudo_rest_ended) {
            // P1: fin d'un pseudo-repos -> ancre OCV basse (repos court).
            // SoC estimé depuis la tension relaxée (dernier échantillon du repos).
            SoC_end = model->ocvToSoc(pseudo_rest_v, 25.0f);
            close = true;
            pseudo_close = true;
            anchor_end = ANCHOR_PSEUDO_REST;
        }

        if (!close) return;

        // v1.6.0: gating OCV — segments désactivés pour NiFe/Sodium
        // (OCV peu fiable) sauf si overridé via setSegmentThresholds()
        if (!getTuning().ocv_segment_allowed) {
            startNewSegment(SoC_end, anchor_end);
            return;
        }

        // v1.9.0 : différence en µC ENTIERS puis conversion unique.
        // Soustraire deux `getAhNet()` en float32 aurait perdu le delta à
        // 910 Ah (ULP 6,1e-05 Ah vs incrément 2,78e-06 Ah).
        int64_t now_uc = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
        float dAh = fabsf((float)(now_uc - seg.Uc_start) / 3.6e9f);
        float dSoC = fabsf(SoC_end - seg.SoC_start);
        float signed_ah = (float)(now_uc - seg.Uc_start) / 3.6e9f;

        // P1: un pseudo-repos n'est une ancre BASSE valable qu'en DÉCHARGE NETTE
        // (creux après la nuit). Un pseudo-repos pendant la CHARGE (tension haute,
        // net positif) doit être IGNORÉ sans casser le segment en cours.
        if (pseudo_close) {
            if (signed_ah >= 0.0f) return;                 // net charge : ignorer, segment conservé
            if (dAh < pseudo_rest_min_ah) { startNewSegment(SoC_end, anchor_end); return; }
        }

        const KalmanTuning& t2 = getTuning();
        float min_dSoC = t2.seg_min_dsoc[1];
        if (phase == PHASE_BOOTSTRAP) min_dSoC = t2.seg_min_dsoc[0];
        if (phase == PHASE_REFINE || phase == PHASE_TRACK) min_dSoC = t2.seg_min_dsoc[2];

        // v1.6.0: seuil relatif (% de C) pour petites batteries (NiMH/Alkaline)
        float dAh_threshold = t2.seg_min_dAh_abs;
        float C_ref = getEffectiveCapacity();
        if (t2.seg_min_dAh_pct > 0.0f && C_ref > 0.0f) {
            dAh_threshold = max(dAh_threshold, t2.seg_min_dAh_pct * C_ref / 100.0f);
        }

        if (dAh < dAh_threshold || dSoC < min_dSoC || seg.n == 0) {
            rejectSegment(SoC_end, anchor_end, 0.0f, SEG_REASON_DSOC, dSoC);
            return;
        }

        float C_measured = dAh / (dSoC / 100.0f);

        if (!isfinite(C_measured) || C_measured < 1.0f || C_measured > 2000.0f) {
            rejectSegment(SoC_end, anchor_end, 0.0f, SEG_REASON_NONFINITE, dSoC);
            return;
        }

        // ---- v2.0.0 : score de qualité (produit de 5 facteurs) ----
        // Facteurs : résolution de l'ancre, amplitude du segment, bruit de
        // tension, dérive thermique, bruit de courant. Le produit est un
        // REJET (un facteur nul annule tout), pas une pondération — la
        // pondération, c'est `R`, calculé plus bas. Deux rôles distincts,
        // aucune duplication : `R` dit « peu informatif », le score dit
        // « inutilisable ».
        _segQ = SegmentValidator::evaluate(
            dSoC, anchorSoCUncertainty(anchor_end),
            seg.dV_ema * 1000.0f,
            fabsf(seg.T_last - seg.T_start),
            seg.dI_ema,
            _segLimits);
        _segReason = _segQ.reason;
        if (!_segQ.accepted) {
            rejectSegment(SoC_end, anchor_end, _segQ.score, _segQ.reason, dSoC);
            return;
        }

        // ---- v2.0.0 : garde croisée avec la capacité indépendante ----
        uint8_t guard = crossGuard(C_measured, _capacityPrior, _crossGuardRel, anchor_end);
        if (guard == GUARD_REJECT) {
            // ⚠️ v5.31 (F3) : motif DÉDIÉ. Passer `_segQ.reason` ici envoyait
            // `SEG_REASON_OK` (le score venait d'être accepté) → un rejet sans
            // cause affichable.
            rejectSegment(SoC_end, anchor_end, _segQ.score,
                          SEG_REASON_CROSS_GUARD, dSoC);
            return;
        }
        // ⚠️ v5.31 (F4) : le drapeau doit pouvoir RETOMBER. Il ne servait à rien
        // s'il restait levé pour tout l'uptime : il conditionne la confiance
        // accordée à `C_hat` par rapport au modèle d'usure.
        _wearSuspect = (guard == GUARD_WEAR_SUSPECT);

        // P2: accumuler le point (dSoC, dAh) pour la régression robuste (passif).
        // UNIQUEMENT les segments de DÉCHARGE NETTE : les segments de charge sont
        // biaisés par le rendement (Ah charge > Ah stockés) et par le maintien
        // FLOAT (Ah accumulés sans variation de SoC).
        if (signed_ah < 0.0f) addRegressionPoint(dSoC, dAh);

        // ---- v2.0.0 : R = max(R_adaptatif, R_ancre) ----
        // UNE loi pour toutes les natures d'ancre (l'ancien traitement spécial
        // du pseudo-repos — R×4 + formule ad hoc — disparaît). La variance
        // apportée par l'incertitude de l'ancre est celle de la propagation
        // standard : σ_C = C_ref·u_ancre/dSoC, R = σ_C².
        float R = kalman->R_measured;
        float R_anchor = anchorCapacityVariance(C_ref, dSoC, anchor_end);
        if (isfinite(R_anchor) && R_anchor > R) R = R_anchor;

        // v2.0.0 (W3) — INITIALISATION SUR HYPOTHÈSE.
        // On AUTORISE l'init même sur une ancre hypothétique : la refuser
        // laisserait `C_hat` à 0, donc `C_effective` à 0, donc l'intégration du
        // courant SAUTÉE et `detectChargeState()` en repli à 100 Ah — c'est-à-
        // dire le bug §M de la documentation projet (FLOAT détecté jusqu'à 2 A,
        // SoC forcé à 100 %). On la marque simplement : `P[0][0]` reçoit la
        // variance d'ancre, si bien que le PREMIER segment ancré sur une mesure
        // reçoit un gain quasi unitaire et corrige en une seule fois.
        if (!kalman->initialized && anchor_end == ANCHOR_FLOAT_HYPOTHESE) {
            _initFromHypothesis = true;
        }
        // ⚠️ v5.31 (F4) : une ancre MESURÉE lève le doute. Sans cette remise à
        // zéro, `c_from_hyp` restait `true` pour tout l'uptime dès la première
        // initialisation sur FLOAT, alors que le champ signifie précisément
        // « C_hat n'a JAMAIS été confronté à une mesure ».
        if (anchor_end == ANCHOR_OCV_MESURE || anchor_end == ANCHOR_PSEUDO_REST) {
            _initFromHypothesis = false;
        }

        // EKF 2D update
        applyKalmanUpdate2D(C_measured, R, delta_cycles);
        cycles_since_update = 0.0f;   // K7: cycles consommés par la mesure
        _lastAnchor = anchor_end;
        _lastDSoC   = dSoC;
        if (_segAccepted < 0xFFFF) _segAccepted++;
        startNewSegment(SoC_end, anchor_end);
    }

public:
    // ============================================================
    // CONSTRUCTEUR
    // ============================================================
    BatteryKalman(SoCData* soc, KalmanState2D* kstate, BatteryModel* battModel, Ah_GD* ah)
        : data(soc), kalman(kstate), model(battModel), ahMeter(ah) {
        _segLimits = SegmentValidator::defaultLimits();
    }

    // ============================================================
    // INITIALISATION
    // ============================================================
    void begin() {
        loadState();

        if (!data->coulomb_initialized) {
            data->SoC_coulomb = data->SoC_voltage;
            data->SoC_fused = data->SoC_voltage;
            data->coulomb_initialized = true;
            data->last_sync_time = millis();
            // v1.9.0 : le compteur est fourni par l'appelant, déjà initialisé.
            last_Uc = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
        }

        // K8 CORRECTION (toujours valable en v1.9.0) : synchroniser TOUJOURS
        // `last_Uc` sur le compteur courant, y compris en restauration. Sinon,
        // si le compteur est restauré à une valeur non nulle (persistance),
        // last_Uc resterait à 0 et le 1er update() injecterait tout le compteur
        // comme incrément (dUc = net − 0) → saut de SoC.
        last_Uc = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;

        updatePhaseFromConfidence();
        state_entry_ms = millis();
        last_cycles = data->cycles_partial;
        // F1: garantir un état "précédent" cohérent au démarrage
        mppt_state_prev = mppt_state;
        mppt_raw_prev = mppt_state;
    }

    // ============================================================
    // MISE À JOUR PRINCIPALE
    // ============================================================
    /**
     * @param dt_s Pas de temps RÉEL écoulé depuis l'appel précédent (s).
     *             v1.9.0 : il est FOURNI par l'appelant au lieu d'être lu sur le
     *             compteur. `Coulomb::getLastInterval()` n'était renseigné que
     *             par `addMeasurement()` ; un appelant utilisant
     *             `addMeasurementWithInterval()` laissait 0,001 s → toutes les
     *             constantes de temps du modèle étaient 100× trop grandes
     *             (τ_pol 12 s → 1200 s, τ2 300 s → 8,3 h, τ_hyst 2000 s → 55 h).
     */
    void update(float V, float I, float T, float dt_s = 0.1f) {
        temp_c_last = T;
        if (!isfinite(dt_s) || dt_s <= 0.0f) dt_s = 0.1f;
        float dt_h = dt_s / 3600.0f;

        updateDerivatives(V, I, dt_s);

        float r_int_eff;
        // Référence = SoC COULOMB (factuel) pour scaler R_soc/R_int — évite
        // l'emballement de la correction OCV sous charge (cf. BatteryModels).
        float V_cell_ocv = model->correctVoltageToOCV(V, I, T, dt_s, &r_int_eff, nullptr,
                                                      data->SoC_coulomb)
                          / model->getCellCount();
        R_int_eff = r_int_eff;

        data->SoC_voltage = model->ocvToSoc(V_cell_ocv * model->getCellCount(), T);

        updateMpptState(V, I);
        updatePseudoRest(V, I);   // P1: ancre OCV basse (repos court), opt-in

        // v1.9.0 : delta en µC ENTIERS puis conversion unique (exact).
        int64_t now_uc = ahMeter ? ahMeter->getNetMicroCoulombs() : last_Uc;
        float dAh = (float)(now_uc - last_Uc) / 3.6e9f;
        last_Uc = now_uc;

        // v1.7.3 (F8) : les compteurs Ah publiés doivent se réconcilier avec le
        // SoC. Avant, `Ah_total_discharged` intégrait I brut et
        // `Ah_total_charged` un 0,98 codé en dur, alors que `SoC_coulomb` utilise
        // le dAh CORRIGÉ (η) du compteur → à 5 A les deux divergeaient de 1,7-1,9×.
        // On dérive désormais les deux compteurs de la MÊME source (dAh).
        if (dAh < 0.0f)
            data->Ah_total_discharged += -dAh;
        else
            data->Ah_total_charged += dAh;
        // ⚠️ v2.1.0 (F-03) — `getSoCCapacity()` au lieu de `getEffectiveCapacity()` :
        // la conversion Ah→SoC tient compte du RÉGIME (Peukert). Sans cela, à
        // 0,56 A de décharge nocturne, le SoC descendait 40 % trop vite et
        // l'ancre FLOAT le remettait à 100 % chaque matin — c'est le désaccord
        // « +29 Ah entrés / SoC qui baisse » observé sur 8 jours.
        // Le compteur de coulombs (dAh) reste BRUT : seule la conversion change.
        float C_ref = getSoCCapacity(I, dt_s);
        if (C_ref > 0.0f) {
            data->SoC_coulomb += (dAh / C_ref) * 100.0f;
            data->SoC_coulomb = constrain(data->SoC_coulomb, 0.0f, 100.0f);
        }


        // F6: apprentissage Kalman TOUJOURS actif (indépendant de isAutoDetect)
        {
            // K7: accumuler les cycles écoulés depuis la dernière mesure Kalman.
            // Sans accumulation, le modèle de processus vieillissement est
            // appliqué avec Δcycles≈0 à la mesure et dC_dCycle n'est jamais appris.
            float delta_cycles = data->cycles_partial - last_cycles;
            last_cycles = data->cycles_partial;
            cycles_since_update += delta_cycles;
            updateSegmentAndKalman(cycles_since_update, V, I);
        }

        handleSyncEvents(V_cell_ocv, T);

        data->SoC_fused = fuseSoC();
        data->SoC_fused = constrain(data->SoC_fused, 0.0f, 100.0f);
        data->SoC_uncertainty = computeUncertainty();

        updateCycleDetection(I);

        // F1: mppt_state_prev mis à jour en fin de cycle pour la prochaine itération
        mppt_state_prev = mppt_state;
        V_prev_sample = V;   // R6: mémorise V pour l'estimation du bruit de mesure
        I_prev_sample = I;   // v2.0.0: idem pour le courant
    }

    // ============================================================
    // DÉTECTION DE CYCLES
    // ============================================================
    void updateCycleDetection(float I) {
        if (!cycle_initialized) {
            cycle.cycle_start_time = millis();
            cycle_initialized = true;
        }

        cycle.SoC_max = max(cycle.SoC_max, data->SoC_fused);
        cycle.SoC_min = min(cycle.SoC_min, data->SoC_fused);
        if (I > 0.05f) cycle.was_charging = true;
        if (I < -0.05f) cycle.was_discharging = true;

        // F1: transition FLOAT désormais détectée -> comptage de cycles fonctionnel
        if (cycle.was_discharging && mppt_state == State_FLOAT
            && mppt_state_prev != State_FLOAT && cycle.SoC_min < 85.0f) {
            float DoD = cycle.SoC_max - cycle.SoC_min;
            data->cycles_partial += DoD / 100.0f;
            data->DoD_accumulated += DoD;
            if (DoD > 80.0f) data->cycles_full++;
            cycle = CycleHistory();
            cycle.cycle_start_time = millis();
        }

        if (millis() - cycle.cycle_start_time > 604800000UL) {
            cycle = CycleHistory();
            cycle.cycle_start_time = millis();
        }
    }

    // ============================================================
    // SYNCHRONISATIONS MANUELLES
    // ============================================================
    void syncToVoltage() {
        doSync(data->SoC_voltage, 0.80f, "Manual sync", ANCHOR_OCV_MESURE);
    }

    void syncToFull() {
        doSync(100.0f, 0.95f, "Manual full", ANCHOR_FLOAT_HYPOTHESE);
    }

    void syncToEmpty() {
        doSync(0.0f, 0.90f, "Manual empty", ANCHOR_OCV_MESURE);
    }

    void resetCoulombCount(float new_SoC = -1.0f) {
        if (new_SoC < 0.0f) new_SoC = data->SoC_voltage;
        data->SoC_coulomb = new_SoC;
        data->SoC_fused = new_SoC;
        last_Uc = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
        data->last_sync_time = millis();
    }

    void resetLearning() {
        const KalmanTuning& t = getTuning();
        kalman->C_hat = 0.0f;
        kalman->dC_dCycle = 0.0005f;
        kalman->P[0][0] = t.p_init_C;
        kalman->P[0][1] = 0.0f;
        kalman->P[1][0] = 0.0f;
        kalman->P[1][1] = t.p_init_aging;
        kalman->n_updates = 0;
        kalman->initialized = false;
        kalman->confidence = 0.0f;
        kalman->R_estimated = t.r_init;
        kalman->R_measured = t.r_init;
        phase = PHASE_BOOTSTRAP;
        // ⚠️ v5.31 (F8) — L'ÉTAT DE SEGMENT DOIT MOURIR AVEC L'APPRENTISSAGE.
        // Avant, `resetLearning()` remettait `C_hat`/`P`/`phase` à zéro mais
        // laissait `seg.active`, `seg.SoC_start` et `seg.Uc_start` vivants. Le
        // segment suivant se fermait alors avec un dSoC et un dAh mesurés
        // depuis l'ancre PRÉ-RESET, et comme `P[0][0]` venait d'être réinitialisé
        // à `p_init_C`, le gain était quasi unitaire : un `C_hat` faux était
        // injecté juste après un reset. Les compteurs exposés restaient ceux
        // d'avant, rendant le reset invérifiable.
        seg.active       = false;
        seg.anchor_start = ANCHOR_NONE;
        seg.anchor_end   = ANCHOR_NONE;
        seg.Uc_start     = ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
        seg.SoC_start    = 0.0f;
        seg.n            = 0;
        seg.dV_ema       = 0.0f;
        seg.dI_ema       = 0.0f;
        seg.T_start      = temp_c_last;
        seg.T_last       = temp_c_last;
        _segQ            = SegmentQuality();
        _segReason       = SEG_REASON_OK;
        _segAccepted     = 0;
        _segRejected     = 0;
        _lastAnchor      = ANCHOR_NONE;
        _lastDSoC        = 0.0f;
        _initFromHypothesis = false;   // v5.31 (F4)
        _wearSuspect        = false;   // v5.31 (F4)
        state_dirty = true;
    }

    // ============================================================
    // GETTERS
    // ============================================================
    float getSoC() { return data->SoC_fused; }
    float getSoCCoulomb() { return data->SoC_coulomb; }
    float getSoCVoltage() { return data->SoC_voltage; }
    float getSoCRaw() { return data->SoC_voltage_raw; }
    bool  isSoCRaw() { return data->soc_is_raw; }
    bool  isSoCKnown() { return phase != PHASE_BOOTSTRAP; }

    float getEffectiveCapacity() const {
        if (kalman->initialized && kalman->C_hat > 1.0f) return kalman->C_hat;
        // v2.0.0 — REPLI SUR LA CAPACITÉ INDÉPENDANTE.
        // Depuis que les segments ancrés sur une hypothèse peuvent être rejetés,
        // `C_hat` peut légitimement rester à 0 (aucune ancre mesurée n'a encore
        // été rencontrée). Renvoyer 0 déclencherait la cascade §M documentée :
        // intégration du courant SAUTÉE, `detectChargeState()` en repli à 100 Ah,
        // FLOAT détecté jusqu'à 2 A, SoC forcé à 100 %. La capacité issue du
        // modèle d'usure, bien qu'imparfaite, est une référence infiniment plus
        // saine que zéro.
        if (_capacityPrior > 1.0f) return _capacityPrior;
        if (model->getNominalCapacity() > 0.0f) return model->getNominalCapacity();
        return 0.0f;
    }

    /**
     * @brief Active la loi de Peukert sur la CONVERSION Ah→SoC (v2.1.0, F-03).
     *
     * @param n       exposant de Peukert (> 1 = actif ; ≤ 1 désactive)
     * @param iref_A  courant de référence (A) ≈ C/20
     * @param tau_s   constante de temps du lissage du courant (s)
     *
     * ⚠️ N'affecte QUE le diviseur `dAh/C`. Le compteur de coulombs reste
     * BRUT et `η` reste la seule pondération du courant (cf. CoulombFct).
     */
    void setSoCPeukert(float n, float iref_A, float tau_s = 300.0f) {
        if (!isfinite(n) || n < 1.0f || n > 2.0f) { _peukertOn = false; return; }
        if (!isfinite(iref_A) || iref_A <= 0.0f) { _peukertOn = false; return; }
        _peukertN = n;
        _peukertIref = iref_A;
        // ⚠️ > 0.0f et non > 1.0f : un tau de 1,0 s (tests hote) etait REJETE
        // silencieusement, la valeur restant a 300 s -> EMA 300x trop lente.
        if (isfinite(tau_s) && tau_s > 0.0f) _peukertTau = tau_s;
        _peukertOn = (n > 1.0f);
        // N17 — on ne ré-amorce PAS le régime depuis le courant instantané : à
        // l'application d'une configuration, ce courant est souvent une CHARGE,
        // qui polluerait le régime pour tout le cycle. Le régime sera ré-amorcé
        // à la première décharge réelle (front montant).
        _peukertIema     = 0.0f;
        _peukertIemaInit = false;
        _peukertWasDis   = false;
        // N22 — `_peukertRatioInit` n'est remis à zéro QUE si l'on désactive :
        // sinon, ré-appliquer la configuration (clic « appliquer ») faisait
        // sauter le ratio publié de 0,58 à 1,47 en un échantillon, soit 11,5 Ah
        // d'un coup sur `Ah`, `aut_h` et `c_soc`. Au boot le drapeau est déjà
        // faux (constructeur), donc l'amorçage initial reste inchangé.
        if (!_peukertOn) {
            _peukertCRatio    = 1.0f;
            _peukertRatioInit = false;
        }
    }
    bool  isSoCPeukertOn() const { return _peukertOn; }
    float getSoCPeukertRatio() const { return _peukertCRatio; }

    /**
     * @brief Capacité de conversion au régime COURANT, sans muter l'état.
     *
     *     C_soc = getEffectiveCapacity() × _peukertCRatio
     *
     * ⚠️ v5.39 (A1) — POURQUOI CET ACCESSEUR EXISTE.
     * `getSoCCapacity()` fait avancer l'EMA de courant : l'appeler deux fois par
     * cycle (une pour le compteur coulomb, une pour l'EKF) diviserait sa
     * constante de temps par deux. Cet accesseur est donc **const** : il relit
     * le dernier ratio calculé et ne touche à rien.
     *
     * Il sert à ce que les DEUX estimateurs de SoC du firmware divisent les Ah
     * par la MÊME capacité. Sans lui, l'EKF recevait `getEffectiveCapacity()`
     * (C_hat, sans ratio) alors que le compteur divisait par C_hat×ratio : à
     * 0,56 A les deux intégrateurs consommaient 26,07 Ah contre 36,5 Ah, soit
     * 1,10 pt de SoC par Ah déchargé d'écart cumulé.
     */
    float getSoCCapacityAtCurrentRegime() const {
        float C = getEffectiveCapacity();
        if (C <= 0.0f) return C;
        float r = _peukertCRatio;
        if (!isfinite(r) || r <= 0.0f) r = 1.0f;
        return C * r;
    }

    /**
     * @brief Capacité utilisée pour convertir les Ah en SoC, à ce régime.
     *
     *     ratio_cible = (I_ref / |I|_régime)^(n−1)     borné ×2, DÉCHARGE
     *     ratio       = EMA(ratio_cible)               τ = 60 s
     *     C_soc       = C_ref · ratio
     *
     * DEUX grandeurs seulement :
     *   • le RÉGIME — `|I|` lissé (τ = `_peukertTau`, 300 s) **pendant la
     *     décharge uniquement**, ré-amorcé sur le front montant. Une décharge
     *     interrompue puis reprise repart donc du courant RÉEL, pas de l'ancien.
     *   • le RATIO PUBLIÉ — EMA de la cible (τ = 60 s). C'est ce qui garantit
     *     qu'aucun échantillon ne porte de saut (`Ah`, `aut_h`, `peuk_ratio`,
     *     `c_soc`).
     * Hors décharge (charge, repos, courant sous le plancher d'identifiabilité)
     * la cible vaut 1,0 : c'est `η` qui porte les pertes en charge, et une
     * décharge de quelques dizaines de mA ne relève pas de la loi de Peukert.
     * L'état « en décharge » a une HYSTÉRÉSIS (entrée `−I_ACTIVE`, sortie
     * `+I_NOISE` ou régime effondré) : une porte instantanée faisait converger
     * le ratio vers la moyenne de deux cibles, valeur qu'aucun régime ne produit.
     *
     * ⚠️ COÛT DU LISSAGE, mesuré : un pic de 0,5 → 5 A de 180 s laisse le SoC
     * 0,79 pt trop haut (dont ~0,18 pt imputables au seul lissage du ratio,
     * compensé à −0,04 pt au retour ⇒ ≈ 0,14 pt de dérive nette par cycle).
     * C'est le prix de la continuité : sans elle, « Ah disponibles » sautait de
     * 19,1 à 7,6 Ah en 100 ms. τ = 60 s est le compromis retenu.
     *
     * @param I courant (A), signé — la correction ne s'applique qu'en décharge
     * @param dt_s intervalle (s) — pour un lissage indépendant de la cadence
     */
    float getSoCCapacity(float I, float dt_s = 0.1f) {
        float C = getEffectiveCapacity();
        if (C <= 0.0f) return C;
        if (!_peukertOn || !(_peukertN > 1.0f)) { _peukertCRatio = 1.0f; return C; }

        // N17 — sur entrée NON FINIE on ne touche à RIEN et on renvoie
        // `C × ratio` : les deux consommateurs (compteur et accesseur `const`)
        // restent donc alignés ET la valeur publiée reste continue. Remettre le
        // ratio à 1 puis le ré-amorcer produisait DEUX discontinuités publiées
        // (−29 % puis +40 %) pour un simple glitch I²C.
        if (!isfinite(I)) return C * _peukertCRatio;

        const float RATIO_MAX = 2.0f;    // plafond physique (idem AhFct)
        const float dt = (isfinite(dt_s) && dt_s > 0.0f) ? dt_s : 0.1f;

        // ── 1. État « en décharge » — HYSTÉRÉSIS, pas porte binaire ────────
        // ⚠️ v5.41 (N21) — POURQUOI UN ÉTAT ET NON UN TEST PAR ÉCHANTILLON.
        // Avec une porte instantanée, un courant qui oscille autour du seuil
        // (0,151 / 0,149 A) fait alterner la cible entre 1,0 et 2,0 : l'EMA du
        // ratio converge alors vers leur MOYENNE (1,50), valeur qu'AUCUN régime
        // ne produit. Mesure : C_soc 39,12 Ah au lieu de 52,14 Ah (−25 %), SoC
        // 33 % trop rapide, +2,3 pt de SoC par 24 h. Le ratio restait parfaite-
        // ment lisse : défaut d'EXACTITUDE, pas de continuité.
        // Deux seuils distincts :
        //   • ENTRÉE  : `I < −I_ACTIVE`  → une décharge réelle commence ;
        //   • SORTIE  : `I > −I_NOISE`   → repos ou charge (le courant cesse
        //                                    d'être franchement négatif) ;
        //   • SORTIE  : `_peukertIema < 0,8 × I_ACTIVE` → le régime s'est effondré.
        const float aI = fabsf(I);
        if (!_peukertWasDis) {
            if (I < -_peukertIActive) {
                _peukertWasDis = true;                 // front montant
                _peukertIema     = aI;                 // ré-amorçage (N15)
                _peukertIemaInit = true;
            }
        } else if (I > -_peukertINoise || _peukertIema < 0.8f * _peukertIActive) {
            // Sortie dès que le courant n'est plus franchement négatif (repos ou
            // charge), ou que le régime s'est effondré. ⚠️ `I > -I_NOISE` et non
            // `I > +I_NOISE` : au repos le courant vaut ≈ 0, il doit faire SORTIR
            // de l'état (sinon l'amplitude continuait de se dégrader pendant
            // l'inactivité et le régime de reprise restait pollué).
            _peukertWasDis = false;
        }

        // ── 2. Régime : |I| lissé pendant l'état « en décharge » ───────────
        float target = 1.0f;
        if (_peukertWasDis) {
            if (!_peukertIemaInit) { _peukertIema = aI; _peukertIemaInit = true; }
            else {
                float aMag = 1.0f - expf(-dt / _peukertTau);
                if (!isfinite(aMag) || aMag <= 0.0f) aMag = 1e-4f;
                if (aMag > 1.0f) aMag = 1.0f;
                _peukertIema += aMag * (aI - _peukertIema);
            }
            // ⚠️ v5.39 (A2) — Peukert est une loi de DÉCHARGE (Wikipedia
            // « Peukert's law », « Lead–acid battery »). En charge, c'est le
            // rendement η (CoulombFct) qui porte les pertes : un ratio < 1 en
            // plus (0,747 à +2,7 A) gonflait la pente du SoC de 34 %.
            float r = powf(_peukertIref / _peukertIema, _peukertN - 1.0f);
            if (isfinite(r) && r > 0.0f) target = (r > RATIO_MAX) ? RATIO_MAX : r;
        }

        // ── 3. Ratio PUBLIÉ : toujours lissé, donc toujours continu ────────
        float aRat = 1.0f - expf(-dt / PEUKERT_RATIO_TAU_S);
        if (!isfinite(aRat) || aRat <= 0.0f) aRat = 1e-4f;
        if (aRat > 1.0f) aRat = 1.0f;
        if (!_peukertRatioInit) {
            _peukertCRatio    = target;
            _peukertRatioInit = true;
        } else {
            _peukertCRatio += aRat * (target - _peukertCRatio);
        }

        if (!isfinite(_peukertCRatio) || _peukertCRatio <= 0.0f) _peukertCRatio = 1.0f;
        return C * _peukertCRatio;
    }

    /**
     * @brief Règle les seuils du régime de décharge (A).
     * @param i_active_A en dessous, pas de correction Peukert (loi non identifiable)
     * @param i_noise_A  seuil de bruit de mesure (INA226)
     */
    void setSoCPeukertThresholds(float i_active_A, float i_noise_A) {
        if (isfinite(i_active_A) && i_active_A > 0.0f) _peukertIActive = i_active_A;
        if (isfinite(i_noise_A)  && i_noise_A  > 0.0f) _peukertINoise  = i_noise_A;
    }

    /**
     * @brief Origine de la capacité effectivement utilisée (diagnostic).
     * @return 2 = apprise (C_hat), 1 = indépendante (usure), 0 = saisie, -1 = aucune
     *
     * Sert à distinguer « capacité apprise » de « capacité supposée » dans l'UI :
     * les deux alimentent le même calcul de SoC, mais n'ont pas la même valeur.
     */
    int8_t getCapacitySource() {
        if (kalman->initialized && kalman->C_hat > 1.0f) return 2;
        if (_capacityPrior > 1.0f) return 1;
        if (model->getNominalCapacity() > 0.0f) return 0;
        return -1;
    }

    float getAhAvailable() {
        // v5.39 (A4) — la capacité au RÉGIME COURANT, pas la capacité brute :
        // le SoC affiché a été calculé en divisant par C_eff×ratio, donc
        // SoC×C_eff sous-estimait l'énergie réellement restituable (à 0,56 A et
        // SoC 50 % : 13,0 Ah annoncés contre 18,3 Ah disponibles).
        float C = getEffectiveCapacity();
        if (C <= 0.0f || phase == PHASE_BOOTSTRAP) return -1.0f;
        return (data->SoC_fused / 100.0f) * getSoCCapacityAtCurrentRegime();
    }

    float getAhIntegrated() {
        return ahMeter ? ahMeter->getAhNet() : 0.0f;
    }

    /** @return Total NET intégré en micro-coulombs (int64, EXACT) — v1.9.0 */
    int64_t getMicroCoulombsIntegrated() {
        return ahMeter ? ahMeter->getNetMicroCoulombs() : 0;
    }

    /**
     * @brief Accès au compteur Coulomb (v1.7.3) — permet à l'UI d'afficher le
     * rendement η RÉELLEMENT appliqué à l'intégration, au lieu de recalculer la
     * loi depuis `BatteryModel` (qui diverge : η constant côté compteur vs
     * η(I) de Peukert côté modèle).
     */
    Ah_GD* getCoulombMeter() { return ahMeter; }   // v1.9.0 : compteur fusionnel

    float getCyclesPartial() { return data->cycles_partial; }
    uint32_t getCyclesFull() { return data->cycles_full; }
    float getRintEff() { return R_int_eff * 1000.0f; }
    bool  isOcvUnreliable() { return !model->isOcvReliable(); }
    bool  isStateDirty() { return state_dirty; }
    void  clearStateDirty() { state_dirty = false; }

    LearningPhase getLearningPhase() { return phase; }
    ChargeState getChargeState() { return mppt_state; }
    const char* getChargeStateStr() {
        static const char* const CHARGE_NAMES[] = {
            "Unknown", "Discharge", "Bulk", "Absorption", "Float", "Rest", "Rest Long"
        };
        if (mppt_state < 7) return CHARGE_NAMES[mppt_state];
        return "Unknown";
    }
    const char* getLearningPhaseStr() { return PHASE_NAMES[phase]; }

    float getConfidence() { return kalman->confidence; }
    float getKalmanC() { return kalman->C_hat; }
    float getKalmanAgingRate() { return kalman->dC_dCycle; }
    float getKalmanP_CC() { return kalman->P[0][0]; }
    float getKalmanP_aging() { return kalman->P[1][1]; }
    uint16_t getKalmanUpdates() { return kalman->n_updates; }

    // ============================================================
    // v2.0.0 — QUALITÉ DES SEGMENTS (observabilité + réglage)
    // ============================================================
    /**
     * @brief Score de qualité du dernier segment évalué.
     * @return [0 ; 1] — 0 si le dernier segment a été rejeté sur un critère dur.
     *         ⚠️ Ce n'est PAS le score complet de `SegmentValidator` : la
     *         librairie ne connaît que le facteur d'ancre. L'appelant peut
     *         écraser la valeur via setSegmentQuality().
     */
    float getLastSegmentQuality() const { return _segQ.score; }

    /** @brief Détail des 5 facteurs du dernier segment évalué. */
    const SegmentQuality& getLastSegmentQualityDetail() const { return _segQ; }

    /** @brief Motif du dernier rejet (SegReject, 0 = OK). */
    uint8_t getLastSegmentReason() const { return _segReason; }

    /** @brief Nombre de segments rejetés depuis le démarrage. */
    uint16_t getSegmentRejected() const { return _segRejected; }

    /** @brief Nombre de segments acceptés depuis le démarrage. */
    uint16_t getSegmentAccepted() const { return _segAccepted; }

    /** @brief Nature de l'ancre qui a fermé le dernier segment (AnchorKind). */
    uint8_t getLastAnchorKind() const { return _lastAnchor; }

    /** @brief Nature de l'ancre qui a fermé le dernier segment, en clair. */
    const char* getLastAnchorKindStr() const { return anchorKindStr(_lastAnchor); }

    /**
     * @brief `C_hat` a-t-il été initialisé sur une ancre HYPOTHÉTIQUE ?
     *
     * Si oui, la valeur publiée repose sur la supposition « FLOAT = 100 % » et
     * n'a encore jamais été confrontée à une ancre mesurée. À afficher : c'est
     * la différence entre « capacité apprise » et « capacité supposée ».
     */
    bool wasInitFromHypothesis() const { return _initFromHypothesis; }

    /**
     * @brief Une ancre MESURÉE a-t-elle contredit la capacité indépendante ?
     *
     * Signifie que le modèle d'usure (`BatteryLifePredictor`) est faux, pas la
     * mesure. À lever manuellement après vérification (setWearSuspect(false)).
     */
    bool isWearSuspect() const { return _wearSuspect; }
    void setWearSuspect(bool s) { _wearSuspect = s; }

    /**
     * @brief Déclare la capacité INDÉPENDANTE de référence (modèle d'usure).
     *
     * Sert à deux choses, et les deux exigent une source indépendante de
     * `C_hat` — sinon la garde se contenterait de comparer `C_hat` à lui-même :
     *   1. échelle de `σ_C` dans `anchorCapacityVariance()` (via `C_ref`) ;
     *   2. garde croisée `crossGuard()`.
     *
     * @param ah        Capacité de référence (Ah) ; <= 0 désactive les deux
     * @param tolRel    Tolérance de la garde croisée (0,40 = 40 %) ;
     *                  <= 0 désactive la garde
     */
    void setCapacityPrior(float ah, float tolRel = SEG_CROSS_GUARD_REL) {
        _capacityPrior = (isfinite(ah) && ah > 0.0f) ? ah : 0.0f;
        _crossGuardRel = (isfinite(tolRel) && tolRel > 0.0f) ? tolRel : 0.0f;
    }
    float getCapacityPrior() const { return _capacityPrior; }

    /**
     * @brief La garde croisée est-elle RÉELLEMENT active ?
     *
     * ⚠️ v5.31 (F6.2) — sans ce drapeau, deux situations très différentes se
     * ressemblaient : « référence absente » (`_capacityPrior == 0`, l'appelant
     * n'a rien fourni) et « garde désactivée » (`_crossGuardRel == 0`, seuil
     * mis à zéro volontairement). Dans les deux cas `crossGuard()` renvoie
     * `GUARD_OK` sans rien comparer — donc un segment faux est accepté en
     * silence. `priorValid` rend cet état observable.
     */
    bool isPriorValid() const { return _capacityPrior > 1.0f && _crossGuardRel > 0.0f; }

    /** @brief Tolérance relative de la garde croisée (0 = désactivée). */
    float getCrossGuardRel() const { return _crossGuardRel; }

    /**
     * @brief Remplace les seuils de qualité des segments.
     *
     * Les défauts viennent de `SegmentValidator::defaultLimits()`. Un projet
     * dont le capteur est plus bruité peut relâcher `v_noise_max_mV` sans
     * toucher au code de la librairie.
     */
    void setSegmentLimits(const SegmentValidator::Limits& lim) { _segLimits = lim; }
    const SegmentValidator::Limits& getSegmentLimits() const { return _segLimits; }

    /** @brief Seuil de score sous lequel un segment est rejeté. */
    void setQualityMin(float q) {
        _segLimits.score_min = (isfinite(q) && q >= 0.0f) ? q : SEG_QUALITY_MIN;
    }
    float getQualityMin() const { return _segLimits.score_min; }

    /** @brief Paramètres de qualité du segment EN COURS (diagnostic). */
    float getSegmentVNoise_mV() const { return seg.dV_ema * 1000.0f; }
    float getSegmentINoise_A() const { return seg.dI_ema; }
    float getSegmentTDrift_C() const { return fabsf(seg.T_last - seg.T_start); }
    /** @brief dSoC du dernier segment FERMÉ (%), accepté ou rejeté. */
    float getSegmentDSoC() const { return _lastDSoC; }
    float getR_estimated() { return kalman->R_estimated; }
    float getR_measured() { return kalman->R_measured; }
    float getAlphaFused() { return alpha_fused; }
    // v1.8.1 : instant d'entrée dans l'état courant (ms) — permet de vérifier
    // de l'extérieur que la rampe de fusion REST s'active réellement (#49).
    uint32_t getStateEntryMs() const { return state_entry_ms; }

    const char* getPhaseInfo() { return phase_info; }
    const char* getLastSyncInfo() { return last_sync_info; }
    const char* getLastLearnInfo() { return last_learn_info; }

    // ============================================================
    // TUNING PAR TECHNOLOGIE (v1.6.0) — API publique
    // ============================================================
    // Résolution : setter runtime > model->getKalmanTuning() > macro
    // compile-time > KALMAN_DEFAULT_TUNING (v1.5.3).
    // Re-résout à chaque appel : un model->setTechnology() ultérieur est
    // automatiquement pris en compte (tant qu'aucun setter n'est actif).
    const KalmanTuning& getTuning() {
        _tuningResolve();
        return _tuningResolved;
    }

    // Désactive les setters runtime et revient au modèle/macros/défaut.
    void resetTuning() {
        _tuningOverrideSet = false;
    }

    // Remplace intégralement le tuning (tous les champs).
    void setTuning(const KalmanTuning& t) {
        _tuningOverride = t;
        _tuningOverrideSet = true;
    }

    void setP(float p_init_C, float p_min_C,
              float p_init_aging, float p_min_aging) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.p_init_C = p_init_C;
        _tuningOverride.p_min_C = p_min_C;
        _tuningOverride.p_init_aging = p_init_aging;
        _tuningOverride.p_min_aging = p_min_aging;
    }

    void setQ(float q_aging_uncertainty, float q_capacity_drift) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.q_aging_uncertainty = q_aging_uncertainty;
        _tuningOverride.q_capacity_drift = q_capacity_drift;
    }

    void setR(float r_init, float r_min, float r_max, float r_smooth_alpha) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.r_init = r_init;
        _tuningOverride.r_min = r_min;
        _tuningOverride.r_max = r_max;
        _tuningOverride.r_smooth_alpha = r_smooth_alpha;
    }

    void setSegmentThresholds(float seg_min_dAh_abs, float seg_min_dAh_pct,
                              float seg_min_dsoc0, float seg_min_dsoc1,
                              float seg_min_dsoc2, float seg_dsoc_high_conf,
                              bool ocv_segment_allowed) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.seg_min_dAh_abs = seg_min_dAh_abs;
        _tuningOverride.seg_min_dAh_pct = seg_min_dAh_pct;
        _tuningOverride.seg_min_dsoc[0] = seg_min_dsoc0;
        _tuningOverride.seg_min_dsoc[1] = seg_min_dsoc1;
        _tuningOverride.seg_min_dsoc[2] = seg_min_dsoc2;
        _tuningOverride.seg_dsoc_high_conf = seg_dsoc_high_conf;
        _tuningOverride.ocv_segment_allowed = ocv_segment_allowed;
    }

    void setBatteryChange(float thr, uint8_t count) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.battery_change_thr = thr;
        _tuningOverride.battery_change_count = count;
    }

    void setRestLong(float min_min, float stable_mv, float stable_min,
                     uint8_t min_samples) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.rest_long_min_min = min_min;
        _tuningOverride.rest_long_stable_mv = stable_mv;
        _tuningOverride.rest_long_stable_min = stable_min;
        _tuningOverride.rest_long_stable_min_samples = min_samples;
    }

    // ============================================================
    // R7: PROFIL REST_LONG PAR TECHNOLOGIE (opt-in, auto-adaptatif)
    // ============================================================
    // Applique des paramètres REST_LONG adaptés à la chimie, déduits du nom
    // renvoyé par model->getTechnologyName() (interface BatteryModel). Utilise
    // le setter runtime (niveau 1 : prime sur le modèle). À appeler après begin().
    // Le seuil crête reste en outre auto-adaptatif au bruit de mesure (R6).
    void applyRecommendedRestLong() {
        const char* n = model ? model->getTechnologyName() : nullptr;
        float   min_min = 15.0f, stable_min = 30.0f, stable_mv = 0.020f;
        uint8_t min_samples = 6;
        if (n) {
            if (strstr(n, "Plomb") || strstr(n, "AGM") || strstr(n, "Gel")) {
                // Plomb (inondé/AGM/GEL/PbC) : relaxation lente -> repos long
                min_min = 30.0f; stable_min = 60.0f; stable_mv = 0.030f;
            } else if (strstr(n, "LiFePO4") || strstr(n, "LFP")) {
                // Plateau OCV plat : repos long, seuil serré
                min_min = 30.0f; stable_min = 60.0f; stable_mv = 0.010f;
            } else if (strstr(n, "Nickel") || strstr(n, "NiCd")) {
                // NiMH/NiCd/NiZn/NiFe : relaxation rapide
                min_min = 15.0f; stable_min = 30.0f; stable_mv = 0.020f;
            } else if (strstr(n, "Sodium")) {
                // Na-ion : OCV peu fiable (segments OCV désactivés côté tuning)
                min_min = 15.0f; stable_min = 30.0f; stable_mv = 0.020f;
            } else if (strstr(n, "Supercondensateur") || strstr(n, "EDLC")) {
                // Supercondensateur : tension linéaire, relaxation quasi nulle
                min_min = 5.0f;  stable_min = 15.0f; stable_mv = 0.050f;
            } else {
                // Li-ion (Li-Ion/NMC/NCA/LCO/LiPo/LMNO/LTO) et défaut
                min_min = 15.0f; stable_min = 30.0f; stable_mv = 0.020f;
            }
        }
        setRestLong(min_min, stable_mv, stable_min, min_samples);
    }

    // ============================================================
    // P1: MODE PSEUDO-REPOS (opt-in) — ancre OCV basse sur repos court
    // ============================================================
    // Pour les installations SANS repos long (décharge permanente la nuit,
    // charge PV le jour, avec de brefs instants à courant ~nul). Active une
    // ancre basse sur ces instants : C = Ah_decharges / (100 - SoC_repos)/100,
    // combinée à l'ancre haute FLOAT (=100 %) et au comptage Ah.
    // Rétro-compatible : désactivé par défaut.
    void enablePseudoRest(float current_max_a = PSEUDO_REST_CURRENT_MAX,
                          uint8_t min_samples = PSEUDO_REST_MIN_SAMPLES,
                          float min_discharge_ah = PSEUDO_REST_MIN_AH,
                          float relax_tau_min = PSEUDO_REST_TAU_MIN,
                          uint32_t min_duration_ms = PSEUDO_REST_MIN_MS) {
        pseudo_rest_enabled = true;
        pseudo_rest_i_max = current_max_a;
        pseudo_rest_min_samples = (min_samples == 0) ? 1 : min_samples;
        pseudo_rest_min_ah = min_discharge_ah;
        pseudo_rest_tau_min = relax_tau_min;
        pseudo_rest_min_ms = min_duration_ms;   // v1.7.4 (B3) : durée mini
        pseudo_rest_count = 0;
    }
    /** v1.7.4 : durée minimale de pseudo-repos (ms) — 0 = critère par échantillons seul. */
    uint32_t getPseudoRestMinMs() { return pseudo_rest_min_ms; }
    void disablePseudoRest() {
        pseudo_rest_enabled = false;
        pseudo_rest_count = 0;
        pseudo_rest_ended = false;
    }
    bool     isPseudoRestEnabled() { return pseudo_rest_enabled; }
    uint16_t getPseudoRestEvents() { return pseudo_rest_events; }

    // ============================================================
    // P2: RÉGRESSION ACCUMULÉE (observateur passif)
    // ============================================================
    // Ajoute un point (dSoC%, dAh) à l'anneau. Un point n'est retenu que si
    // dSoC >= REG_MIN_DSOC (sinon l'erreur relative explose).
    //
    // ⚠️ v5.35 (F-23) — LE GARDE NE FILTRAIT PAS NaN.
    // `if (dSoC < REG_MIN_DSOC || dAh <= 0.0f) return;` : les DEUX comparaisons
    // sont FAUSSES quand l'opérande est NaN (IEEE 754 : tout ordre avec NaN est
    // faux), donc un point NaN était ACCEPTÉ. Il polluait ensuite
    // `getRegressionCapacity()` et `getRegressionDispersion()` pour toujours
    // (NaN + x = NaN), et était réécrit tel quel en FRAM à chaque sauvegarde.
    // C'est le MÊME motif que F-04 (`if (r.R_noise < 0.001f)` laissait passer
    // NaN) et que F-02 (`if (!f)` sur un rename non testé) : une garde écrite
    // en pensant « valeur hors plage » au lieu de « valeur invalide ».
    // → `isfinite()` d'abord, comparaisons ensuite.
    void addRegressionPoint(float dSoC, float dAh) {
        if (!isfinite(dSoC) || !isfinite(dAh)) return;
        if (dSoC < REG_MIN_DSOC || dSoC > 100.0f) return;
        if (dAh <= 0.0f || dAh > 100000.0f) return;
        reg_dsoc[reg_head] = dSoC;
        reg_dah[reg_head] = dAh;
        reg_head = (uint8_t)((reg_head + 1) % REG_MAX_POINTS);
        if (reg_count < REG_MAX_POINTS) reg_count++;
    }
    // Capacité par régression linéaire pondérée passant par l'origine.
    // Poids = dSoC² : la variance de C_i = dAh/(dSoC/100) est ∝ 1/dSoC², donc les
    // cycles PROFONDS (grand dSoC) dominent naturellement -> les décharges fortes
    // ponctuelles "recalent" l'estimation sans écraser l'historique.
    //
    // ⚠️ v5.35 (F-23) — `sxx > 0.0f` NE PROTÈGE PAS DE NaN.
    // Le garde historique `return (sxx > 0.0f) ? (sxy / sxx * 100.0f) : 0.0f;`
    // est INEFFICACE dans le cas précis qui s'est produit sur l'appareil : un
    // seul `dAh` corrompu en NaN (4 octets à 0xFF dans la FRAM) rend `sxy` NaN
    // alors que `sxx` reste FINI et POSITIF → le test passe, et la division
    // renvoie NaN. Vérifié : dSoC={10,20,30}, dAh={7,2 ; NaN ; 10,8} →
    // sxy=nan, sxx=980000, `sxx>0` vrai, résultat **nan**.
    // C'est exactement le `"reg_C":-nan` publié le 21/10/2026.
    // → on rejette tout point non fini AVANT de sommer, et on valide le résultat.
    float getRegressionCapacity() {
        if (reg_count < 2) return 0.0f;
        float sxy = 0.0f, sxx = 0.0f;
        for (uint8_t i = 0; i < reg_count; i++) {
            if (!isfinite(reg_dsoc[i]) || !isfinite(reg_dah[i])) continue;
            if (!(reg_dsoc[i] > 0.0f)) continue;
            float w = reg_dsoc[i] * reg_dsoc[i];
            sxy += w * reg_dsoc[i] * reg_dah[i];
            sxx += w * reg_dsoc[i] * reg_dsoc[i];
        }
        if (!(sxx > 0.0f) || !isfinite(sxx) || !isfinite(sxy)) return 0.0f;
        float c = sxy / sxx * 100.0f;
        return isfinite(c) ? c : 0.0f;
    }
    uint8_t getRegressionPoints() { return reg_count; }
    void    resetRegression() { reg_count = 0; reg_head = 0; }
    // dSoC maximal observé (profondeur du cycle le plus profond accumulé)
    float getRegressionMaxDoD() {
        float m = 0.0f;
        for (uint8_t i = 0; i < reg_count; i++) if (reg_dsoc[i] > m) m = reg_dsoc[i];
        return m;
    }
    // Dispersion : coefficient de variation (%) des capacités implicites C_i = dAh/(dSoC/100)
    //
    // ⚠️ v5.35 (F-23) — garde de sortie : `isfinite()` sur le RÉSULTAT.
    // Même avec un ring assaini à l'entrée, une division peut produire inf/NaN
    // (dSoC dénormalisé, dAh énorme). Un NaN publié dans `/api/kalman/status`
    // casse le `JSON.parse` du client — même symptôme que F-04. On préfère
    // déclarer « inconnue » (-1) que publier une valeur non finie.
    float getRegressionDispersion() {
        if (reg_count < 3) return -1.0f;
        float sum = 0.0f;
        for (uint8_t i = 0; i < reg_count; i++) {
            if (!(reg_dsoc[i] > 0.0f)) return -1.0f;
            sum += reg_dah[i] / (reg_dsoc[i] / 100.0f);
        }
        float mean = sum / reg_count;
        if (!(mean > 0.0f) || !isfinite(mean)) return -1.0f;
        float s2 = 0.0f;
        for (uint8_t i = 0; i < reg_count; i++) {
            float ci = reg_dah[i] / (reg_dsoc[i] / 100.0f);
            s2 += (ci - mean) * (ci - mean);
        }
        float d = sqrtf(s2 / reg_count) / mean * 100.0f;
        return isfinite(d) ? d : -1.0f;
    }

    // ============================================================
    // P2/P1: ÉTAT PERSISTABLE (ring de régression + compteur pseudo-repos)
    // ============================================================
    // La classe ne persiste pas elle-même (cf. saveState()/loadState()).
    // Ces accesseurs permettent à l'utilisateur de sauver/restaurer le ring
    // de régression P2 (observateur passif) et le compteur d'événements P1
    // dans son propre stockage (FRAM/EEPROM/LittleFS). Buffers externes de
    // taille >= REG_MAX_POINTS (cf. getRegressionMaxPoints()).
    //
    // getRegressionRing() copie les points valides et renvoie leur nombre.
    // setRegressionRing() restaure count points (count <= REG_MAX_POINTS) ;
    // la régression étant invariante par permutation des points, l'ordre
    // chronologique n'est pas nécessaire.
    uint8_t getRegressionRing(float* dSoC_out, float* dAh_out) {
        for (uint8_t i = 0; i < reg_count; i++) {
            dSoC_out[i] = reg_dsoc[i];
            dAh_out[i]  = reg_dah[i];
        }
        return reg_count;
    }
    void setRegressionRing(const float* dSoC_in, const float* dAh_in, uint8_t count) {
        if (count > REG_MAX_POINTS) count = REG_MAX_POINTS;
        for (uint8_t i = 0; i < count; i++) {
            reg_dsoc[i] = dSoC_in[i];
            reg_dah[i]  = dAh_in[i];
        }
        reg_count = count;
        reg_head  = (uint8_t)(count % REG_MAX_POINTS);
    }
    static uint8_t getRegressionMaxPoints() { return REG_MAX_POINTS; }
    void setPseudoRestEvents(uint16_t n) { pseudo_rest_events = n; }

    void setConfidence(float conf_float, float conf_rest_base, float conf_rest_span,
                       float conf_phase_coarse, float conf_phase_refine) {
        if (!_tuningOverrideSet) { _tuningOverride = getTuning(); _tuningOverrideSet = true; }
        _tuningOverride.conf_float = conf_float;
        _tuningOverride.conf_rest_base = conf_rest_base;
        _tuningOverride.conf_rest_span = conf_rest_span;
        _tuningOverride.conf_phase_coarse = conf_phase_coarse;
        _tuningOverride.conf_phase_refine = conf_phase_refine;
    }

    // ============================================================
    // PERSISTANCE
    // ============================================================
    void saveState() {
        // À adapter selon votre stockage (EEPROM, LittleFS, Preferences, etc.)
        // Persistez: SoCData + KalmanState2D (matrices)
        state_dirty = false;
    }

    void loadState() {
        // À adapter selon votre stockage
        if (!kalman->initialized && kalman->C_hat > 1.0f) {
            kalman->initialized = true;
        }
    }
};

#endif // BATTERY_KALMAN_H
