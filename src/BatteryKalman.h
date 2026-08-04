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
 * Licence: GNU General Public License v3
 * Version: 1.6.0
 * Date: Août 2026
 */

#ifndef BATTERY_KALMAN_H
#define BATTERY_KALMAN_H

#include <Arduino.h>
#include <math.h>
#include "BatteryModels.h"
#include "Coulomb.h"

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
#ifndef REST_LONG_SYNC_MS
#define REST_LONG_SYNC_MS       1800000UL   // délai entre 2 synchros REST_LONG (hors tuning)
#endif

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
    30.0f,      // rest_long_stable_min (min)
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
    float    dC_dCycle = -0.0005f;      // Taux vieillissement (Ah/cycle)

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
    Coulomb*        coulombMeter;

    ChargeState mppt_state = State_UNKNOWN;
    ChargeState mppt_state_prev = State_UNKNOWN;
    uint32_t  state_entry_ms = 0;
    // F8: suivi de stabilité de tension pour la détection REST_LONG
    float     rest_v_ring[REST_LONG_STABLE_MAX_SAMPLES] = {0.0f};
    uint32_t  rest_v_ring_ms[REST_LONG_STABLE_MAX_SAMPLES] = {0};
    uint8_t   rest_v_count = 0;
    bool      rest_stable = false;
    float     dVdt = 0.0f;
    float     V_prev = 0.0f;
    float     I_prev = 0.0f;

    LearningPhase phase = PHASE_BOOTSTRAP;

    struct Segment {
        bool     active = false;
        float    Ah_start = 0.0f;
        float    SoC_start = 0.0f;
        float    conf_start = 0.0f;
        uint32_t start_ms = 0;
        float    I_sq_sum = 0.0f;
        float    I_sum = 0.0f;
        uint32_t n = 0;
        float    I_min = 999.0f;
        float    I_max = 0.0f;
        bool     had_discharge = false;
        bool     had_charge = false;
    } seg;

    float R_int_eff = 0.0f;
    bool  state_dirty = false;
    float last_Ah = 0.0f;

    char  phase_info[80] = "Bootstrap";
    char  last_sync_info[64] = "aucune";
    char  last_learn_info[80] = "aucun";
    float alpha_fused = 0.5f;

    struct CycleHistory {
        float SoC_max = 0.0f;
        float SoC_min = 100.0f;
        bool was_charging = false;
        bool was_discharging = false;
        uint32_t cycle_start_time = 0;
    } cycle;

    float last_cycles = 0.0f;           // Pour calcul delta_cycles

    // ============================================================
    // TUNING PAR TECHNOLOGIE (v1.6.0)
    // ============================================================
    // Hiérarchie de résolution (du plus fort au plus faible) :
    //   1. override runtime (setters)          -> _tuningOverride
    //   2. model->getKalmanTuning()            -> _tuningFromModel
    //   3. macros compile-time (optionnelles)  -> KALMAN_*_MACRO
    //   4. défaut interne                      -> KALMAN_DEFAULT_TUNING
    // _tuningResolved est re-résolu à chaque getTuning() (léger) : les
    // changements de technologie du modèle sont donc pris en compte.
    KalmanTuning _tuningOverride;
    KalmanTuning _tuningFromModel;
    KalmanTuning _tuningResolved;
    bool _tuningOverrideSet = false;

    void _tuningResolve() {
        KalmanTuning t = KALMAN_DEFAULT_TUNING;
        if (_tuningOverrideSet) {
            // setter runtime (niveau 1) : remplace tout, y compris les macros
            t = _tuningOverride;
        } else if (model) {
            const KalmanTuning& mt = model->getKalmanTuning();
            t = mt;
            _tuningFromModel = mt;
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
    uint8_t _restLongMinSamples() {
        const KalmanTuning& t = getTuning();
        return t.rest_long_stable_min_samples;
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
            kalman->dC_dCycle = -0.0005f;  // Valeur par défaut
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
                kalman->dC_dCycle = -0.0005f;
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

    void updateMpptState(float V, float I) {
        float C_ref = getEffectiveCapacity();
        mppt_state = model->detectChargeState(V, I, C_ref);

        // --- F1 CORRECTION: mppt_state_prev n'est PLUS mis à jour ici ---
        // Il ne reflète plus la transition de ce même échantillon. La détection
        // de transition (FLOAT/REST_LONG) dans handleSyncEvents,
        // updateSegmentAndKalman et updateCycleDetection devient possible.
        // mppt_state_prev sera mis à jour en FIN de update().
        if (mppt_state != mppt_state_prev) {
            state_entry_ms = millis();
        }

        // --- F8: porte de stabilité pour REST -> REST_LONG ---
        // On alimente un anneau (temps, tension); REST_LONG n'est atteint
        // que si (a) repos >= rest_long_min_min (tuning), et (b) tension
        // stable sur la fenêtre rest_long_stable_min (écart crête <
        // rest_long_stable_mv). Durées config par technologie (v1.6.0).
        if (mppt_state == State_REST) {
            rest_v_ring[rest_v_count % REST_LONG_STABLE_MAX_SAMPLES] = V;
            rest_v_ring_ms[rest_v_count % REST_LONG_STABLE_MAX_SAMPLES] = millis();
            if (rest_v_count < 255) rest_v_count++;
            if (rest_v_count > REST_LONG_STABLE_MAX_SAMPLES) rest_v_count = REST_LONG_STABLE_MAX_SAMPLES;

            // Conserver uniquement les échantillons dans la fenêtre temporelle
            // et calculer l'écart crête (ptp) sur ceux-ci.
            uint32_t stable_ms = _restLongStableMs();
            uint32_t cutoff = millis() - stable_ms;
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
            bool time_ok = (millis() - state_entry_ms >= _restLongMinMs());
            bool stable_ok = (kept >= t.rest_long_stable_min_samples)
                          && (v_max - v_min <= t.rest_long_stable_mv);
            rest_stable = stable_ok;

            if (time_ok && stable_ok) {
                mppt_state = State_REST_LONG;
            }
        } else {
            rest_v_count = 0;
            for (uint8_t i = 0; i < REST_LONG_STABLE_MAX_SAMPLES; i++) rest_v_ring_ms[i] = 0;
            rest_stable = false;
        }
    }

    void handleSyncEvents(float V_cell_ocv) {
        float total_ocv = V_cell_ocv * model->getCellCount();
        const KalmanTuning& t = getTuning();

        // F1: ces transitions sont maintenant réellement détectées
        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            doSync(100.0f, t.conf_float, "Float charge");
            return;
        }

        if (mppt_state == State_REST_LONG && millis() - data->last_sync_time > REST_LONG_SYNC_MS) {
            uint32_t rest_min_ms = _restLongMinMs();
            uint32_t extra = millis() - state_entry_ms - rest_min_ms;
            float conf = t.conf_rest_base + t.conf_rest_span * min(1.0f, extra / (float)rest_min_ms);
            doSync(data->SoC_voltage, conf, "Long rest OCV");
            return;
        }

        float min_voltage = model->getMinVoltage();
        if ((mppt_state == State_REST || mppt_state == State_REST_LONG) && total_ocv < min_voltage) {
            doSync(3.0f, 0.75f, "Low voltage cutoff");
            return;
        }

        if (mppt_state == State_REST && millis() - data->last_sync_time > 300000UL) {
            data->SoC_coulomb = 0.90f * data->SoC_coulomb + 0.10f * data->SoC_voltage;
            data->last_sync_time = millis();
        }
    }

    void doSync(float new_SoC, float confidence, const char* reason) {
        data->SoC_coulomb = new_SoC;
        data->SoC_fused = new_SoC;
        data->last_sync_time = millis();
        coulombMeter->reset(false);
        last_Ah = coulombMeter->getAmpereHours();

        snprintf(last_sync_info, sizeof(last_sync_info),
                 "%s conf=%.0f%%", reason, confidence * 100.0f);
        startNewSegment(new_SoC, confidence);
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

        float kalman_conf = kalman->confidence;

        float alpha_base;
        switch (mppt_state) {
            case State_FLOAT:      alpha_base = 0.00f; break;
            case State_REST_LONG:  alpha_base = 0.02f; break;
            case State_REST:       alpha_base = 0.10f; break;
            case State_ABSORPTION: alpha_base = 0.70f; break;
            case State_BULK:       alpha_base = 0.95f; break;
            case State_DISCHARGE:  alpha_base = 0.97f; break;
            default:               alpha_base = 0.50f; break;
        }

        if (phase == PHASE_COARSE) {
            alpha_fused = alpha_base * kalman_conf;
        } else {
            alpha_fused = alpha_base;
        }

        return alpha_fused * data->SoC_coulomb + (1.0f - alpha_fused) * data->SoC_voltage;
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

    void startNewSegment(float SoC_start, float conf) {
        seg.active = true;
        seg.Ah_start = coulombMeter->getAmpereHours();
        seg.SoC_start = SoC_start;
        seg.conf_start = conf;
        seg.start_ms = millis();
        seg.I_sq_sum = 0.0f;
        seg.I_sum = 0.0f;
        seg.n = 0;
        seg.I_min = 999.0f;
        seg.I_max = 0.0f;
        seg.had_discharge = false;
        seg.had_charge = false;
    }

    void updateSegmentAndKalman(float I, float delta_cycles) {
        if (!seg.active) {
            if (mppt_state == State_FLOAT || mppt_state == State_REST_LONG) {
                const KalmanTuning& t = getTuning();
                float conf = (mppt_state == State_FLOAT) ? t.conf_float : (t.conf_rest_base + t.conf_rest_span);
                startNewSegment(data->SoC_fused, conf);
            }
            return;
        }

        float I_abs = fabsf(I);
        seg.I_sum += I_abs;
        seg.I_sq_sum += I_abs * I_abs;
        seg.n++;
        seg.I_min = min(seg.I_min, I_abs);
        seg.I_max = max(seg.I_max, I_abs);
        if (I < -0.05f) seg.had_discharge = true;
        if (I > 0.05f) seg.had_charge = true;

        float SoC_end = 0.0f;
        float conf_end = 0.0f;
        bool close = false;

        // F1: transitions détectées (plus de code mort)
        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            SoC_end = 100.0f; conf_end = getTuning().conf_float; close = true;
        } else if (mppt_state == State_REST_LONG && mppt_state_prev != State_REST_LONG) {
            // F5 + F8: fermeture par transition REST_LONG (porte de stabilité)
            const KalmanTuning& t = getTuning();
            uint32_t rest_min_ms = _restLongMinMs();
            uint32_t extra = millis() - state_entry_ms - rest_min_ms;
            SoC_end = data->SoC_voltage;
            conf_end = t.conf_rest_base + t.conf_rest_span * min(1.0f, extra / (float)rest_min_ms);
            close = true;
        }

        if (!close) return;

        // v1.6.0: gating OCV — segments désactivés pour NiFe/Sodium
        // (OCV peu fiable) sauf si overridé via setSegmentThresholds()
        if (!getTuning().ocv_segment_allowed) return;

        float dAh = fabsf(coulombMeter->getAmpereHours() - seg.Ah_start);
        float dSoC = fabsf(SoC_end - seg.SoC_start);

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
            startNewSegment(SoC_end, conf_end);
            return;
        }

        float C_measured = dAh / (dSoC / 100.0f);

        if (C_measured < 1.0f || C_measured > 2000.0f) {
            startNewSegment(SoC_end, conf_end);
            return;
        }

        // F3: R adaptatif réellement utilisé
        float R = kalman->R_measured;

        // EKF 2D update
        applyKalmanUpdate2D(C_measured, R, delta_cycles);
        startNewSegment(SoC_end, conf_end);
    }

public:
    // ============================================================
    // CONSTRUCTEUR
    // ============================================================
    BatteryKalman(SoCData* soc, KalmanState2D* kstate, BatteryModel* battModel, Coulomb* coulomb)
        : data(soc), kalman(kstate), model(battModel), coulombMeter(coulomb) {}

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
            coulombMeter->begin(0, false);
            last_Ah = coulombMeter->getAmpereHours();
        }

        updatePhaseFromConfidence();
        state_entry_ms = millis();
        last_cycles = data->cycles_partial;
        // F1: garantir un état "précédent" cohérent au démarrage
        mppt_state_prev = mppt_state;
    }

    // ============================================================
    // MISE À JOUR PRINCIPALE
    // ============================================================
    void update(float V, float I, float T) {
        float dt_s = coulombMeter->getLastInterval();
        if (dt_s <= 0) dt_s = 0.1f;
        float dt_h = dt_s / 3600.0f;

        updateDerivatives(V, I, dt_s);

        float r_int_eff;
        float V_cell_ocv = model->correctVoltageToOCV(V, I, T, dt_s, &r_int_eff, nullptr)
                          / model->getCellCount();
        R_int_eff = r_int_eff;

        data->SoC_voltage = model->ocvToSoc(V_cell_ocv * model->getCellCount(), T);

        updateMpptState(V, I);

        float current_Ah = coulombMeter->getAmpereHours();
        float dAh = current_Ah - last_Ah;
        last_Ah = current_Ah;

        if (I < 0.0f)
            data->Ah_total_discharged += (-I) * dt_h;
        else
            data->Ah_total_charged += I * dt_h * 0.98f;

        float C_ref = getEffectiveCapacity();
        if (C_ref > 0.0f) {
            data->SoC_coulomb += (dAh / C_ref) * 100.0f;
            data->SoC_coulomb = constrain(data->SoC_coulomb, 0.0f, 100.0f);
        }

        // F6: apprentissage Kalman TOUJOURS actif (indépendant de isAutoDetect)
        {
            float delta_cycles = data->cycles_partial - last_cycles;
            updateSegmentAndKalman(I, delta_cycles);
            last_cycles = data->cycles_partial;
        }

        handleSyncEvents(V_cell_ocv);

        data->SoC_fused = fuseSoC();
        data->SoC_fused = constrain(data->SoC_fused, 0.0f, 100.0f);
        data->SoC_uncertainty = computeUncertainty();

        updateCycleDetection(I);

        // F1: mppt_state_prev mis à jour en fin de cycle pour la prochaine itération
        mppt_state_prev = mppt_state;
    }

    // ============================================================
    // DÉTECTION DE CYCLES
    // ============================================================
    void updateCycleDetection(float I) {
        static bool initialized = false;
        if (!initialized) {
            cycle.cycle_start_time = millis();
            initialized = true;
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
        doSync(data->SoC_voltage, 0.80f, "Manual sync");
    }

    void syncToFull() {
        doSync(100.0f, 0.95f, "Manual full");
    }

    void syncToEmpty() {
        doSync(0.0f, 0.90f, "Manual empty");
    }

    void resetCoulombCount(float new_SoC = -1.0f) {
        if (new_SoC < 0.0f) new_SoC = data->SoC_voltage;
        data->SoC_coulomb = new_SoC;
        data->SoC_fused = new_SoC;
        coulombMeter->reset(false);
        last_Ah = coulombMeter->getAmpereHours();
        data->last_sync_time = millis();
    }

    void resetLearning() {
        const KalmanTuning& t = getTuning();
        kalman->C_hat = 0.0f;
        kalman->dC_dCycle = -0.0005f;
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

    float getEffectiveCapacity() {
        if (kalman->initialized && kalman->C_hat > 1.0f) return kalman->C_hat;
        if (model->getNominalCapacity() > 0.0f) return model->getNominalCapacity();
        return 0.0f;
    }

    float getAhAvailable() {
        float C = getEffectiveCapacity();
        if (C <= 0.0f || phase == PHASE_BOOTSTRAP) return -1.0f;
        return (data->SoC_fused / 100.0f) * C;
    }

    float getAhIntegrated() {
        return coulombMeter->getAmpereHours();
    }

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
    float getR_estimated() { return kalman->R_estimated; }
    float getR_measured() { return kalman->R_measured; }
    float getAlphaFused() { return alpha_fused; }

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
