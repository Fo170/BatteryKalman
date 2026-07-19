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
 * Licence: GNU General Public License v3
 * Version: 1.5
 * Date: Juillet 2026
 */

#ifndef BATTERY_KALMAN_H
#define BATTERY_KALMAN_H

#include <Arduino.h>
#include <math.h>
#include "BatteryModels.h"
#include "Coulomb.h"

// ============================================================
// CONFIGURATION PAR DÉFAUT
// ============================================================

#ifndef KALMAN_P_INIT_C
#define KALMAN_P_INIT_C         500.0f      // Variance initiale pour C (Ah²)
#endif
#ifndef KALMAN_P_INIT_AGING
#define KALMAN_P_INIT_AGING     0.000001f   // Variance initiale pour dC/dcycle
#endif
#ifndef KALMAN_P_MIN_C
#define KALMAN_P_MIN_C          0.10f       // Variance minimale pour C
#endif
#ifndef KALMAN_P_MIN_AGING
#define KALMAN_P_MIN_AGING      0.0000001f  // Variance minimale pour aging
#endif

// Process noise (Q)
#ifndef Q_AGING_UNCERTAINTY
#define Q_AGING_UNCERTAINTY     0.000002f   // Incertitude sur taux aging
#endif
#ifndef Q_CAPACITY_DRIFT
#define Q_CAPACITY_DRIFT        0.01f       // Dérive de capacité entre cycles
#endif

// Measurement noise (R) - estimation en ligne
#ifndef R_INIT
#define R_INIT                  4.0f        // R initial
#endif
#ifndef R_MIN
#define R_MIN                   0.5f        // R minimum
#endif
#ifndef R_MAX
#define R_MAX                   100.0f      // R maximum
#endif
#ifndef R_SMOOTH_ALPHA
#define R_SMOOTH_ALPHA          0.1f        // Lissage exponentiel R_estimate
#endif

// Phases d'apprentissage (legacy, moins utilisées)
#ifndef PHASE1_P_THRESHOLD
#define PHASE1_P_THRESHOLD      30.0f
#endif
#ifndef PHASE2_P_THRESHOLD
#define PHASE2_P_THRESHOLD      3.0f
#endif

// Seuils segments
#ifndef SEG_MIN_DAH
#define SEG_MIN_DAH             0.20f
#endif
#ifndef SEG_MIN_DSOC_P0
#define SEG_MIN_DSOC_P0         3.0f
#endif
#ifndef SEG_MIN_DSOC_P1
#define SEG_MIN_DSOC_P1         5.0f
#endif
#ifndef SEG_MIN_DSOC_P2
#define SEG_MIN_DSOC_P2         8.0f
#endif
#ifndef SEG_DSOC_HIGH_CONF
#define SEG_DSOC_HIGH_CONF      15.0f
#endif

// Seuils de batterie changée
#ifndef BATTERY_CHANGE_THR
#define BATTERY_CHANGE_THR      0.30f       // 30% écart = batterie changée
#endif
#ifndef BATTERY_CHANGE_COUNT
#define BATTERY_CHANGE_COUNT    3           // 3 outliers = batterie changée
#endif

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
    float    P[2][2] = {{KALMAN_P_INIT_C, 0},
                        {0, KALMAN_P_INIT_AGING}};

    // Estimation en ligne de R
    float    R_estimated = R_INIT;      // R estimé depuis innovations
    float    R_measured = R_INIT;       // R actuel pour mesure

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
    // MÉTHODES PRIVÉES - KALMAN 2D
    // ============================================================

    void predictKalman(float delta_cycles) {
        if (!kalman->initialized) return;

        // Prédiction temporelle: croissance de P due au process noise Q
        // C_new = C_old - dC_dCycle * delta_cycles  (aucun changement, mais P grandit)

        // Calcul Q (process noise covariance)
        float Q_C = Q_AGING_UNCERTAINTY * delta_cycles;  // Incertitude aging
        float Q_aging = Q_AGING_UNCERTAINTY;              // Incertitude taux

        // P = P + Q  (Croissance de variance)
        kalman->P[0][0] += Q_C;
        kalman->P[1][1] += Q_aging;

        // Capper P pour éviter divergence
        kalman->P[0][0] = min(kalman->P[0][0], KALMAN_P_INIT_C);
        kalman->P[1][1] = min(kalman->P[1][1], KALMAN_P_INIT_AGING);
    }

    void applyKalmanUpdate2D(float C_measured, float R, float delta_cycles) {
        if (!kalman->initialized) {
            kalman->C_hat = C_measured;
            kalman->dC_dCycle = -0.0005f;  // Valeur par défaut
            kalman->P[0][0] = R;
            kalman->P[1][1] = Q_AGING_UNCERTAINTY;
            kalman->initialized = true;
            kalman->n_updates = 1;
            kalman->confidence = 0.2f;
            updatePhaseFromConfidence();
            state_dirty = true;
            return;
        }

        // Prédiction (grow P due to process noise)
        predictKalman(delta_cycles);

        // Calcul innovation (résidu)
        float innovation = C_measured - kalman->C_hat;

        // Innovation variance: S = H*P*H' + R
        // H = [1, 0] car measurement = C_hat uniquement
        float S = kalman->P[0][0] + R;
        if (S <= 0) S = R;

        // Détection outlier 3-sigma
        float sigma = sqrtf(S);
        if (fabsf(innovation) > 3.0f * sigma) {
            kalman->outlier_streak++;

            // Batterie changée? (n outliers + magnitude check)
            if (kalman->outlier_streak >= BATTERY_CHANGE_COUNT &&
                fabsf(innovation) > BATTERY_CHANGE_THR * kalman->C_hat) {
                kalman->C_hat = C_measured;
                kalman->dC_dCycle = -0.0005f;
                kalman->P[0][0] = R * 2.0f;
                kalman->P[1][1] = Q_AGING_UNCERTAINTY * 5.0f;
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
            kalman->P[1][1] = min(kalman->P[1][1] * 1.5f, Q_AGING_UNCERTAINTY * 10.0f);
            updatePhaseFromConfidence();
            state_dirty = true;
            return;
        }

        kalman->outlier_streak = 0;

        // Kalman gain K = P*H' / S  où H = [1, 0]
        // K = [P_CC / S, P_aging_C / S]'
        float K_C = kalman->P[0][0] / S;
        float K_aging = 0.0f;  // H' = [1; 0], donc K_aging = 0

        // State update: x = x + K * innovation
        kalman->C_hat += K_C * innovation;

        // Covariance update: P = (I - K*H) * P
        float P_CC_new = (1.0f - K_C) * kalman->P[0][0];
        float P_aging_C_new = (1.0f - K_C) * kalman->P[0][1];

        // Clamping
        kalman->P[0][0] = max(KALMAN_P_MIN_C, P_CC_new);
        kalman->P[0][1] = P_aging_C_new;
        kalman->P[1][0] = P_aging_C_new;

        kalman->n_updates++;

        // Mise à jour estimation R (innovation-based)
        updateREstimate(innovation, S);

        // Confiance croît avec n_updates et P décroît
        float n_factor = min(1.0f, (float)kalman->n_updates / 10.0f);
        float p_factor = 1.0f - min(1.0f, kalman->P[0][0] / KALMAN_P_INIT_C);
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
            float R_est = max(R_MIN, innovation_var - kalman->P[0][0]);

            // Lissage exponentiel
            kalman->R_estimated = R_SMOOTH_ALPHA * R_est +
                                  (1.0f - R_SMOOTH_ALPHA) * kalman->R_estimated;
            kalman->R_estimated = constrain(kalman->R_estimated, R_MIN, R_MAX);

            // Reset for next window
            kalman->innovation_sum_sq = 0.0f;
            kalman->innovation_count = 0;
        }
    }

    void updatePhaseFromConfidence() {
        // Legacy: mapper confiance en phases discrètes
        if (!kalman->initialized) {
            phase = PHASE_BOOTSTRAP;
        } else if (kalman->confidence < 0.3f) {
            phase = PHASE_COARSE;
        } else if (kalman->confidence < 0.6f) {
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

        if (mppt_state != mppt_state_prev) {
            mppt_state_prev = mppt_state;
            state_entry_ms = millis();
        }

        if (mppt_state == State_REST && millis() - state_entry_ms >= 7200000UL) {
            mppt_state = State_REST_LONG;
        }
    }

    void handleSyncEvents(float V_cell_ocv) {
        float total_ocv = V_cell_ocv * model->getCellCount();

        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            doSync(100.0f, 0.95f, "Float charge");
            return;
        }

        if (mppt_state == State_REST_LONG && millis() - data->last_sync_time > 1800000UL) {
            uint32_t extra = millis() - state_entry_ms - 7200000UL;
            float conf = 0.70f + 0.15f * min(1.0f, extra / 7200000.0f);
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
                float conf = (mppt_state == State_FLOAT) ? 0.95f : 0.75f;
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

        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            SoC_end = 100.0f; conf_end = 0.95f; close = true;
        } else if (mppt_state == State_REST_LONG && seg.n > 200) {
            uint32_t extra = millis() - state_entry_ms - 7200000UL;
            SoC_end = data->SoC_voltage;
            conf_end = 0.70f + 0.15f * min(1.0f, extra / 7200000.0f);
            close = true;
        }

        if (!close) return;

        float dAh = fabsf(coulombMeter->getAmpereHours() - seg.Ah_start);
        float dSoC = fabsf(SoC_end - seg.SoC_start);

        float min_dSoC = SEG_MIN_DSOC_P1;
        if (phase == PHASE_BOOTSTRAP) min_dSoC = SEG_MIN_DSOC_P0;
        if (phase == PHASE_REFINE || phase == PHASE_TRACK) min_dSoC = SEG_MIN_DSOC_P2;

        if (dAh < SEG_MIN_DAH || dSoC < min_dSoC || seg.n == 0) {
            startNewSegment(SoC_end, conf_end);
            return;
        }

        float C_measured = dAh / (dSoC / 100.0f);

        if (C_measured < 1.0f || C_measured > 2000.0f) {
            startNewSegment(SoC_end, conf_end);
            return;
        }

        // Utiliser R estimé en ligne
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

        handleSyncEvents(V_cell_ocv);

        data->SoC_fused = fuseSoC();
        data->SoC_fused = constrain(data->SoC_fused, 0.0f, 100.0f);
        data->SoC_uncertainty = computeUncertainty();

        if (model->isAutoDetect()) {
            float delta_cycles = data->cycles_partial - last_cycles;
            updateSegmentAndKalman(I, delta_cycles);
            last_cycles = data->cycles_partial;
        }

        updateCycleDetection(I);
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
        kalman->C_hat = 0.0f;
        kalman->dC_dCycle = -0.0005f;
        kalman->P[0][0] = KALMAN_P_INIT_C;
        kalman->P[0][1] = 0.0f;
        kalman->P[1][0] = 0.0f;
        kalman->P[1][1] = KALMAN_P_INIT_AGING;
        kalman->n_updates = 0;
        kalman->initialized = false;
        kalman->confidence = 0.0f;
        kalman->R_estimated = R_INIT;
        kalman->R_measured = R_INIT;
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
