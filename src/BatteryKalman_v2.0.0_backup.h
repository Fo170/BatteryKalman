/**
 * @file BatteryKalman.h
 * @brief Filtre de Kalman pour l'estimation de l'état de charge batterie
 * 
 * Utilise Coulomb.h pour le comptage précis et BatteryModels.h pour les modèles
 * Version: 2.0.0
 * Date: Mars 2026
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

#ifndef KALMAN_P_INIT
#define KALMAN_P_INIT           500.0f
#endif
#ifndef KALMAN_P_MIN
#define KALMAN_P_MIN            0.30f
#endif

#ifndef PHASE1_P_EXIT
#define PHASE1_P_EXIT           30.0f
#endif
#ifndef PHASE1_MAX_UPDATES
#define PHASE1_MAX_UPDATES      5
#endif
#ifndef PHASE2_P_EXIT
#define PHASE2_P_EXIT           3.0f
#endif
#ifndef PHASE2_MAX_UPDATES
#define PHASE2_MAX_UPDATES      15
#endif

#ifndef R_BASE_PHASE1
#define R_BASE_PHASE1           4.0f
#endif
#ifndef R_BASE_PHASE2
#define R_BASE_PHASE2           8.0f
#endif
#ifndef R_BASE_PHASE3
#define R_BASE_PHASE3           40.0f
#endif

#ifndef PHASE2_WINDOW_SIZE
#define PHASE2_WINDOW_SIZE      2
#endif

#ifndef BATTERY_CHANGE_THR
#define BATTERY_CHANGE_THR      0.30f
#endif
#ifndef BATTERY_CHANGE_COUNT
#define BATTERY_CHANGE_COUNT    3
#endif

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

#ifndef AGING_DRIFT_PER_CYCLE
#define AGING_DRIFT_PER_CYCLE   0.0005f
#endif

// ============================================================
// PHASES D'APPRENTISSAGE
// ============================================================
enum LearningPhase : uint8_t {
    PHASE_BOOTSTRAP  = 0,
    PHASE_COARSE     = 1,
    PHASE_REFINE     = 2,
    PHASE_TRACK      = 3
};

static const char* const PHASE_NAMES[] = {
    "Bootstrap (C inconnue)",
    "Convergence rapide",
    "Raffinement",
    "Suivi vieillissement"
};

// ============================================================
// DONNÉES SoC (état à persister)
// ============================================================
struct SoCData {
    float SoC_coulomb = 50.0f;          // % (comptage coulombs)
    float SoC_voltage = 50.0f;          // % (OCV corrigé)
    float SoC_voltage_raw = 50.0f;      // % (OCV brut - Phase Bootstrap)
    float SoC_fused = 50.0f;            // % (fusion finale)
    float SoC_uncertainty = 10.0f;      // % (incertitude estimée)
    bool  soc_is_raw = true;             // true = afficher ~XX%
    
    float Ah_total_charged = 0.0f;       // Ah total chargés (vie batterie)
    float Ah_total_discharged = 0.0f;    // Ah total déchargés (vie batterie)
    
    float cycles_partial = 0.0f;         // Cycles partiels cumulés
    uint32_t cycles_full = 0;             // Cycles complets (>80% DoD)
    float DoD_accumulated = 0.0f;         // Profondeur décharge cumulée
    
    uint32_t last_sync_time = 0;          // Timestamp dernière synchro
    bool coulomb_initialized = false;     // true après première init
};

// ============================================================
// ÉTAT KALMAN (à persister)
// ============================================================
struct KalmanState {
    float    C_hat = 0.0f;                // Capacité estimée (Ah)
    float    P = KALMAN_P_INIT;            // Variance de l'estimation
    uint16_t n_updates = 0;                // Nombre de mises à jour
    bool     initialized = false;          // true après première estimation
    uint8_t  outlier_streak = 0;           // Compteur outliers consécutifs
};

// ============================================================
// CLASSE PRINCIPALE
// ============================================================
class BatteryKalman {
private:
    SoCData*        data;
    KalmanState*    kalman;
    BatteryModel*   model;                 // Modèle batterie
    Coulomb*        coulombMeter;          // Compteur Coulombs externe

    // États internes
    ChargeState mppt_state = State_UNKNOWN;
    ChargeState mppt_state_prev = State_UNKNOWN;
    uint32_t  state_entry_ms = 0;
    float     I_abs_peak = 0.0f;
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

    struct WindowedEstimate {
        float  C_sum = 0.0f;
        float  R_sum = 0.0f;
        float  dSoC_max = 0.0f;
        uint8_t count = 0;
    } window;

    float R_int_eff = 0.0f;                 // Résistance effective calculée
    bool  state_dirty = false;               // true si sauvegarde nécessaire
    float last_Ah = 0.0f;                    // Dernière valeur Ah pour calcul Δ

    char  phase_info[80] = "Bootstrap";
    char  last_sync_info[64] = "aucune";
    char  last_learn_info[80] = "aucun";
    float alpha_fused = 0.5f;

    struct CycleHistory {
        float V_max_seen = 0.0f;
        float V_min_seen = 999.0f;
        float SoC_max = 0.0f;
        float SoC_min = 100.0f;
        bool was_charging = false;
        bool was_discharging = false;
        uint32_t cycle_start_time = 0;
    } cycle;

    // ============================================================
    // MÉTHODES PRIVÉES
    // ============================================================

    void updatePhase() {
        LearningPhase old_phase = phase;

        if (!kalman->initialized || kalman->n_updates == 0) {
            phase = PHASE_BOOTSTRAP;
        } else if (kalman->n_updates <= PHASE1_MAX_UPDATES && kalman->P >= PHASE1_P_EXIT) {
            phase = PHASE_COARSE;
        } else if (kalman->n_updates <= PHASE2_MAX_UPDATES && kalman->P >= PHASE2_P_EXIT) {
            phase = PHASE_REFINE;
        } else {
            phase = PHASE_TRACK;
        }

        if (phase != old_phase) {
            snprintf(phase_info, sizeof(phase_info), "%s (P=%.1f n=%d)",
                     PHASE_NAMES[phase], kalman->P, kalman->n_updates);
        }
    }

    void updateDerivatives(float V, float I, float dt_s) {
        if (dt_s <= 0.0f || V_prev <= 0.0f) {
            V_prev = V; I_prev = I; return;
        }
        float alpha_f = dt_s / (20.0f + dt_s); // tau = 20s
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

        // Détection repos long
        if (mppt_state == State_REST && millis() - state_entry_ms >= 7200000UL) {
            mppt_state = State_REST_LONG;
        }
    }

    void handleSyncEvents(float V_cell_ocv) {
        float total_ocv = V_cell_ocv * model->getCellCount();

        // Priorité 1 : Float = charge complète
        if (mppt_state == State_FLOAT && mppt_state_prev != State_FLOAT) {
            doSync(100.0f, 0.95f, "Float charge");
            return;
        }

        // Priorité 2 : Repos long = OCV fiable
        if (mppt_state == State_REST_LONG && millis() - data->last_sync_time > 1800000UL) {
            uint32_t extra = millis() - state_entry_ms - 7200000UL;
            float conf = 0.70f + 0.15f * min(1.0f, extra / 7200000.0f);
            doSync(data->SoC_voltage, conf, "Long rest OCV");
            return;
        }

        // Priorité 3 : Tension plancher
        float min_voltage = model->getMinVoltage();
        if ((mppt_state == State_REST || mppt_state == State_REST_LONG) && total_ocv < min_voltage) {
            doSync(3.0f, 0.75f, "Low voltage cutoff");
            return;
        }

        // Priorité 4 : Dérive douce au repos court
        if (mppt_state == State_REST && millis() - data->last_sync_time > 300000UL) {
            data->SoC_coulomb = 0.90f * data->SoC_coulomb + 0.10f * data->SoC_voltage;
            data->last_sync_time = millis();
        }
    }

    void doSync(float new_SoC, float confidence, const char* reason) {
        float old = data->SoC_coulomb;
        data->SoC_coulomb = new_SoC;
        data->SoC_fused = new_SoC;
        data->last_sync_time = millis();
        
        // Réinitialiser le compteur Coulomb pour aligner avec la nouvelle référence
        coulombMeter->reset(false); // Garde la valeur persistée
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
            // Utiliser OCV brut (table Unknown)
            data->SoC_voltage_raw = (V_prev > 1.0f) 
                ? model->ocvToSoc(V_prev, 25.0f) 
                : data->SoC_voltage;
            return data->SoC_voltage_raw;
        }

        data->soc_is_raw = false;

        // Facteur de confiance Kalman
        float kalman_conf = 0.0f;
        if (kalman->initialized) {
            float n_factor = min(1.0f, (float)kalman->n_updates / 5.0f);
            float p_factor = 1.0f - min(1.0f, kalman->P / KALMAN_P_INIT);
            kalman_conf = (n_factor + p_factor) * 0.5f;
        }

        // Alpha de base selon état de charge
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

    void updateSegmentAndKalman(float I) {
        if (!seg.active) {
            if (mppt_state == State_FLOAT || mppt_state == State_REST_LONG) {
                float conf = (mppt_state == State_FLOAT) ? 0.95f : 0.75f;
                startNewSegment(data->SoC_fused, conf);
            }
            return;
        }

        // Accumuler stats courant (via Coulomb meter)
        float I_abs = fabsf(I);
        seg.I_sum += I_abs;
        seg.I_sq_sum += I_abs * I_abs;
        seg.n++;
        seg.I_min = min(seg.I_min, I_abs);
        seg.I_max = max(seg.I_max, I_abs);
        if (I < -0.05f) seg.had_discharge = true;
        if (I > 0.05f) seg.had_charge = true;

        // Chercher condition de fermeture
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

        float R_base = getRBase();
        float R = computeR(R_base, dSoC, conf_end, seg.conf_start,
                           seg.I_min, seg.I_max, seg.I_sq_sum, seg.I_sum, seg.n,
                           seg.had_discharge, seg.had_charge);

        if (phase == PHASE_BOOTSTRAP) {
            kalman->C_hat = C_measured;
            kalman->P = R;
            kalman->initialized = true;
            kalman->n_updates = 1;
            updatePhase();
            state_dirty = true;
            startNewSegment(SoC_end, conf_end);
            return;
        }

        if (phase == PHASE_COARSE || phase == PHASE_TRACK) {
            applyKalmanUpdate(C_measured, R, dSoC);
            startNewSegment(SoC_end, conf_end);
            return;
        }

        if (phase == PHASE_REFINE) {
            window.C_sum += C_measured / R;
            window.R_sum += 1.0f / R;
            window.dSoC_max = max(window.dSoC_max, dSoC);
            window.count++;

            if (window.count >= PHASE2_WINDOW_SIZE) {
                float C_window = window.C_sum / window.R_sum;
                float R_window = 1.0f / window.R_sum;
                applyKalmanUpdate(C_window, R_window, window.dSoC_max);
                window = WindowedEstimate();
            }

            startNewSegment(SoC_end, conf_end);
        }
    }

    void applyKalmanUpdate(float C_measured, float R, float dSoC) {
        if (!kalman->initialized) {
            kalman->C_hat = C_measured;
            kalman->P = R;
            kalman->initialized = true;
            kalman->n_updates = 1;
            updatePhase();
            state_dirty = true;
            return;
        }

        float sigma = sqrtf(kalman->P + R);
        if (fabsf(C_measured - kalman->C_hat) > 3.0f * sigma) {
            kalman->outlier_streak++;

            if (phase == PHASE_TRACK && kalman->outlier_streak >= BATTERY_CHANGE_COUNT
                && fabsf(C_measured - kalman->C_hat) > BATTERY_CHANGE_THR * kalman->C_hat) {
                kalman->C_hat = C_measured;
                kalman->P = R * 2.0f;
                kalman->n_updates = PHASE1_MAX_UPDATES;
                kalman->outlier_streak = 0;
                snprintf(last_learn_info, sizeof(last_learn_info),
                         "Battery replaced: C=%.1fAh", C_measured);
            } else {
                kalman->P = min(kalman->P * 2.0f, KALMAN_P_INIT * 0.5f);
            }
            updatePhase();
            state_dirty = true;
            return;
        }

        kalman->outlier_streak = 0;

        float K = kalman->P / (kalman->P + R);
        kalman->C_hat += K * (C_measured - kalman->C_hat);
        kalman->P = max(KALMAN_P_MIN, (1.0f - K) * kalman->P);
        kalman->n_updates++;

        snprintf(last_learn_info, sizeof(last_learn_info),
                 "Phase %d: C=%.1fAh P=%.2f n=%d dSoC=%.0f%%",
                 phase, kalman->C_hat, kalman->P, kalman->n_updates, dSoC);

        updatePhase();
        state_dirty = true;
    }

    float getRBase() {
        switch (phase) {
            case PHASE_BOOTSTRAP: return R_BASE_PHASE1 * 0.5f;
            case PHASE_COARSE:    return R_BASE_PHASE1;
            case PHASE_REFINE:    return R_BASE_PHASE2;
            case PHASE_TRACK:     return R_BASE_PHASE3;
        }
        return R_BASE_PHASE2;
    }

    float computeR(float R_base, float dSoC,
                   float conf_end, float conf_start,
                   float I_min, float I_max,
                   float I_sq_sum, float I_sum, uint32_t n,
                   bool had_discharge, bool had_charge) {
        float R = R_base;

        if (dSoC >= SEG_DSOC_HIGH_CONF) R *= 0.40f;
        else if (dSoC >= SEG_MIN_DSOC_P1) R *= 1.00f;
        else R *= 2.50f;

        float avg_conf = (conf_end + conf_start) * 0.5f;
        if (avg_conf >= 0.90f) R *= 0.50f;
        else if (avg_conf >= 0.75f) R *= 1.00f;
        else R *= 2.00f;

        if (n > 1 && I_sum > 0.0f) {
            float I_mean = I_sum / n;
            float var_I = (I_sq_sum / n) - (I_mean * I_mean);
            float cv = (I_mean > 0.01f) ? sqrtf(max(0.0f, var_I)) / I_mean : 1.0f;
            if (cv < 0.20f) R *= 0.60f;
            else if (cv < 0.50f) R *= 1.00f;
            else if (cv < 1.00f) R *= 1.40f;
            else R *= 1.80f;
        }

        if (!had_discharge || !had_charge) R *= 1.15f;

        return constrain(R, R_BASE_PHASE1 * 0.2f, R_BASE_PHASE3 * 5.0f);
    }

    void applyAgingDrift() {
        static float last_cycles = 0.0f;
        float new_cycles = data->cycles_partial;
        float delta_cycles = new_cycles - last_cycles;

        if (delta_cycles > 0.01f && kalman->initialized) {
            float drift = model->getAgingDriftPerCycle();
            float C_drift = kalman->C_hat * (1.0f - drift * delta_cycles);
            if (fabsf(C_drift - kalman->C_hat) > 0.1f) {
                kalman->C_hat = max(C_drift, 1.0f);
            }
            last_cycles = new_cycles;
        }
    }

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

        if (millis() - cycle.cycle_start_time > 604800000UL) { // 7 jours
            cycle = CycleHistory();
            cycle.cycle_start_time = millis();
        }
    }

public:
    // ============================================================
    // CONSTRUCTEUR
    // ============================================================
    BatteryKalman(SoCData* soc, KalmanState* kstate, BatteryModel* battModel, Coulomb* coulomb)
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
            
            // Initialiser le compteur Coulomb
            coulombMeter->begin(0, false);
            last_Ah = coulombMeter->getAmpereHours();
        }

        updatePhase();
        state_entry_ms = millis();
    }

    // ============================================================
    // MISE À JOUR PRINCIPALE
    // ============================================================
    void update(float V, float I, float T) {
        // Obtenir l'intervalle depuis le compteur Coulomb
        float dt_s = coulombMeter->getLastInterval();
        float dt_h = dt_s / 3600.0f;
        uint32_t dt_ms = dt_s * 1000;

        updateDerivatives(V, I, dt_s);
        
        // Utiliser le modèle pour corriger la tension
        float r_int_eff;
        float V_cell_ocv = model->correctVoltageToOCV(V, I, T, dt_s, &r_int_eff, nullptr) 
                          / model->getCellCount();
        R_int_eff = r_int_eff;
        
        // SoC via OCV
        data->SoC_voltage = model->ocvToSoc(V_cell_ocv * model->getCellCount(), T);

        // Mise à jour état MPPT
        updateMpptState(V, I);

        // Mise à jour des Ah depuis le compteur Coulomb
        float current_Ah = coulombMeter->getAmpereHours();
        float dAh = current_Ah - last_Ah;
        last_Ah = current_Ah;

        // Mettre à jour les totaux de vie
        if (I < 0.0f)
            data->Ah_total_discharged += (-I) * dt_h;
        else
            data->Ah_total_charged += I * dt_h * 0.98f; // Rendement Faraday

        // Mise à jour SoC coulombs
        float C_ref = getEffectiveCapacity();
        if (C_ref > 0.0f) {
            data->SoC_coulomb += (dAh / C_ref) * 100.0f;
            data->SoC_coulomb = constrain(data->SoC_coulomb, 0.0f, 100.0f);
        }

        // Synchronisations automatiques
        handleSyncEvents(V_cell_ocv);

        // Fusion SoC
        data->SoC_fused = fuseSoC();
        data->SoC_fused = constrain(data->SoC_fused, 0.0f, 100.0f);
        data->SoC_uncertainty = computeUncertainty();

        // Apprentissage capacité
        if (model->isAutoDetect()) {
            updateSegmentAndKalman(I);
        }

        // Vieillissement
        if (phase == PHASE_TRACK) {
            applyAgingDrift();
        }

        // Détection cycles
        updateCycleDetection(I);
    }

    // ============================================================
    // SYNCHRONISATIONS
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
        *kalman = KalmanState();
        window = WindowedEstimate();
        seg = Segment();
        phase = PHASE_BOOTSTRAP;
        snprintf(phase_info, sizeof(phase_info), "Bootstrap (manual reset)");
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
    float getRintEff() { return R_int_eff * 1000.0f; } // mΩ
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
    const char* getPhaseInfo() { return phase_info; }
    const char* getLastSyncInfo() { return last_sync_info; }
    const char* getLastLearnInfo() { return last_learn_info; }
    float getKalmanP() { return kalman->P; }
    uint16_t getKalmanUpdates() { return kalman->n_updates; }
    float getAlphaFused() { return alpha_fused; }

    // ============================================================
    // PERSISTANCE (à adapter selon stockage)
    // ============================================================
    void saveState() {
        // Exemple avec ArduinoJson (à adapter)
        /*
        StaticJsonDocument<256> doc;
        doc["SoC_coulomb"] = data->SoC_coulomb;
        doc["Ah_charged"] = data->Ah_total_charged;
        doc["Ah_discharged"] = data->Ah_total_discharged;
        doc["cycles_partial"] = data->cycles_partial;
        doc["cycles_full"] = data->cycles_full;
        doc["DoD_acc"] = data->DoD_accumulated;
        doc["C_hat"] = kalman->C_hat;
        doc["P"] = kalman->P;
        doc["n_updates"] = kalman->n_updates;
        doc["initialized"] = kalman->initialized;
        // Sauvegarder...
        */
        state_dirty = false;
    }

    void loadState() {
        // Exemple avec ArduinoJson (à adapter)
        /*
        StaticJsonDocument<256> doc;
        // Charger...
        data->SoC_coulomb = doc["SoC_coulomb"] | 50.0f;
        data->Ah_total_charged = doc["Ah_charged"] | 0.0f;
        data->Ah_total_discharged = doc["Ah_discharged"] | 0.0f;
        data->cycles_partial = doc["cycles_partial"] | 0.0f;
        data->cycles_full = doc["cycles_full"] | 0;
        data->DoD_accumulated = doc["DoD_acc"] | 0.0f;
        kalman->C_hat = doc["C_hat"] | 0.0f;
        kalman->P = doc["P"] | KALMAN_P_INIT;
        kalman->n_updates = doc["n_updates"] | 0;
        kalman->initialized = doc["initialized"] | false;
        */
        
        if (!kalman->initialized && kalman->C_hat > 1.0f) {
            kalman->initialized = true;
        }
    }
};

#endif // BATTERY_KALMAN_H