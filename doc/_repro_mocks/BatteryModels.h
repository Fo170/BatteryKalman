#pragma once
#include "Arduino.h"

enum ChargeState : uint8_t {
    State_UNKNOWN = 0, State_DISCHARGE = 1, State_BULK = 2, State_ABSORPTION = 3,
    State_FLOAT = 4, State_REST = 5, State_REST_LONG = 6
};

struct KalmanTuning {
    float p_init_C, p_min_C, p_init_aging, p_min_aging;
    float q_aging_uncertainty, q_capacity_drift;
    float r_init, r_min, r_max, r_smooth_alpha;
    float seg_min_dAh_abs, seg_min_dAh_pct, seg_min_dsoc[3], seg_dsoc_high_conf;
    bool  ocv_segment_allowed;
    float battery_change_thr; uint8_t battery_change_count;
    float rest_long_min_min, rest_long_stable_mv, rest_long_stable_min;
    uint8_t rest_long_stable_min_samples, rest_long_stable_max_samples;
    float conf_float, conf_rest_base, conf_rest_span, conf_phase_coarse, conf_phase_refine;
};

// Mock fonctionnel, valeurs réglables par le test.
// - detectChargeState() : renvoie `state` (override) sauf si useAuto=true (détection réelle).
// - ocvToSoc() : table plomb 12 V (V_pack -> SoC%).
class BatteryModel {
public:
    ChargeState state = State_REST;
    bool  useAuto = false;      // false = override `state` (compat) ; true = détection réelle
    float minV  = 10.0f;
    float ocvOut = 8.0f;        // correctVoltageToOCV() renvoie cette valeur (pack)

    void setAutoState(bool a) { useAuto = a; }

    ChargeState detectChargeState(float V, float I, float) const {
        if (!useAuto) return state;
        if (I < -0.05f) return State_DISCHARGE;
        if (V >= 14.10f && I < 0.5f) return State_FLOAT;
        if (fabsf(I) < 0.05f) return State_REST;
        if (I > 0.05f) return State_BULK;
        return State_UNKNOWN;
    }
    float correctVoltageToOCV(float V, float, float, float, float* r, float*) { if (r) *r = 0.0f; return useAuto ? V : ocvOut; }
    float ocvToSoc(float V, float) const {
        static const float T[][2] = {{11.80f,0},{11.90f,20},{12.05f,40},{12.25f,60},{12.45f,80},{12.70f,100}};
        if (V <= T[0][0]) return 0.0f;
        if (V >= T[5][0]) return 100.0f;
        for (int i = 0; i < 5; i++)
            if (V >= T[i][0] && V <= T[i+1][0])
                return T[i][1] + (V-T[i][0])/(T[i+1][0]-T[i][0])*(T[i+1][1]-T[i][1]);
        return 50.0f;
    }
    uint8_t getCellCount() const { return 4; }
    float getNominalCapacity() const { return 100.0f; }
    float getMinVoltage() const { return minV; }
    float getMaxVoltage() const { return 14.0f; }
    bool  isOcvReliable() const { return true; }
    bool  isAutoDetect() const { return true; }
    const char* getTechnologyName() const { return "Plomb"; }

    const KalmanTuning& getKalmanTuning() const {
        static KalmanTuning t = {
            500.0f, 0.10f, 0.000001f, 0.0000001f,
            0.000002f, 0.01f,
            4.0f, 0.5f, 100.0f, 0.1f,
            0.20f, 0.0f, {3.0f, 5.0f, 8.0f}, 15.0f,
            true,
            0.30f, 3,
            15.0f, 0.020f, 30.0f,
            6, 32,
            0.95f, 0.70f, 0.15f, 0.30f, 0.60f
        };
        return t;
    }
};
