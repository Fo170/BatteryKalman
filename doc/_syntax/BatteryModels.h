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

class BatteryModel {
public:
    const char* _stubName = "stub";
    void setStubName(const char* n) { _stubName = n; }
    ChargeState detectChargeState(float, float, float) const { return State_REST; }
    float correctVoltageToOCV(float V, float, float, float, float* r, float*) { if (r) *r = 0.0f; return V; }
    float ocvToSoc(float, float) const { return 50.0f; }
    uint8_t getCellCount() const { return 4; }
    float getNominalCapacity() const { return 100.0f; }
    float getMinVoltage() const { return 10.0f; }
    float getMaxVoltage() const { return 14.0f; }
    bool  isOcvReliable() const { return true; }
    bool  isAutoDetect() const { return true; }
    const char* getTechnologyName() const { return _stubName; }
    const KalmanTuning& getKalmanTuning() const { static KalmanTuning t = {}; return t; }
};
