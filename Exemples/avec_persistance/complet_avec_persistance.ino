#include <BatteryKalman.h>
#include <BatteryModels.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

SoCData socData;
KalmanState2D kalmanState;  // v3.0.0: EKF 2D avec aging learnable
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);
Coulomb coulomb;
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

void setup() {
    Serial.begin(115200);
    LittleFS.begin();
    
    // Charger état depuis LittleFS
    loadBatteryState();
    
    battery.begin();
}

void loop() {
    static uint32_t lastUpdate = 0;
    static uint32_t lastSave = 0;
    uint32_t now = millis();
    
    if (now - lastUpdate >= 100) {
        lastUpdate = now;
        
        float voltage = analogRead(A0) * 0.0125;
        float current = readINA226();  // Votre lecture INA226
        float temp = 25.0;
        
        battery.update(voltage, current, temp, 100);
    }
    
    // Sauvegarde toutes les 5 minutes si nécessaire
    if (now - lastSave >= 300000 && battery.isStateDirty()) {
        lastSave = now;
        saveBatteryState();
        battery.clearStateDirty();
    }
}

void saveBatteryState() {
    File f = LittleFS.open("/battery.json", "w");
    if (!f) return;

    StaticJsonDocument<512> doc;

    // SoCData (inchangé)
    doc["SoC_coulomb"] = socData.SoC_coulomb;
    doc["SoC_voltage"] = socData.SoC_voltage;
    doc["SoC_fused"] = socData.SoC_fused;
    doc["Ah_charged"] = socData.Ah_total_charged;
    doc["Ah_discharged"] = socData.Ah_total_discharged;
    doc["cycles_partial"] = socData.cycles_partial;
    doc["cycles_full"] = socData.cycles_full;
    doc["DoD_acc"] = socData.DoD_accumulated;

    // KalmanState2D (v3.0.0)
    doc["C_hat"] = kalmanState.C_hat;
    doc["dC_dCycle"] = kalmanState.dC_dCycle;  // NEW: taux vieillissement
    doc["P_CC"] = kalmanState.P[0][0];         // NEW: P[2x2]
    doc["P_Caging"] = kalmanState.P[0][1];     // NEW: covariance croisée
    doc["P_aging"] = kalmanState.P[1][1];      // NEW: variance aging
    doc["R_estimated"] = kalmanState.R_estimated;
    doc["confidence"] = kalmanState.confidence;
    doc["n_updates"] = kalmanState.n_updates;
    doc["initialized"] = kalmanState.initialized;

    serializeJson(doc, f);
    f.close();
    Serial.println("État sauvegardé (v3.0.0)");
}

void loadBatteryState() {
    File f = LittleFS.open("/battery.json", "r");
    if (!f) return;

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, f) != DeserializationError::Ok) {
        f.close();
        return;
    }

    // SoCData (inchangé)
    socData.SoC_coulomb = doc["SoC_coulomb"] | 50.0f;
    socData.SoC_voltage = doc["SoC_voltage"] | 50.0f;
    socData.SoC_fused = doc["SoC_fused"] | 50.0f;
    socData.Ah_total_charged = doc["Ah_charged"] | 0.0f;
    socData.Ah_total_discharged = doc["Ah_discharged"] | 0.0f;
    socData.cycles_partial = doc["cycles_partial"] | 0.0f;
    socData.cycles_full = doc["cycles_full"] | 0;
    socData.DoD_accumulated = doc["DoD_acc"] | 0.0f;

    // KalmanState2D (v3.0.0)
    kalmanState.C_hat = doc["C_hat"] | 0.0f;
    kalmanState.dC_dCycle = doc["dC_dCycle"] | -0.0005f;  // Défault si ancien format
    kalmanState.P[0][0] = doc["P_CC"] | KALMAN_P_INIT_C;
    kalmanState.P[0][1] = doc["P_Caging"] | 0.0f;
    kalmanState.P[1][0] = kalmanState.P[0][1];
    kalmanState.P[1][1] = doc["P_aging"] | KALMAN_P_INIT_AGING;
    kalmanState.R_estimated = doc["R_estimated"] | R_INIT;
    kalmanState.confidence = doc["confidence"] | 0.0f;
    kalmanState.n_updates = doc["n_updates"] | 0;
    kalmanState.initialized = doc["initialized"] | false;

    f.close();
    Serial.println("État chargé (v3.0.0)");
}