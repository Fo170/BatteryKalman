#include <BatteryKalman.h>
#include <BatteryModels.h>

// Données à persister
SoCData socData;
KalmanState2D kalmanState;  // v3.0.0: EKF 2D (Capacity + Aging)

// Modèle batterie (LiFePO4 12.8V 100Ah)
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);
Coulomb coulomb;  // Compteur coulombs (dépend de ton implémentation)

// Instance du filtre Kalman amélioré
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

void setup() {
    Serial.begin(115200);
    
    // Initialisation
    battery.begin();
    
    Serial.println("BatteryKalman démarré");
    Serial.print("Technologie: ");
    Serial.println(model.getTechnologyName());
}

void loop() {
    static uint32_t lastUpdate = 0;
    uint32_t now = millis();
    
    // Lecture à 10Hz
    if (now - lastUpdate >= 100) {
        lastUpdate = now;
        
        // Lire tension (V), courant (A), température (°C)
        float voltage = readVoltage();   // À implémenter
        float current = readCurrent();   // À implémenter
        float temp = readTemperature();  // À implémenter
        
        // Mise à jour Kalman
        battery.update(voltage, current, temp, 100);
        
        // Résultats
        if (battery.isSoCKnown()) {
            Serial.printf("SoC: %.1f%%, Cap: %.1fAh, Phase: %s\n",
                         battery.getSoC(),
                         battery.getEffectiveCapacity(),
                         battery.getLearningPhaseStr());
        } else {
            Serial.printf("SoC: ~%.0f%% (Phase Bootstrap)\n",
                         battery.getSoCRaw());
        }
    }
}