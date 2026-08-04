#include <BatteryKalman.h>
#include <BatteryModels.h>

// Données à persister
SoCData socData;
KalmanState2D kalmanState;  // EKF 2D (Capacity + Aging)

// Modèle batterie (LiFePO4 12.8V 100Ah)
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);
Coulomb coulomb;  // Compteur coulombs (dépend de ton implémentation)

// Instance du filtre Kalman amélioré
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

void setup() {
    Serial.begin(115200);

    // Initialisation
    battery.begin();

    Serial.println("BatteryKalman v1.6.0 démarré");
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

        // 1) Alimenter le compteur coulomb avec le courant. CoulombsAh v1.1+
        //    intègre I*dt en interne (API: addMeasurement(current) — utilise
        //    micros() pour le dt). getLastInterval() est ensuite lu par
        //    battery.update(). Utilisez addMeasurementWithInterval(I, dt_s)
        //    si vous fournissez l'intervalle vous-même.
        coulomb.addMeasurement(current);

        // 2) Mise à jour Kalman — API v1.6.0: update(V, I, T)
        battery.update(voltage, current, temp);

        // Résultats
        if (battery.isSoCKnown()) {
            Serial.printf("SoC: %.1f%%, Cap: %.1fAh, Phase: %s, conf: %.0f%%\n",
                         battery.getSoC(),
                         battery.getEffectiveCapacity(),
                         battery.getLearningPhaseStr(),
                         battery.getConfidence() * 100.0f);
        } else {
            Serial.printf("SoC: ~%.0f%% (Phase Bootstrap)\n",
                         battery.getSoCRaw());
        }
    }
}
