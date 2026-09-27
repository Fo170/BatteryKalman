#pragma once

// Mock fonctionnel : compteur coulomb qui intègre I*dt (dt = lastInterval, en s).
class Coulomb {
public:
    float ah = 0.0f;
    float lastInterval = 60.0f;
    int   resetCount = 0;

    void  begin(float v, bool) { ah = v; }
    void  addMeasurement(float I) { ah += I * lastInterval / 3600.0f; }
    void  addMeasurementWithInterval(float I, float dt) { ah += I * dt / 3600.0f; }
    float getAmpereHours() const { return ah; }
    float getLastInterval() const { return lastInterval; }
    void  reset(bool) { resetCount++; ah = 0.0f; }
};
