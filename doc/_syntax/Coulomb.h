#pragma once

class Coulomb {
public:
    void  begin(float, bool) {}
    void  addMeasurement(float) {}
    void  addMeasurementWithInterval(float, float) {}
    float getAmpereHours() const { return 0.0f; }
    float getLastInterval() const { return 0.1f; }
    void  reset(bool) {}
};
