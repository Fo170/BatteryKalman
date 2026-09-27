#include "BatteryKalman.h"

SoCData       soc;
KalmanState2D ks;
BatteryModel  model;
Coulomb       coulomb;
BatteryKalman bk(&soc, &ks, &model, &coulomb);

int main() {
    bk.begin();
    for (int i = 0; i < 100; i++) {
        bk.update(12.8f, 1.0f, 25.0f);
        bk.update(12.8f, -1.0f, 25.0f);
    }
    bk.syncToVoltage();
    bk.setP(500.0f, 0.1f, 1e-6f, 1e-7f);
    bk.setRestLong(15.0f, 0.02f, 30.0f, 6);
    bk.applyRecommendedRestLong();
    (void)bk.getSoC();
    (void)bk.getPhaseInfo();
    (void)bk.getKalmanAgingRate();
    return 0;
}
