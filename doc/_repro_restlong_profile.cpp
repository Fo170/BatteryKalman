// R7: verifie applyRecommendedRestLong() contre l'en-tete reel, par technologie.
#include "BatteryKalman.h"
#include <cstdio>

SoCData       soc;
KalmanState2D ks;
BatteryModel  model;
Coulomb       coulomb;
BatteryKalman bk(&soc, &ks, &model, &coulomb);

int main() {
    const char* names[] = {
        "Plomb Inonde (BCI)", "Plomb AGM", "Plomb Gel",
        "LiFePO4 (LFP)", "Li-Ion (NMC/NCA)", "Li-Titanate (LTO)",
        "Nickel-Metal Hydride (NiMH)", "Nickel-Cadmium (NiCd)",
        "Sodium-Ion (Na-ion)", "Supercondensateur (EDLC)"
    };
    printf("Profil REST_LONG applique par technologie (R7):\n");
    printf("  %-30s | min_min | fenetre | seuil(mV)\n", "technologie");
    for (const char* n : names) {
        model.setStubName(n);
        bk.resetTuning();
        bk.applyRecommendedRestLong();
        const KalmanTuning& t = bk.getTuning();
        printf("  %-30s | %6.1f  | %6.1f  | %6.1f\n",
               n, t.rest_long_min_min, t.rest_long_stable_min, t.rest_long_stable_mv * 1000.0f);
    }
    return 0;
}
