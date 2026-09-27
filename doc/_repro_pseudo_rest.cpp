// Reproduction autonome (g++) : P1 (pseudo-repos) + P2 (régression pondérée).
// Scénario PASSIF : cycles quotidiens DOUX (10 %) + UNE décharge PROFONDE
// ponctuelle (50 %) -> la pondération par dSoC² fait que le cycle profond
// "recale" l'estimation. Objectif : rester proche de la capacité réelle.
// Compilation :
//   g++ -std=c++11 -I doc/_repro_mocks -I src -o doc/_repro_pseudo_rest.exe doc/_repro_pseudo_rest.cpp
#include "Arduino.h"
#include "BatteryModels.h"
#include "Coulomb.h"
#include "BatteryKalman.h"

unsigned long g_millis = 0;
static SoCData soc; static KalmanState2D ks; static BatteryModel model; static Coulomb coul;
static BatteryKalman bk(&soc, &ks, &model, &coul);

static const float C_TRUE = 60.0f;
static const float R_INT  = 0.090f;
static float trueSoC = 100.0f;

static float socToOcv(float s) {
    static const float T[][2] = {{11.80f,0},{11.90f,20},{12.05f,40},{12.25f,60},{12.45f,80},{12.70f,100}};
    if (s <= 0) return T[0][0];
    if (s >= 100) return T[5][0];
    for (int i = 0; i < 5; i++)
        if (s >= T[i][1] && s <= T[i+1][1])
            return T[i][0] + (s-T[i][1])/(T[i+1][1]-T[i][1])*(T[i+1][0]-T[i][0]);
    return 12.2f;
}
static void core(float V, float I, float dt_s) {
    coul.lastInterval = dt_s; coul.addMeasurement(I);
    g_millis += (unsigned long)(dt_s * 1000.0f);
    if (V < 10.0f) V = 10.0f;
    if (V > 14.30f) V = 14.30f;
    bk.update(V, I, 25.0f);
    trueSoC += (I * (dt_s / 3600.0f)) / C_TRUE * 100.0f;
    if (trueSoC < 0) trueSoC = 0;
    if (trueSoC > 100) trueSoC = 100;
}
static void stepI(float I, float dt_s) { core(socToOcv(trueSoC) + I * R_INT, I, dt_s); }
static void stepV(float V, float I, float dt_s) { core(V, I, dt_s); }

// un cycle complet : décharge de `dod` %, pseudo-repos, recharge, FLOAT
static void cycle(float dod_pct) {
    const float DT = 300.0f;
    float night_I = dod_pct / 100.0f * C_TRUE / 11.0f;
    while (trueSoC > 100.0f - dod_pct) stepI(-night_I, DT);
    for (int k = 0; k < 3; k++) stepI(0.0f, DT);                 // pseudo-repos
    for (int k = 0; k < (int)(11 * 12); k++) stepI(2.0f, DT);    // bulk
    for (int k = 0; k < (int)(2 * 12); k++) stepV(14.25f, 0.40f, DT);  // float
}

int main() {
    model.setAutoState(true);
    bk.begin();
    bk.applyRecommendedRestLong();
    bk.enablePseudoRest(0.05f, 2, 0.5f, 0.0f);

    printf("Capacité réelle = %.1f Ah — cycles doux (10%%) + 1 décharge profonde (50%%)\n\n", C_TRUE);
    for (int d = 0; d < 6; d++) cycle(10.0f);
    printf("après 6 cycles doux  : C_reg = %6.1f Ah  (max DoD %.0f%%, %u pts)\n",
           bk.getRegressionCapacity(), bk.getRegressionMaxDoD(), bk.getRegressionPoints());
    cycle(50.0f);   // décharge profonde ponctuelle
    printf("après 1 cycle profond: C_reg = %6.1f Ah  (max DoD %.0f%%, %u pts)\n",
           bk.getRegressionCapacity(), bk.getRegressionMaxDoD(), bk.getRegressionPoints());
    for (int d = 0; d < 5; d++) cycle(10.0f);
    printf("après 5 doux de plus : C_reg = %6.1f Ah  (écart %+.1f%%)\n",
           bk.getRegressionCapacity(),
           (bk.getRegressionCapacity() - C_TRUE) / C_TRUE * 100.0f);
    printf("C (EKF)              : %.1f Ah (n=%u)\n", ks.C_hat, ks.n_updates);
    return 0;
}
