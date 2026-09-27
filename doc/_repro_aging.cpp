// K7: le taux de vieillissement dC_dCycle n'est appris QUE si le process model
// recoit des cycles accumules (delta_cycles != 0) a la mesure.
// Simule applyKalmanUpdate2D avec delta=0 (ancien) vs delta=accumule (corrige).
#include <cstdio>
#include <cmath>
#include <algorithm>
using std::min; using std::max;

static const float P_INIT_C = 500.0f, P_MIN_C = 0.10f;
static const float P_INIT_A = 1e-6f, P_MIN_A = 1e-7f;
static const float Q_A = 2e-6f, Q_C = 0.01f, R = 4.0f;

struct St { float C = 0, a = 0.0005f; float P[2][2] = {{0,0},{0,0}}; bool init = false; };

static void predict(St& k, float dc) {
    if (!k.init) return;
    if (dc < 0) dc = 0;
    k.C -= k.a * dc;
    float F01 = -dc;
    float P00 = k.P[0][0], P01 = k.P[0][1], P10 = k.P[1][0], P11 = k.P[1][1];
    float FP00 = P00 + F01 * P10, FP01 = P01 + F01 * P11, FP10 = P10, FP11 = P11;
    float n00 = FP00 + FP01 * F01, n01 = FP01, n10 = FP10 + FP11 * F01, n11 = FP11;
    k.P[0][0] = min(n00 + Q_A * dc + Q_C, P_INIT_C);
    k.P[0][1] = n01; k.P[1][0] = n10;
    k.P[1][1] = min(n11 + Q_A, P_INIT_A);
}

static void update(St& k, float Cm, float dc) {
    if (!k.init) { k.C = Cm; k.a = 0.0005f; k.P[0][0] = R; k.P[1][1] = Q_A; k.init = true; return; }
    predict(k, dc);
    float innov = Cm - k.C;
    float S = k.P[0][0] + R; if (S <= 0) S = R;
    float KC = k.P[0][0] / S, Ka = k.P[1][0] / S;
    k.C += KC * innov; k.a += Ka * innov;
    float n00 = (1 - KC) * k.P[0][0], n01 = (1 - KC) * k.P[0][1];
    float n10 = k.P[1][0] - Ka * k.P[0][0], n11 = k.P[1][1] - Ka * k.P[0][1];
    k.P[0][0] = max(P_MIN_C, n00); k.P[0][1] = n01; k.P[1][0] = n10; k.P[1][1] = max(P_MIN_A, n11);
}

int main() {
    St before, after;
    // init (1re mesure), puis 5 mesures indiquant une capacite plus basse (vieillissement)
    update(before, 100.0f, 0); update(after, 100.0f, 0);
    for (int i = 0; i < 5; i++) {
        float Cm = 100.0f - 0.5f * (i + 1);
        update(before, Cm, 0.0f);   // ancien: delta_cycles ~ 0
        update(after,  Cm, 1.0f);   // corrige: 1 cycle accumule
    }
    printf("dC_dCycle apres 5 mesures :\n");
    printf("  AVANT (delta=0)  : a=%.8f  P[1][0]=%.3e  (a INCHANGE -> vieillissement non appris)\n",
           before.a, before.P[1][0]);
    printf("  APRES (delta=1)  : a=%.8f  P[1][0]=%.3e  (a EVOLUE)\n",
           after.a, after.P[1][0]);
    printf("  => %s\n", (fabsf(after.a - 0.0005f) > 1e-6f) ? "OK: accumulation necessaire" : "A REVOIR");
    return 0;
}
