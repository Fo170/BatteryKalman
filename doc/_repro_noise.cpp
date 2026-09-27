// R6: seuil de planéité fixe vs auto-adaptatif au bruit de mesure.
// Simulation d'une tension PLATE (batterie au repos) + bruit ADC.
#include <cstdio>
#include <cmath>
#include <cstdint>

static const int   MAX = 32;        // REST_LONG_STABLE_MAX_SAMPLES
static const float BASE_MV = 20.0f; // rest_long_stable_mv
static const float NOISE_K = 5.0f;  // REST_LONG_NOISE_K

static uint32_t s = 12345;
static float rnd() { s = s * 1664525u + 1013904223u; return ((s >> 8) & 0xFFFFFF) / 16777216.0f; }
static float gauss() { float u1 = rnd() > 1e-6f ? rnd() : 1e-6f; float u2 = rnd();
                       return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2); }

static void trial(float sigma_mv) {
    float ring[MAX], prev = 0.0f, noise = 0.0f;
    for (int i = 0; i < MAX; i++) {
        float v = 12800.0f + sigma_mv * gauss();
        ring[i] = v;
        if (i > 0) { float d = fabsf(v - prev); noise = (noise <= 0.0f) ? d : 0.95f * noise + 0.05f * d; }
        prev = v;
    }
    float mn = 1e9f, mx = -1e9f;
    for (int i = 0; i < MAX; i++) { if (ring[i] < mn) mn = ring[i]; if (ring[i] > mx) mx = ring[i]; }
    float ptp = mx - mn;

    float thr_fixed = BASE_MV;
    float thr_adapt = BASE_MV;
    float nf = NOISE_K * noise;
    if (nf > thr_adapt) thr_adapt = (nf < 3.0f * BASE_MV) ? nf : 3.0f * BASE_MV;

    printf("  bruit sigma=%.1f mV | ptp=%5.1f | fixe=%4.1f -> %-7s | adapt=%4.1f -> %s\n",
           sigma_mv, ptp, thr_fixed, (ptp <= thr_fixed ? "ACCEPTE" : "REJETTE"),
           thr_adapt, (ptp <= thr_adapt ? "ACCEPTE" : "REJETTE"));
}

int main() {
    printf("R6: tension PLATE (batterie relaxee) + bruit capteur\n");
    printf("  (fixe = rest_long_stable_mv; adapt = max(fixe, 5*|dV|), borne 3x)\n");
    trial(2.0f);
    trial(6.0f);
    trial(10.0f);
    printf("  => a bruit eleve, le seuil FIXE rejette une batterie pourtant relaxee ;\n");
    printf("     le seuil ADAPTATIF (R6) l'accepte.\n");
    return 0;
}
