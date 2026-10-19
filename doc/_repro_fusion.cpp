// Reproduction autonome (g++) : fusion SoC de fuseSoC() — « le coulomb est la
// référence » (v1.7.2, 02/10/2026).
//
// Vérifie, pour chaque ChargeState, que SoC_fused suit bien la table d'alpha :
//   FLOAT 0.00 · REST_LONG 0.02 · REST 0.10 · BULK/ABSORPTION/DISCHARGE/UNKNOWN 1.00
// et — c'est LE point du correctif — que le résultat ne dépend PLUS de
// kalman->confidence (l'ancien `alpha_base * conf` en PHASE_COARSE faisait
// sauter le SoC d'une dizaine de % quand la confiance remontait).
//
// AVANT (v1.7.1) : BULK -> 0.95*80 + 0.05*94 = 80.70 (et ~92 en COARSE)
// APRES (v1.7.2) : BULK -> 80.00 (exactement SoC_coulomb)
//
// Compilation :
//   g++ -std=c++11 -I doc/_repro_mocks -I src -o doc/_repro_fusion.exe doc/_repro_fusion.cpp
#include "Arduino.h"
#include "BatteryModels.h"
#include "Coulomb.h"
#include "BatteryKalman.h"

unsigned long g_millis = 0;

static SoCData soc; static KalmanState2D ks; static BatteryModel model; static Coulomb coul;
static BatteryKalman bk(&soc, &ks, &model, &coul);

static int g_fail = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  [KO] %s\n", msg); g_fail++; } \
    else { printf("  [OK] %s\n", msg); } \
} while (0)

// SoC_coulomb imposé, tension d'OCV imposée (via le mock `ocvOut`, pack).
static const float SOC_COUL = 80.0f;
static const float OCV_PACK = 12.625f;   // -> ocvToSoc = 94.0 % dans la table du mock

static float fusedFor(ChargeState st, float confidence) {
    model.state = st;
    ks.confidence = confidence;
    // Amorçage : 2 échantillons pour absorber les transitions
    // (doSync FLOAT, promotion REST->REST_LONG, startNewSegment).
    for (int i = 0; i < 2; i++) {
        soc.SoC_coulomb = SOC_COUL;
        bk.update(13.0f, 0.0f, 25.0f);
    }
    soc.SoC_coulomb = SOC_COUL;   // neutralise tout doSync() de l'amorçage
    bk.update(13.0f, 0.0f, 25.0f);
    return bk.getSoC();
}

int main() {
    model.useAuto = false;        // état forcé par le test
    model.ocvOut  = OCV_PACK;
    model.minV    = 10.0f;        // évite le cutoff basse tension
    ks.initialized = true;        // sinon fuseSoC() renvoie le SoC brut (Bootstrap)
    ks.confidence  = 0.50f;       // phase != BOOTSTRAP
    bk.begin();

    const float socV = model.ocvToSoc(OCV_PACK, 25.0f);
    printf("SoC_coulomb = %.2f %%   SoC_voltage = %.2f %%\n\n", SOC_COUL, socV);

    struct { ChargeState st; const char* name; float alpha; } cases[] = {
        { State_FLOAT,      "FLOAT",      0.00f },
        { State_REST_LONG,  "REST_LONG",  0.02f },
        { State_REST,       "REST",       0.10f },
        { State_BULK,       "BULK",       1.00f },
        { State_ABSORPTION, "ABSORPTION", 1.00f },
        { State_DISCHARGE,  "DISCHARGE",  1.00f },
        { State_UNKNOWN,    "UNKNOWN",    1.00f },
    };

    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        float got     = fusedFor(cases[i].st, 0.50f);
        float expect  = cases[i].alpha * SOC_COUL + (1.0f - cases[i].alpha) * socV;
        printf("%-11s alpha=%.2f  attendu %6.2f  obtenu %6.2f\n",
               cases[i].name, cases[i].alpha, expect, got);
        char msg[96];
        snprintf(msg, sizeof(msg), "%s : alpha=%.2f respecté", cases[i].name, cases[i].alpha);
        CHECK(fabsf(got - expect) < 0.05f, msg);
    }

    printf("\nIndépendance vis-à-vis de la confiance (le correctif) :\n");
    {
        const ChargeState sts[] = { State_BULK, State_DISCHARGE, State_ABSORPTION };
        for (unsigned i = 0; i < 3; i++) {
            float lo = fusedFor(sts[i], 0.05f);   // ex-PHASE_COARSE (conf < 0.30)
            float hi = fusedFor(sts[i], 0.99f);   // ex-PHASE_TRACK
            printf("  conf 0.05 -> %6.2f   conf 0.99 -> %6.2f\n", lo, hi);
            CHECK(fabsf(lo - hi) < 0.01f, "résultat identique quelle que soit la confiance");
        }
    }

    printf("\nCharge/décharge = SoC_coulomb exactement (avant : 0.95/0.97) :\n");
    {
        float b = fusedFor(State_BULK, 0.99f);
        float d = fusedFor(State_DISCHARGE, 0.99f);
        printf("  BULK %.4f   DISCHARGE %.4f   (SoC_coulomb %.4f)\n", b, d, SOC_COUL);
        CHECK(fabsf(b - SOC_COUL) < 1e-3f, "BULK : SoC_fused == SoC_coulomb");
        CHECK(fabsf(d - SOC_COUL) < 1e-3f, "DISCHARGE : SoC_fused == SoC_coulomb");
    }

    printf("\n=== %s (%d echec(s)) ===\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
