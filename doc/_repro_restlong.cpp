#include <cstdio>
#include <cstdint>

enum { State_UNKNOWN=0, State_DISCHARGE=1, State_BULK=2, State_ABSORPTION=3,
       State_FLOAT=4, State_REST=5, State_REST_LONG=6 };

// FIXED=false : state_entry_ms rearme sur l'etat FINAL (mppt_state_prev)
// FIXED=true  : state_entry_ms rearme sur l'etat BRUT (mppt_raw_prev)
template <bool FIXED>
int rest_long_promotions() {
    int      mppt = State_UNKNOWN, prev = State_UNKNOWN, raw_prev = State_UNKNOWN;
    uint32_t entry = 0;
    uint32_t min_ms = 900000;   // 15 min
    int      promos = 0;              // transitions REST -> REST_LONG
    uint32_t now = 0;

    for (int i = 0; i < 60; i++) {
        now += 60000;                 // 1 echantillon / minute
        int raw = State_REST;         // detectChargeState() renvoie toujours REST
        mppt = raw;
        int cmp = FIXED ? raw_prev : prev;
        if (raw != cmp) entry = now;
        if (FIXED) raw_prev = raw;
        if (now - entry >= min_ms) mppt = State_REST_LONG;
        if (mppt == State_REST_LONG && prev != State_REST_LONG) promos++;
        prev = mppt;                  // fin de update()
    }
    return promos;
}

int main() {
    int before = rest_long_promotions<false>();
    int after  = rest_long_promotions<true>();
    printf("REST_LONG atteint sur 60 min : AVANT=%d (clignotant)  APRES=%d (stable) -> %s\n",
           before, after, (after == 1) ? "OK" : "A REVOIR");

    // K3 : signe du taux de vieillissement par defaut
    float C_before = 50.0f, a_before = -0.0005f;
    float C_after  = 50.0f, a_after  =  0.0005f;
    for (int k = 0; k < 3; k++) { C_before -= a_before * 1.0f; C_after -= a_after * 1.0f; }
    printf("Capacite apres 3 cycles 'vieillissement' : AVANT=%.5f Ah (monte)  APRES=%.5f Ah (baisse) -> %s\n",
           C_before, C_after, (C_after < 50.0f) ? "OK" : "A REVOIR");
    return 0;
}
