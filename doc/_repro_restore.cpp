// Reproductions autonomes (g++) pour le cycle bug-hunt 2 :
//   K8 — begin() doit toujours synchroniser last_Ah (restauration)
//   K9 — cutoff basse tension : porte temporelle (plus de doSync par échantillon)
// Compilation :
//   g++ -std=c++11 -I doc/_repro_mocks -I src -o doc/_repro_restore.exe doc/_repro_restore.cpp
#include "Arduino.h"
#include "BatteryModels.h"
#include "Coulomb.h"
#include "BatteryKalman.h"

unsigned long g_millis = 0;   // horloge simulée (voir mocks/Arduino.h)

// --- K9 : logique isolée de la branche cutoff basse tension (avant/après) ---
template <bool GATED>
static int lowv_syncs(int n, unsigned long dt_ms) {
    unsigned long last_sync = 0, now = 0;
    int syncs = 0;
    for (int i = 0; i < n; i++) {
        now += dt_ms;
        // état = REST, tension < V_min  → condition de cutoff remplie
        bool fire = GATED ? (now - last_sync > 300000UL) : true;
        if (fire) { syncs++; last_sync = now; }
    }
    return syncs;
}

int main() {
    // ---------------- K8 ----------------
    {
        SoCData      soc;
        KalmanState2D ks;
        Coulomb      coulomb;
        BatteryModel model;
        soc.coulomb_initialized = true;   // état restauré
        soc.SoC_coulomb = 60.0f;
        soc.SoC_voltage = 60.0f;
        coulomb.ah = 40.0f;               // compteur coulomb restauré non nul
        model.state = State_DISCHARGE;
        model.ocvOut = 12.0f;             // pas de cutoff, pas de repos

        BatteryKalman bk(&soc, &ks, &model, &coulomb);
        g_millis = 1000;
        bk.begin();
        float before = soc.SoC_coulomb;
        bk.update(12.0f, 0.0f, 25.0f);    // I = 0 → dAh attendu = 0
        float after = soc.SoC_coulomb;

        printf("K8 begin()/last_Ah  : SoC_coulomb avant=%.1f apres=%.1f  (AVANT-fix aurait saute a %.1f)\n",
               before, after, before + 40.0f / 100.0f * 100.0f);
        printf("   -> %s\n", (after == before) ? "OK (pas de saut)" : "A REVOIR");
    }

    // ---------------- K9 ----------------
    {
        int n = 60;                        // 60 échantillons à 10 s = 10 min
        int before = lowv_syncs<false>(n, 10000UL);
        int after  = lowv_syncs<true>(n, 10000UL);
        printf("K9 cutoff basse V   : resets AVANT=%d (chaque ech.)  APRES=%d  -> %s\n",
               before, after, (after < before / 10 && after >= 1) ? "OK (borne)" : "A REVOIR");
    }

    // ---------------- K9 bout-en-bout (vrai header) ----------------
    {
        SoCData       soc;
        KalmanState2D ks;
        Coulomb       coulomb;
        BatteryModel  model;
        model.state  = State_REST;
        model.ocvOut = 8.0f;               // < V_min → cutoff basse tension
        model.minV   = 10.0f;
        BatteryKalman bk(&soc, &ks, &model, &coulomb);
        g_millis = 0;
        bk.begin();
        for (int i = 0; i < 60; i++) { g_millis += 10000UL; bk.update(8.0f, 0.0f, 25.0f); }
        printf("K9 e2e (header)     : resets coulomb=%d sur 60 ech. -> %s\n",
               coulomb.resetCount, (coulomb.resetCount <= 2) ? "OK" : "A REVOIR");
    }
    return 0;
}
