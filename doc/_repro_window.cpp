#include <cstdio>
#include <cstdint>

static const int      MAX = 32;
static const uint32_t STABLE_MS = 900000;    // 15 min (rest_long_stable_min, décision R2)
static const int      MIN_SAMPLES = 6;       // rest_long_stable_min_samples

// Etat de l'anneau (sentinel 0 = case vide, comme dans l'en-tete) au temps T.
template <bool DECIM>
void ring_state(uint32_t T, uint32_t dt, uint32_t& span_out, int& kept_out) {
    uint32_t ring_ms[MAX] = {0};
    uint8_t  count = 0;
    uint32_t last_push = 0;
    bool     has_push = false;
    uint32_t push_ms = STABLE_MS / MAX;      // 28125 ms

    for (uint32_t t = 0; t <= T; t += dt) {
        bool push = DECIM ? (!has_push || (t - last_push) >= push_ms) : true;
        if (push) {
            ring_ms[count % MAX] = (t == 0) ? 1u : t;   // evite le sentinel 0 au boot
            count = (uint8_t)((count + 1) % MAX);
            last_push = t; has_push = true;
        }
    }

    uint32_t cutoff = (T > STABLE_MS) ? (T - STABLE_MS) : 0;
    uint32_t mn = 0xFFFFFFFFu, mx = 0;
    int kept = 0;
    for (int i = 0; i < MAX; i++) {
        if (ring_ms[i] == 0 || ring_ms[i] < cutoff) continue;
        if (ring_ms[i] < mn) mn = ring_ms[i];
        if (ring_ms[i] > mx) mx = ring_ms[i];
        kept++;
    }
    span_out = (kept == 0) ? 0 : (mx - mn);
    kept_out = kept;
}

int main() {
    const uint32_t T = 900000;   // 15 min (fenetre stable pleine)
    struct { const char* rate; uint32_t dt; } cases[] = {
        {"10 Hz", 100}, {"1 Hz", 1000}, {"0.2 Hz (5min)", 300000}, {"0.1 Hz (10min)", 600000}
    };
    printf("Anneau a T=15min  (min_samples=%d)\n", MIN_SAMPLES);
    printf("  %-15s | %-22s | %-22s\n", "cadence", "AVANT (span/kept)", "APRES (span/kept)");
    for (auto& c : cases) {
        uint32_t sb, sa; int kb, ka;
        ring_state<false>(T, c.dt, sb, kb);
        ring_state<true >(T, c.dt, sa, ka);
        printf("  %-15s | %7u ms / %2d %-8s | %7u ms / %2d %-8s\n",
               c.rate, sb, kb, (kb >= MIN_SAMPLES ? "OK" : "kept<6"),
               sa, ka, (ka >= MIN_SAMPLES ? "OK" : "kept<6"));
    }
    printf("\nNote R3: au-dela de dt = STABLE_MS/(min_samples-1) = %u ms, kept < %d -> REST_LONG inatteignable\n",
           STABLE_MS / (MIN_SAMPLES - 1), MIN_SAMPLES);
    return 0;
}
