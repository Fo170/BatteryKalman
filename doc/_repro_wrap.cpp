// Reproduction autonome (g++) : K11 — tolérance au wrap de millis() dans l'anneau REST.
// Compilation : g++ -O0 -o doc/_repro_wrap.exe doc/_repro_wrap.cpp
#include <cstdio>
#include <cstdint>

// Rejoue la purge de l'anneau : fenêtre stable_ms, N cases (ms, V).
// GUARD=false : comportement avant K11 (cutoff = now>stable_ms ? now-stable_ms : 0)
// GUARD=true  : vidage de l'anneau si l'horloge a rebouclé (now < last_push)
template <bool GUARD>
static int ring_kept_after_wrap(uint32_t stable_ms, uint32_t now_pre, uint32_t now_post) {
    const int N = 8;
    uint32_t ms[N]; float v[N];
    for (int i = 0; i < N; i++) { ms[i] = now_pre - (N - 1 - i) * 1000u; v[i] = 12.0f; }
    uint32_t last_push = now_pre;
    bool has_push = true;

    // --- itération post-wrap ---
    if (GUARD && has_push && now_post < last_push) {
        for (int i = 0; i < N; i++) ms[i] = 0;   // vidage
    }
    uint32_t cutoff = (now_post > stable_ms) ? (now_post - stable_ms) : 0u;
    int kept = 0;
    for (int i = 0; i < N; i++) {
        if (ms[i] == 0) continue;
        if (ms[i] < cutoff) continue;
        kept++;
    }
    return kept;
}

int main() {
    const uint32_t stable_ms = 1800000u;         // 30 min
    const uint32_t near_wrap = 0xFFFFFF00u;      // juste avant wrap (~49,7 j)
    const uint32_t post_wrap = 0x00000100u;      // juste après wrap
    int before = ring_kept_after_wrap<false>(stable_ms, near_wrap, post_wrap);
    int after  = ring_kept_after_wrap<true>(stable_ms, near_wrap, post_wrap);
    printf("K11 wrap millis()   : cases conservees AVANT=%d (perimees)  APRES=%d (anneau vide) -> %s\n",
           before, after, (before > 0 && after == 0) ? "OK" : "A REVOIR");
    return 0;
}
