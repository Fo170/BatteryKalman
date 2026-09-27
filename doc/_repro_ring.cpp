#include <cstdio>
#include <cstdint>

static const int MAX = 32;

// FIXED=false : logique AVANT correction (rest_v_count plafonne a MAX)
// FIXED=true  : logique APRES correction (curseur modulo MAX)
template <bool FIXED>
void run_ring(const char* label) {
    float    ring[MAX] = {0};
    uint32_t ringms[MAX] = {0};
    uint8_t  count = 0;
    uint32_t now = 0;
    bool     touched[MAX] = {false};
    bool     filled = false;

    for (int s = 0; s < 100; s++) {
        now += 100;
        uint8_t idx = count % MAX;
        ring[idx] = (float)s;
        ringms[idx] = now;
        if (filled) touched[idx] = true;

        if (FIXED) {
            count = (uint8_t)((count + 1) % MAX);
        } else {
            if (count < 255) count++;
            if (count > MAX) count = MAX;
        }
        if (s == MAX - 1) filled = true;
    }

    int n = 0;
    for (int i = 0; i < MAX; i++) if (touched[i]) n++;
    printf("%-6s : %2d/%d cases ecrites apres remplissage (attendu %d) -> %s\n",
           label, n, MAX, MAX, (n == MAX) ? "OK" : "BUG");
}

int main() {
    run_ring<false>("AVANT");
    run_ring<true>("APRES");
    return 0;
}
