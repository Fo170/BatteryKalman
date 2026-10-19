#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <algorithm>

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

inline float constrain(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
inline int   constrain(int x, int a, int b)       { return x < a ? a : (x > b ? b : x); }
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }

// Mock minimal de Serial : nécessaire pour compiler CoulombsAh (`Coulomb.h`,
// printDebug) contre la vraie librairie BatteryModels/CoulombsAh en -fsyntax-only.
struct _StubSerial {
    void begin(unsigned long) {}
    template <class... A> void print(A&&...) {}
    template <class... A> void println(A&&...) {}
    int  printf(const char*, ...) { return 0; }
};
static _StubSerial Serial;
