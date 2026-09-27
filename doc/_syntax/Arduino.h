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
