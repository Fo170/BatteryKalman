# AGENTS.md — BatteryKalman

Guide for AI coding agents working in this repository. Keep it accurate and concise.

---

## 1. Project Overview

**BatteryKalman** is a header-only **Arduino / PlatformIO C++ library** implementing a
**2D Extended Kalman Filter (EKF)** for battery state estimation. It simultaneously
estimates:

- **Capacity `C`** (Ah)
- **Aging rate `dC/dcycle`** (Ah/cycle, learned online)
- **State of Charge (SoC)** — fusion of coulomb-counting and voltage/OCV

- **Language:** C++ (Arduino framework, C++11+), single header.
- **Current version:** `1.6.0` (see `library.json`). Older docs still say `1.5.3` — the
  changelog is authoritative.
- **License:** GPL-3.0-only.
- **Repo:** https://github.com/Fo170/BatteryKalman

### Architecture

```
BatteryKalman (this lib, src/BatteryKalman.h)
   ├── depends on BatteryModel  (abstract interface)   → provided by BatteryModels v1.4+
   ├── depends on Coulomb       (Ah integration)        → provided by CoulombsAh v1.1+
   └── persists SoCData + KalmanState2D  (user-implemented, e.g. Fo170/Persistance)
```

`BatteryKalman` is **technology-agnostic**. All chemistry-specific parameters (OCV tables,
resistances, charge-state detection, per-technology Kalman tuning) come from a
`BatteryModel*`. **BatteryModels v1.4+** is the recommended implementation (provides
`getKalmanTuning()`), but any class implementing the interface works.

The core algorithm is the EKF in `src/BatteryKalman.h`. Everything else is docs, examples,
and Python reference ports used for validation.

---

## 2. Directory Structure

```
.
├── src/
│   └── BatteryKalman.h          # THE library. Header-only. All logic lives here.
├── Exemples/                    # Arduino sketches (.ino) — compile targets / usage reference
│   ├── Utilisation_de_base/     # Minimal example
│   └── avec_persistance/        # LittleFS + ArduinoJson persistence example
├── doc/                         # Analysis & validation (not shipped code)
│   ├── comparaison_kalman.md    # KF/EKF theory, battery variants (AEKF/DEKF/UKF), comparison with this filter
│   ├── rest_long_tuning.md      # REST_LONG tuning per technology + adaptive flatness threshold (R6/R7)
│   ├── correction_BatteryModels.md  # F7 fix for BatteryModels.detectChargeState()
│   ├── _repro_*.cpp             # Standalone g++ reproductions (K1–K12, R1–R7, P1/P2 proofs)
│   ├── _repro_mocks/            # Functional Arduino/BatteryModels/Coulomb mocks for the repros
│   └── _syntax/                 # Stub headers for a dependency-free g++ -fsyntax-only check
├── README.md                    # User-facing: intro, install, deps, usage, EKF theory, tuning, compat, migration, v1.5 history
├── CHANGELOG.md                 # AUTHORITATIVE version history + bug-hunt journal (F1–F8, K1–K12, R1–R7, P1/P2)
├── analyse.md                   # bug-hunt: analysis context (variables, API, invariants)
├── library.json                 # PlatformIO manifest (name, version, deps)
└── LICENSE
```

Notes:
- No `platformio.ini`, no `Makefile`, no CI (`.github/`), no `test/` directory.
- Docs are primarily in **French**. Code comments are French too. Match that style when
  editing existing files.
- `doc/` is analysis material, not library code; only edit if the task is about analysis.

---

## 3. Build, Lint, Test Commands

**There is no configured build system, linter, or test suite in this repo.** Do not invent
`npm`/`make`/`pytest` commands. Verification is done by compiling examples and/or running the
Python validation ports.

### Compile the library (real check)

The library has external dependencies (`BatteryModels`, `CoulombsAh`), so a bare compile of
`src/BatteryKalman.h` will fail without them. Compile an example sketch instead:

```bash
# Arduino CLI (deps are resolved from library.json)
arduino-cli compile --fqbn esp32:esp32:esp32 Exemples/Utilisation_de_base
arduino-cli compile --fqbn esp32:esp32:esp32 Exemples/avec_persistance

# PlatformIO (create a throwaway platformio.ini first if needed)
pio run
```

If `arduino-cli`/`pio` are unavailable, at minimum do a syntax-level review — there is no
substitute build in this repo.

### Validation / "tests"

The de-facto test harness is the set of standalone C++ reproductions in `doc/`, run with
`g++` (no Arduino toolchain needed):

```bash
g++ -std=c++11 -I doc/_syntax -I src -fsyntax-only doc/_syntax/test_bk.cpp   # compile check
g++ -O0 -o /tmp/r doc/_repro_ring.cpp && /tmp/r                 # K1
g++ -O0 -o /tmp/r doc/_repro_restlong.cpp && /tmp/r             # K2/K3
g++ -O0 -o /tmp/r doc/_repro_window.cpp && /tmp/r               # R1/R3
g++ -O0 -o /tmp/r doc/_repro_noise.cpp && /tmp/r                # R6
g++ -O0 -o /tmp/r doc/_repro_aging.cpp && /tmp/r                # K7
g++ -std=c++11 -I doc/_repro_mocks -I src -o /tmp/r doc/_repro_restore.cpp && /tmp/r   # K8/K9
g++ -O0 -o /tmp/r doc/_repro_wrap.cpp && /tmp/r                 # K11
```

`doc/_syntax/` holds dependency-free stub headers; `doc/_repro_mocks/` holds functional
mocks (Arduino clock, Coulomb counter, BatteryModel) used by `_repro_restore.cpp`. Each
reproduction prints an `AVANT`/`APRES` (before/after) verdict and is the ground truth for
the corresponding fix (K/R ids). `doc/rest_long_tuning.md` documents the R6/R7 rationale.

### Lint / format

None configured. Follow surrounding style; do not add a linter config unless asked.

---

## 4. Code Conventions

- **Single header, header-only.** All classes/structs/functions are in `src/BatteryKalman.h`.
  No `.cpp` files.
- **Include guard:** `#ifndef BATTERY_KALMAN_H`.
- **Formatting:** 4-space indent; opening brace on same line for control flow, on next line
  for function/class definitions (Arduino/`clang-format`-ish style already used).
- **Naming:**
  - Classes / structs: `PascalCase` (`BatteryKalman`, `SoCData`, `KalmanState2D`, `KalmanTuning`).
  - Methods / functions: `camelCase` (`update`, `getEffectiveCapacity`, `applyKalmanUpdate2D`).
  - Member fields: `camelCase`; private tuning/helper members prefixed `_` (`_tuningResolve`,
    `_tuningOverrideSet`); private "legacy" members sometimes unprefixed (`seg`, `cycle`, `phase`).
  - Macros / compile-time config: `UPPER_SNAKE_CASE` (`R_INIT`, `SEG_MIN_DAH`, `KALMAN_P_INIT_C`).
  - Private methods are declared in the `private:` section, public API in `public:`.
- **Comments:** Dense French comments, often tagged with fix IDs (`F1`, `F3`, `F8`) referencing
  `CHANGELOG.md`. Preserve these tags — they are cross-referenced.
- **No comments policy for new code:** this repo DOES use comments heavily (it's the existing
  convention). Follow the file's convention rather than a generic "no comments" rule.
- **Constants:** defaults live in `KALMAN_DEFAULT_TUNING` (v1.5.3 values), NOT in macros.

---

## 5. Key Abstractions

### `class BatteryKalman` (src/BatteryKalman.h)
The filter. Constructed with pointers to external state (no internal allocation):

```cpp
BatteryKalman(SoCData* soc, KalmanState2D* kstate, BatteryModel* model, Coulomb* coulomb);
```

Lifecycle: `begin()` (loads state, initializes coulomb) then `update(V, I, T)` each sample.

- `update(float V, float I, float T)` — **`V` is pack voltage**, `I` is current (A, negative =
  discharge), `T` temperature. Internally divides by `model->getCellCount()` for OCV.
- Feed `coulomb->addMeasurement(I)` **before** `update()` each iteration.
- Key getters: `getSoC()`, `getEffectiveCapacity()`, `getConfidence()`, `getKalmanC()`,
  `getKalmanAgingRate()`, `getChargeState()`, `getLearningPhaseStr()`, `getR_measured()`.
- `isStateDirty()` / `clearStateDirty()` — persistence signalling.
- `saveState()` / `loadState()` are **empty stubs**; users implement persistence (see
  `Exemples/avec_persistance/`). `begin()` calls `loadState()`.

### `struct SoCData`
Persistable SoC state: `SoC_coulomb`, `SoC_voltage`, `SoC_voltage_raw`, `SoC_fused`,
`SoC_uncertainty`, `soc_is_raw`, Ah totals, cycle counters, `last_sync_time`,
`coulomb_initialized`.

### `struct KalmanState2D`
Persistable EKF state: `C_hat`, `dC_dCycle`, `P[2][2]` (covariance), `R_estimated`,
`R_measured`, `n_updates`, `initialized`, `outlier_streak`, `confidence` [0,1],
innovation accumulators. **Renamed from `KalmanState` in v1.5** — a breaking change for
persistence formats.

### `struct KalmanTuning` (defined in BatteryModels)
Per-technology tuning: P init/min, Q (aging/capacity), R init/min/max/smooth, segment
thresholds (`seg_min_dAh_abs`, `seg_min_dAh_pct`, `seg_min_dsoc[3]`), battery-change
thresholds, REST_LONG gate params, confidence params.

**Tuning resolution hierarchy** (highest → lowest), implemented in `_tuningResolve()` and
exposed via `getTuning()` (re-resolved on every call):
1. **Runtime setters** (`setTuning/setP/setQ/setR/setSegmentThresholds/setBatteryChange/setRestLong/setConfidence`)
   → stored in `_tuningOverride`.
2. **`model->getKalmanTuning()`** (BatteryModels v1.4+).
3. **Compile-time macros** (`KALMAN_P_INIT_C`, `R_INIT`, `SEG_MIN_DAH`, …) — optional.
4. **`KALMAN_DEFAULT_TUNING`** (v1.5.3 values).

`resetTuning()` clears the runtime override. First `setX()` call copies the current resolved
tuning into the override before mutating.

### `enum LearningPhase`
`PHASE_BOOTSTRAP` → `PHASE_COARSE` → `PHASE_REFINE` → `PHASE_TRACK`. Mapped from the
continuous `confidence` in `updatePhaseFromConfidence()`. Legacy/observability only.

### `enum ChargeState` (from BatteryModels)
`State_UNKNOWN, State_DISCHARGE, State_BULK, State_ABSORPTION, State_FLOAT, State_REST,
State_REST_LONG`. Renamed from `MpptState`/`MPPT_*` in BatteryModels v1.2 (see `README.md` § Migration).

### External deps
- **`BatteryModel`** (abstract, from BatteryModels): `detectChargeState`, `correctVoltageToOCV`,
  `ocvToSoc`, `getCellCount`, `getNominalCapacity`, `getMinVoltage/MaxVoltage`,
  `getTechnologyName`, `isAutoDetect`, `isOcvReliable`, and (v1.4+) `getKalmanTuning()`.
- **`Coulomb`** (from CoulombsAh, header `Coulomb.h`): `addMeasurement(I)` integrates I·dt
  internally via `micros()`; `getAmpereHours()`, `getLastInterval()`, `reset(false)`, `begin()`.

---

## 6. Common Pitfalls

1. **`mppt_state_prev` ordering (F1).** It is updated **at the end of `update()`**, never
   inside `updateMpptState()`. Moving it earlier breaks FLOAT sync, segment closure, and
   cycle counting (they test `mppt_state == FLOAT && mppt_state_prev != FLOAT`).

2. **Order of `updateSegmentAndKalman()` vs `handleSyncEvents()` (F2).** Segment measurement
   MUST run **before** sync events (which reset the coulomb counter). Reversing makes `dAh≈0`.

3. **Macros are sentinels, not defaults.** `R_INIT`, `SEG_MIN_DAH`, etc. default to `NAN`
   (or `0` for ints). The "is defined?" test is the odd-looking `if (MACRO == MACRO)`. The real
   defaults live in `KALMAN_DEFAULT_TUNING`. Don't treat these macros as tunable values.

4. **`R_measured` must be kept in sync (F3).** `updateREstimate()` sets `R_estimated`; the
   segment code uses `R_measured`. If you break `R_measured = R_estimated`, adaptive R silently
   stops working.

5. **Segments close only on FLOAT / REST_LONG transitions.** A real measurement needs a full
   charge (FLOAT = SoC 100%) or a long stable rest. Without those, the filter legitimately
   stays in Bootstrap. `ocv_segment_allowed=false` (NiFe/Sodium) disables OCV segments entirely.

6. **`REST_LONG_STABLE_MAX_SAMPLES` is compile-time only** (ring buffer size), not part of
   `KalmanTuning`. Other REST_LONG params are tuning-controlled.

7. **`begin()` calls the `loadState()` stub** — it does nothing by default. Users must implement
   persistence; the library never persists on its own.

8. **`saveState()` is also a stub.** It only clears `state_dirty`.

9. **Version strings disagree across docs.** `library.json` and code say `1.6.0`; several docs
   say `1.5.3`. Trust `CHANGELOG.md` + `library.json`.

10. **Units.** `update()` takes **pack** voltage; OCV is derived per-cell internally. `getRintEff()`
    returns **milliohms** (multiplied by 1000). Coulomb counter is fed externally.

11. **`getEffectiveCapacity()` can return `0.0f`** (Bootstrap, no nominal capacity) — guard
    divisions by it.

12. **Changing `src/BatteryKalman.h` is a global change.** It is the entire library; there is no
    smaller unit to modify. Update `CHANGELOG.md` for behavior changes, and keep the F-tags.

13. **Verification lives in `doc/_repro_*.cpp` (g++), not Python.** The old Python ports
    (buggy v1.5 / fixed v1.5.3) and the real dataset were removed in the doc cleanup. To change
    algorithm behavior, update the relevant `_repro_*.cpp` (K/R ids) so the before/after proof
    still holds, and keep `doc/comparaison_kalman.md` (theory + variants) in sync.

14. **The REST stability ring is time-decimated (K1/R1/R4).** Writes are gated by
    `rest_v_has_push` / `rest_v_last_push_ms` with `push_ms = stable_ms / REST_LONG_STABLE_MAX_SAMPLES`,
    and `rest_v_count` is a wrapping cursor. Do NOT cap `rest_v_count` or drop the decimation —
    the window collapses to ~32 raw samples (~3 s at 10 Hz) instead of the configured duration.

15. **`state_entry_ms` re-arms on the RAW charge state (K2).** It uses `mppt_raw_prev` (raw
    `detectChargeState()` result), NOT `mppt_state_prev` (which holds the promoted
    `State_REST_LONG`). Using the final state makes REST_LONG flicker every `rest_long_min_min`.

16. **`dC_dCycle` default is `+0.0005f` (K3).** Positive = capacity loss, consistent with
    `C_hat -= dC_dCycle * delta_cycles` and `F = [[1,-Δ],[0,1]]`. Do not set it negative.

17. **Bug-hunt artifacts.** `analyse.md` (root) and the **Annexe K/R of `CHANGELOG.md`** document
    the K1–K12/R1–R7 analysis; `doc/_repro_*.cpp` are standalone g++ reproductions; `doc/_syntax/`
    holds stub headers for a dependency-free `g++ -fsyntax-only` compile check of the library. The
    EKF math is documented in `README.md` and cross-checked against 4 sources.

18. **REST_LONG tuning is auto-adaptive (R6/R7).** `applyRecommendedRestLong()` (public API) sets
    chemistry-specific `rest_long_min_min/stable_min/stable_mv` via the runtime setter (level 1,
    so it overrides BatteryModels). The flatness threshold is also noise-adaptive:
    `stable_mv_eff = max(rest_long_stable_mv, REST_LONG_NOISE_K × rest_v_noise)` capped at 3× base,
    where `rest_v_noise` tracks `|V - V_prev_sample|` at rest. Full rationale + per-technology
    table: `doc/rest_long_tuning.md`. Do not reintroduce a fixed threshold without the cap.

19. **Sampling-rate floor.** With `min_samples = 6`, the REST_LONG gate needs
    `dt ≤ rest_long_stable_min/(min_samples-1)` (e.g. 30 min → ≤ 6 min). Slower sampling requires
    lowering `rest_long_stable_min_samples`.

20. **Aging requires accumulated cycles (K7).** `predictKalman()` must receive the cycles elapsed
    since the LAST MEASUREMENT, not the per-sample increment. `update()` accumulates into
    `cycles_since_update` and `updateSegmentAndKalman()` resets it after `applyKalmanUpdate2D()`.
    If you pass the per-sample increment (≈0 at a segment close), `F01 = -Δ ≈ 0`, no cross-covariance
    `P[1][0]` is generated, and `dC_dCycle` is never learned (filter degenerates to 1D).

21. **`begin()` must ALWAYS sync `last_Ah` (K8).** `last_Ah = coulombMeter->getAmpereHours()` runs
    unconditionally (outside the `if (!coulomb_initialized)` block). If it only ran at first init,
    a restored `SoCData` (`coulomb_initialized == true`) with a restored non-zero coulomb counter
    would leave `last_Ah == 0` → the first `update()` injects the whole counter as `dAh` → SoC jump.

22. **Low-voltage cutoff is time-gated (K9).** In `handleSyncEvents()`, the `total_ocv < V_min`
    cutoff only fires when `millis() - last_sync_time > REST_ALIGN_SYNC_MS` (5 min). Without the
    gate it called `doSync()` every sample → permanent `state_dirty` (EEPROM wear) + repeated
    coulomb reset. `REST_ALIGN_SYNC_MS` also gates the REST SoC-alignment branch.

23. **REST ring is wrap-tolerant (K11).** In `updateMpptState()`, if `now_ms < rest_v_last_push_ms`
    (millis wrap ≈49.7 d) the ring is cleared so stale pre-wrap timestamps can't inflate the
    peak-to-peak and defer REST_LONG. `REST_LONG_STABLE_MAX_SAMPLES` is compile-time ONLY: the
    `KalmanTuning::rest_long_stable_max_samples` field is NOT consumed (K10).

24. **Pseudo-rest mode is opt-in (P1).** `enablePseudoRest()` uses brief ~zero-current moments as
    a LOW OCV anchor for installations with no long rest. Only net-DISCHARGE pseudo-rests are
    accepted; pseudo-rests during charge are IGNORED without breaking the segment. Disabled by
    default → v1.6.0 behavior unchanged.

25. **Regression uses net-discharge segments only (P2).** `addRegressionPoint()` is called only
    when `signed_ah < 0`. Charge segments are biased (coulomb efficiency + FLOAT hold where Ah
    accumulate without SoC change). Weighted fit `dAh = C·dSoC/100` through the origin with
    weight `dSoC²` (statistically optimal: Var(C_i) ∝ 1/dSoC²) so DEEP discharges recalibrate
    the estimate. Needs ≥2 points. Observability limits (small DoD, few anchors, relaxation) and
    the deep-discharge recalibration strategy are documented in `README.md`.
