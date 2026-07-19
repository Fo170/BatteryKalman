# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**BatteryKalman** is an Arduino library implementing an **Extended Kalman Filter (EKF 2D)** for battery capacity estimation, aging tracking, and state-of-charge (SoC) determination. It is technology-agnostic and uses an abstract `BatteryModel` interface for battery-specific parameters.

**Version**: 1.5 (July 2026)  
**Status**: Production-ready with full theoretical Kalman implementation  
**Licence**: GNU General Public License v3

**Key Features (v1.5)**:
- ✓ **2D State Space**: Simultaneously estimates capacity (C) and aging rate (dC/dcycle)
- ✓ **Process Noise Q**: Realistic uncertainty growth between measurements
- ✓ **Extended Kalman Filter**: Proper EKF with Jacobian, not simplified scalar filter
- ✓ **Online R Estimation**: Measurement noise adapts to real conditions
- ✓ **Continuous Confidence**: Data-driven metric [0,1] replaces discrete phases
- ✓ **Automatic Aging Learning**: No hardcoded drift rates; learned from measurements

**Interface Required**:
- `BatteryModel` abstract class (any compatible implementation)
- Recommended: [BatteryModels v1.2.0+](https://github.com/Fo170/BatteryModels)
- Alternative: Your own BatteryModel implementation

**Compatibility**:
- Any `BatteryModel` implementation (required interface)
- [BatteryModels v1.2.0+](https://github.com/Fo170/BatteryModels) (recommended, fully compatible)
- Coulomb library (optional, for Ah integration)
- Arduino framework (ESP32, ESP8266, STM32, Teensy, ATmega)
- C++11 minimum

## Architecture

### Core Design

- **Single header file library**: All implementation is in `src/BatteryKalman.h` (included directly, not compiled)
- **Language**: C++ with Arduino framework compatibility
- **Dependencies**: 
  - `BatteryModels` library (for battery technology models, OCV curves, MPPT detection)
  - `Coulomb` library (for coulomb counting / Ah tracking)
  - Arduino standard libraries (Arduino.h, math.h)

### Main Components

#### 1. **Data Structures**
- `SoCData`: Persistent battery state including SoC estimates (coulomb/voltage/fused), accumulated Ah, cycle counts, and depth-of-discharge metrics
- `KalmanState`: Persistent Kalman filter state including estimated capacity (C_hat), variance (P), update count, and outlier tracking

#### 2. **Learning Phases**
The library progresses through 4 phases with increasing precision:
- **Phase 0 (Bootstrap)**: Capacity unknown, SoC based on raw OCV table, ±15-20% accuracy
- **Phase 1 (Convergence)**: First rough estimate, ±10-15% accuracy, up to 5 updates
- **Phase 2 (Refinement)**: Windowed estimates for stable convergence, ±5-8% accuracy, up to 15 updates
- **Phase 3 (Tracking)**: Maximum precision with aging drift tracking, ±3-5% accuracy

Phases are automatically managed; phase transitions based on Kalman variance (P) and update count thresholds.

#### 3. **Core Algorithm Flow** (in `update()` method)
1. Reads voltage (V), current (I), temperature (T) from external sensors via Coulomb meter
2. Computes derivatives (dV/dt) for state detection
3. Corrects voltage to OCV using BatteryModel resistance model
4. Updates SoC from voltage via OCV lookup curve
5. Detects MPPT state (Float, Bulk, Absorption, Rest, Rest_Long, Discharge, Unknown)
6. Accumulates Coulomb-counted Ah and applies to SoC_coulomb
7. Triggers synchronization events (Full/Empty/Long-Rest detected via MPPT states)
8. Fuses SoC_coulomb and SoC_voltage with phase-dependent alpha weighting
9. Segments discharge/charge cycles and applies Kalman updates when conditions met
10. Detects full/partial cycles and aging drift (Phase 3 only)

#### 4. **Kalman Update Strategy**
- **Measurement model**: Capacity estimates derived from observed ΔAh / ΔSoC during segments
- **Outlier handling**: >3σ deviations trigger streak counter; repeated outliers may indicate battery replacement
- **Battery replacement detection** (Phase 3): Automatic reversion to Phase 1 learning with reinitialized P variance
- **Windowing** (Phase 2): Multiple measurements weighted by reciprocal variance before update to reduce noise

#### 5. **Synchronization (Anchor) Events**
Automatic SoC corrections triggered by:
- **Float state**: Charge complete → SoC = 100%
- **Long rest** (>2h): OCV reliable after voltage stabilization → sync to voltage-based SoC
- **Low voltage cutoff**: Discharge floor → SoC = 0%
- **Short rest**: Soft drift alignment (10% voltage, 90% coulomb)

#### 6. **Configurable Parameters** (via #ifndef macros at top of .h)
Key tuning constants:
- `KALMAN_P_INIT` / `KALMAN_P_MIN`: Initial and minimum variance
- `PHASE1_P_EXIT`, `PHASE1_MAX_UPDATES` etc.: Phase transition thresholds
- `R_BASE_PHASE1/2/3`: Measurement noise level per phase
- `PHASE2_WINDOW_SIZE`: Number of measurements to window in Phase 2
- `BATTERY_CHANGE_THR` / `BATTERY_CHANGE_COUNT`: Outlier detection sensitivity
- `SEG_MIN_DAH` / `SEG_MIN_DSOC_P*`: Minimum segment requirements per phase
- `AGING_DRIFT_PER_CYCLE`: Fractional capacity loss per partial cycle

Override any via `#define` before including the header.

### 7. **2D State Space & EKF**

**State Vector:**
```
x = [C_hat, dC_dCycle]^T

C_hat           : Estimated capacity (Ah)
dC_dCycle       : Aging rate (Ah/cycle) — learned automatically
```

**Process Model:**
```
C_new = C_old - dC_dCycle * delta_cycles   (capacity ages)
dC_new = dC_old                             (rate stable)
```

**Process Noise (Q):**
```
Q = [[Q_capacity, 0], [0, Q_aging]]
Q_capacity = Q_AGING_UNCERTAINTY * delta_cycles
Q_aging    = Q_AGING_UNCERTAINTY
```
Grows P between measurements; prevents overconfidence.

**Measurement Model:**
```
z = C_measured = dAh / (dSoC / 100)

H = [1, 0]  (measure C only, not aging rate directly)
```

**Kalman Gain & Update:**
```
S = H*P*H' + R = P[0][0] + R
K = [P[0][0] / S, 0]^T
x_new = x + K * (z - H*x)
P_new = (I - K*H) * P
```

**Benefits Over 1D:**
- ✓ Aging rate learned from data (not hardcoded 0.05%/cycle)
- ✓ Cross-covariance P[0][1] captures C ↔ aging interaction
- ✓ Process noise Q properly models uncertainty growth
- ✓ Extensible (add temperature, SOH, etc. as states)

### State Persistence

The library does **not** persist state directly; the user must implement:
- **Save/load**: Implement `saveState()` and `loadState()` methods (updated for `KalmanState2D`)
- **Trigger**: Call `saveState()` only when `isStateDirty()` returns true
- **Storage backend**: LittleFS (ESP32/ESP8266), EEPROM, Preferences, SD card, etc.
- **New fields (v3.0.0)**: Save `C_hat`, `dC_dCycle`, `P[2][2]`, `confidence`

Examples show LittleFS + ArduinoJson pattern; see `Exemples/avec_persistance/` for complete implementation with v3.0.0 structures.

## Common Development Tasks

### Building/Testing
This is a header-only library; no compilation step. Integration testing is done in Arduino sketches:
- **Basic example**: `Exemples/Utilisation_de_base/Utilisation_de_base.ino` — minimal 10Hz update loop
- **With persistence**: `Exemples/avec_persistance/complet_avec_persistance.ino` — shows LittleFS + JSON save/load

To test:
1. Copy `src/BatteryKalman.h` to your project's `src/` or Arduino libraries folder
2. Ensure `BatteryModels.h` and `Coulomb.h` are available (same location or via PlatformIO)
3. Compile and upload a sketch that instantiates `BatteryKalman` with your sensor inputs

### Tuning the Filter

**To speed up capacity convergence** (aggressive learning):
- Lower `KALMAN_P_INIT` (e.g., 200 instead of 500)
- Lower `PHASE1_P_EXIT` / `PHASE2_P_EXIT` (tighter variance target)
- Increase `R_BASE_PHASE1` / `R_BASE_PHASE2` (trust measurements more)

**To stabilize against noisy sensors**:
- Raise `R_BASE_PHASE*` values (distrust measurements)
- Increase `SEG_MIN_DAH` / `SEG_MIN_DSOC_*` (require larger segments before Kalman update)
- Increase `PHASE2_WINDOW_SIZE` (average more measurements)

**To detect battery replacement faster**:
- Lower `BATTERY_CHANGE_THR` (smaller % deviation triggers suspicion)
- Lower `BATTERY_CHANGE_COUNT` (fewer outliers needed to confirm)

### Integration with BatteryModels

BatteryKalman **depends on and extends** [BatteryModels](https://github.com/Fo170/BatteryModels):

**BatteryModels provides:**
- OCV lookup tables (per battery chemistry)
- Internal resistance models
- Thermal compensation (Arrhenius model)
- Charge state detection (BULK, ABSORPTION, FLOAT, REST, DISCHARGE)
- Peukert effect correction

**BatteryKalman adds:**
- Extended Kalman Filter for capacity estimation
- Aging rate tracking (dC/dcycle learned)
- SoC fusion (coulomb-counting + voltage)
- Cycle counting and battery health monitoring

**Example initialization:**
```cpp
#include <BatteryModels.h>   // Must include first
#include <BatteryKalman.h>

// 1. Create model from BatteryModels
BatteryModel model(TECH_LIFEPO4, 4, 100.0f);  // LiFePO4, 4S, 100Ah

// 2. Create Kalman state
KalmanState2D kalmanState;

// 3. Create BatteryKalman (uses model internally)
BatteryKalman battery(&socData, &kalmanState, &model, &coulomb);

// 4. In loop: BatteryModels detects charge state,
//    BatteryKalman estimates capacity + SoC
battery.update(voltage, current, temperature);
```

### Integration Points

1. **Sensor input**: Wire voltage/current/temperature ADC to call `battery.update(V, I, T)` ≥10 Hz
2. **BatteryModel instance** (from [BatteryModels](https://github.com/Fo170/BatteryModels)): 
   - Choose technology: `TECH_LIFEPO4`, `TECH_LIION`, `TECH_LEAD`, etc.
   - Specify cell count and nominal capacity
   - BatteryKalman uses this for OCV curves, charge state detection, etc.
3. **Coulomb meter instance**: Provide `Coulomb` object for Ah tracking
4. **Persistence layer**: Serialize `SoCData` + `KalmanState2D` when `isStateDirty()`
5. **Query results**: `getSoC()`, `getEffectiveCapacity()`, `getKalmanAgingRate()`, cycle counts

### Key Methods to Know

**Initialization & Updates:**
- `begin()`: Initialize; calls `loadState()` and resets Coulomb meter
- `update(V, I, T)`: Core update; call ≥10 Hz with fresh measurements (V in volts, I in amps, T in °C)

**Manual Anchors:**
- `syncToFull()` / `syncToEmpty()` / `syncToVoltage()`: Force SoC to known value
- `resetCoulombCount()`: Realign coulomb counter after confirmed SoC event
- `resetLearning()`: Force revert to Phase 0 and reset Kalman state

**Query State (v3.0.0):**
- `getSoC()`: Current SoC estimate (fused coulomb+voltage)
- `getEffectiveCapacity()`: C_hat (learned or nominal)
- `getConfidence()`: Filter confidence [0, 1] (v3.0.0 NEW)
- `getKalmanC()`: Current capacity estimate (v3.0.0 NEW)
- `getKalmanAgingRate()`: Estimated aging rate in Ah/cycle (v3.0.0 NEW)
- `getKalmanP_CC()`: Variance of C (v3.0.0 NEW)
- `getKalmanP_aging()`: Variance of aging rate (v3.0.0 NEW)
- `getKalmanUpdates()`: Number of Kalman updates performed
- `getR_estimated()`: Adaptive measurement noise (v3.0.0 NEW)
- `getLearningPhaseStr()`: Current learning phase name
- `getChargeStateStr()`: Current charge state (Bulk, Float, etc.)

**Persistence:**
- `isStateDirty()`: Check if state changed, needs save
- `clearStateDirty()`: Clear dirty flag after save
- `saveState()`: User-implemented; saves SoCData + KalmanState2D
- `loadState()`: User-implemented; restores state from storage

## Code Style & Conventions

- **Comments**: Mostly French (matching README and examples); implementation comments explain algorithm intent
- **Naming**: Snake_case for struct members, camelCase for methods
- **Constants**: Uppercase with `_` separator; configurable via `#ifndef` guards
- **Math**: Uses `fabsf()`, `sqrtf()`, `min()`, `max()`, `constrain()` from Arduino; no dynamic allocation
- **Embedded focus**: No STL containers; fixed-size arrays and structs only; careful with float precision

## Dependencies & Compatibility

### Required: BatteryModel Interface

BatteryKalman **requires** a class implementing the abstract `BatteryModel` interface with these methods:
```cpp
class BatteryModel {
    ChargeState detectChargeState(float V, float I, float C);
    float correctVoltageToOCV(float V, float I, float T, float dt, float* r_eff, ...);
    float ocvToSoc(float V, float T);
    uint8_t getCellCount();
    float getNominalCapacity();
    float getMinVoltage() / getMaxVoltage();
    const char* getTechnologyName();
    bool isAutoDetect() / isOcvReliable();
    // ... other methods
};
```

### Recommended: [BatteryModels v1.2.0+](https://github.com/Fo170/BatteryModels)

- ✅ **Fully compatible** with BatteryKalman v1.5
- ✅ **Recommended implementation** of BatteryModel interface
- ✅ Provides OCV curves, thermal compensation, charge state detection
- ✅ License: GNU GPLv3
- Alternative: Use your own BatteryModel implementation

### Optional: Coulomb Library

- Provides: Ampere-hour integration with Peukert correction
- Optional: Implement your own coulomb meter if preferred

### Hardware Platforms
- ESP32, ESP8266, STM32, Teensy, ATmega (any with Arduino framework)
- C++11 minimum
- Memory: ~850 bytes for library objects + SoCData + KalmanState2D

### Installation Checklist
```
✓ Arduino IDE 1.8.13+ or PlatformIO
✓ BatteryModel implementation (BatteryModels v1.2.0+ recommended)
✓ Coulomb library (optional, or implement your own)
✓ Target board with Arduino framework support
```

## API Compatibility Notes (BatteryModels v1.2.0 Update)

**Breaking changes in BatteryModels v1.2.0** required updates to BatteryKalman:
- **Method renamed**: `model->detectMpptState()` → `model->detectChargeState(V, I, capacity)`
- **Enum renamed**: `MpptState` → `ChargeState` (from BatteryModels)
- **Enum values renamed**: `MPPT_*` → `State_*` (e.g., `MPPT_FLOAT` → `State_FLOAT`)
- **Getter renamed**: `getMpptStateStr()` → `getChargeStateStr()` (returns charge state description)
- **New method**: `getChargeState()` returns current `ChargeState` (replaces implicit `getMpptState()`)

These changes reflect BatteryModels' shift from MPPT (Maximum Power Point Tracking) terminology to more general charge state terminology.

## GitHub Repository

**Official**: [https://github.com/Fo170/BatteryKalman](https://github.com/Fo170/BatteryKalman)

- **Main Branch**: Latest stable release (v1.5)
- **License**: GNU General Public License v3
- **Issues**: https://github.com/Fo170/BatteryKalman/issues
- **Discussions**: https://github.com/Fo170/BatteryKalman/discussions

Clone latest from main:
```bash
git clone https://github.com/Fo170/BatteryKalman.git
cd BatteryKalman
git checkout main
```

## Important Notes

- The library assumes regular, frequent updates (10 Hz minimum recommended). Sparse updates degrade convergence.
- SoC estimates assume stable Coulomb counting; integrate current accurately to avoid drift.
- Charge state detection is delegated to `BatteryModel.detectChargeState()` — ensure your model correctly identifies charge phases.
- Capacity convergence takes time; typically 3–5 full charge/discharge cycles in Phase 2–3 to reach ±3-5% accuracy.
- All state that must survive power-loss (SoC_fused, C_hat, cycle counts, etc.) must be explicitly persisted by the user.
