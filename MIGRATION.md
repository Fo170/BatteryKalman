# Migration Guide: BatteryKalman v2.1.0 (BatteryModels v1.2.0 Compatible)

## Summary of Changes

BatteryKalman has been updated to work with **BatteryModels v1.2.0** (released July 19, 2026), which introduced advanced thermal modeling. This update required API changes due to terminology shifts in BatteryModels.

## Breaking Changes

### 1. Method Rename: `detectMpptState()` → `detectChargeState()`

**Before (BatteryModels v1.1.x compatible):**
```cpp
MpptState state = model->detectMpptState(voltage, current, capacity);
```

**After (BatteryModels v1.2.0+):**
```cpp
ChargeState state = model->detectChargeState(voltage, current, capacity);
```

The method is now called internally in `updateMpptState()` — no user code change needed unless you were calling it directly.

### 2. Enum Type: `MpptState` → `ChargeState`

**Before:**
```cpp
MpptState mppt_state = MPPT_UNKNOWN;
if (mppt_state == MPPT_FLOAT) { ... }
```

**After:**
```cpp
ChargeState charge_state = State_UNKNOWN;  // Now from BatteryModels
if (charge_state == State_FLOAT) { ... }
```

**Enum value mapping:**
| Old (v1.1.x) | New (v1.2.0+) |
|---|---|
| MPPT_UNKNOWN | State_UNKNOWN |
| MPPT_DISCHARGE | State_DISCHARGE |
| MPPT_BULK | State_BULK |
| MPPT_ABSORPTION | State_ABSORPTION |
| MPPT_FLOAT | State_FLOAT |
| MPPT_REST | State_REST |
| MPPT_REST_LONG | State_REST_LONG |

### 3. Getter Methods

**Before:**
```cpp
MpptState state = battery.getMpptState();
const char* name = battery.getMpptStateStr();
```

**After:**
```cpp
ChargeState state = battery.getChargeState();
const char* name = battery.getChargeStateStr();
```

## User Code Updates

If your code uses these methods, update as shown:

```cpp
// Example: Get current charge state and print it
ChargeState state = battery.getChargeState();
Serial.println(battery.getChargeStateStr());

// Old code checking MPPT state
// if (battery.getMpptStateStr() == "MPPT_FLOAT") { ... }
// New code
if (battery.getChargeState() == State_FLOAT) { ... }
```

## Dependency Requirements

- **BatteryModels**: v1.2.0 or later (July 2026+)
- **Coulomb**: Any compatible version
- **Arduino Framework**: C++11 minimum

## Backward Compatibility

This version is **NOT compatible** with BatteryModels v1.1.x and earlier. If you need the old API:
- Keep using BatteryKalman v2.0.0 with BatteryModels v1.1.x
- Or update both libraries together

## Testing After Migration

After updating, verify:
1. ✓ Code compiles without `detectMpptState` or `MPPT_*` errors
2. ✓ Charge state detection works (check `getChargeStateStr()` output)
3. ✓ SoC convergence progresses through 4 phases normally
4. ✓ State persistence (if used) continues to work

## Questions?

If you encounter issues:
1. Check that BatteryModels v1.2.0+ is installed
2. Verify `#include <BatteryModels.h>` points to v1.2.0
3. Ensure no old `MPPT_` enum values remain in your code
4. See CLAUDE.md for detailed API documentation
