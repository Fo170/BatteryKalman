# AGENTS.md

Lead-acid battery telemetry analysis workspace plus a corrected Arduino EKF library. Analysis work (data, Python scripts, report) lives entirely in `doc/` — which is **untracked local-only content**, not part of the published library on GitHub. No build, test, or lint tooling.

## Layout

- `src/BatteryKalman.h` — **corrected** library v1.5.3 (7 bug fixes F1–F7 + stability gate F8). It is currently an **uncommitted edit** over the committed v1.5 (`git status` shows `M src/BatteryKalman.h`) — do not revert or fresh-clone it. It `#include`s `BatteryModels.h` and `Coulomb.h`, which are NOT in this repo: supply your own implementations (or use `Fo170/BatteryModels` v1.3, which integrates the F7 fix documented in `doc/correction_BatteryModels.md`).
- `doc/` — analysis workspace (untracked):
  - `courant.xlsx` — sheet `courant`, 44,052 rows. Columns: `DateTime` (ISO, 5-min interval, starts `2026-02-18 00:20:00`), `courant` (amps; negative = discharge).
  - `tension.xlsx` — sheet `tension`, 44,667 rows. Columns: `DateTime`, `tension` (volts).
  - `evo_bat.csv` — merged telemetry, the analysis input: `epoch secondes utc; tension (V); courant (A)` (epoch = UTC seconds; French-time timestamps converted via `Europe/Paris`).
  - `analyse_kalman.py` / `analyse_kalman_buggy.py` / `analyse_kalman_fixed.py` — Python ports; `_fixed` has all corrections applied. Output: `evo_bat_kalman.csv`.
  - `analyse_kalman.md` — report incl. bug catalog (§8); `kalman_serie_temps.png`, `kalman_zoom_semaine.png` — figures; `correction_BatteryModels.md` — F7 fix; `Battery Kalman Filter.docx` — original algorithm spec.
- Root library docs: `CLAUDE.md` (in-depth architecture + API), `CHANGELOG.md` (bug history), `MIGRATION.md` (BatteryModels v1.2+ API renames), `Exemples/` (Arduino usage sketches).

## Gotchas

- Data CSVs are **semicolon-separated** — read/write with `sep=";"`, not the default comma.
- The analysis scripts read/write `evo_bat.csv` / `evo_bat_kalman.csv` by **relative path** — run them from inside `doc/`.
- Files are real `.xlsx` — read with `openpyxl` or `pandas.read_excel`; never treat as CSV.
- Row counts differ (44,052 vs 44,667) and timestamps are not guaranteed aligned: **join the two datasets on `DateTime`/epoch, never on row index.**
- If regenerating `evo_bat.csv`: pandas stores tz-aware datetimes in microseconds — divide the epoch conversion by `10**6`, not `10**9`.
- The stock library had 7 real bugs (F1–F7, `doc/analyse_kalman.md` §8): FLOAT transitions were dead code, `R_measured` never updated, EKF aging state never learned, segment measurement reset before close, REST_LONG closure blocked, learning gated by `isAutoDetect()`, and FLOAT masked by REST in `detectChargeState()`. Use `src/BatteryKalman.h`, not a fresh GitHub copy.
