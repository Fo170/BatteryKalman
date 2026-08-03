"""
Faithful Python port of BatteryKalman v1.5 (Fo170/BatteryKalman) +
BatteryModels v1.2.0 (TECH_FLOODED, 6 cells) applied to evo_bat.csv.

Data: 5-min samples, epoch UTC / tension(V) / courant(A).
Temperature: no data available -> fixed at 25 C (thermal terms vanish).
"""
import pandas as pd
import numpy as np
from datetime import datetime, timezone

# ---------------- Model constants (BatteryModels.h TECH_FLOODED) ----------------
N_CELLS = 6
PARAMS = dict(
    A=0.017143, B=0.109657, C=1.981743,       # OCV poly (per cell)
    R25_mOhm=15.0, beta_temp=0.015, gamma_soc=0.5,
    Rpol0_mOhm=4.0, delta_pol=2.0, tau_pol_s=4.0, k_peukert=1.40,
    dVocv_dT=-0.0037,
    V_float_per_cell=2.37, V_abs_per_cell=2.45,
    V_min_per_cell=1.75, V_max_per_cell=2.50,
    R_soc_max=18.0,
    E_a_R=3500.0, alpha_tau=0.04, d2Vocv_dT2=-8e-6,
)

# ---------------- Kalman constants (BatteryKalman.h) ----------------
KALMAN_P_INIT_C = 500.0
KALMAN_P_INIT_AGING = 0.000001
KALMAN_P_MIN_C = 0.10
KALMAN_P_MIN_AGING = 0.0000001
Q_AGING_UNCERTAINTY = 0.000002
Q_CAPACITY_DRIFT = 0.01
R_INIT = 4.0
R_MIN = 0.5
R_MAX = 100.0
R_SMOOTH_ALPHA = 0.1
SEG_MIN_DAH = 0.20
SEG_MIN_DSOC_P0 = 3.0
SEG_MIN_DSOC_P1 = 5.0
SEG_MIN_DSOC_P2 = 8.0
SEG_DSOC_HIGH_CONF = 15.0
BATTERY_CHANGE_THR = 0.30
BATTERY_CHANGE_COUNT = 3
TEMP = 25.0

State_UNKNOWN = 0
State_DISCHARGE = 1
State_BULK = 2
State_ABSORPTION = 3
State_FLOAT = 4
State_REST = 5
State_REST_LONG = 6

PHASE_BOOTSTRAP = 0
PHASE_COARSE = 1
PHASE_REFINE = 2
PHASE_TRACK = 3
PHASE_NAMES = ["Bootstrap", "Convergence rapide", "Raffinement", "Suivi vieillissement"]
STATE_NAMES = ["Unknown", "Discharge", "Bulk", "Absorption", "Float", "Rest", "Rest Long"]


class BatteryModel:
    def __init__(self, cap=0.0):
        self.C_nominal = cap
        self.auto_detect = (cap == 0.0)
        self.R_int_nominal = 0.0
        self.V_pol_state = 0.0
        self.last_current = 0.0

    def getParams(self):
        return PARAMS

    def getNominalVoltage(self):
        return 2.0 * N_CELLS

    def getCellCount(self):
        return N_CELLS

    def getNominalCapacity(self):
        return self.C_nominal

    def getVoltageToleranceBand(self):
        V_nom = self.getNominalVoltage()
        if V_nom <= 0:
            return 0.2
        return max(0.1, V_nom * 0.015)

    def getResistanceAtTemp(self, R25, T):
        tp = self.getParams()
        if tp["E_a_R"] > 0.001:
            T_abs = T + 273.15
            T25_abs = 298.15
            R_gas = 8.314
            alpha = tp["E_a_R"] / (R_gas * T25_abs * T_abs)
            return R25 * np.exp(alpha * (T25_abs - T_abs))
        else:
            return R25 * (1.0 + tp["beta_temp"] * (25.0 - T))

    def getTauPol_thermal(self, T):
        tp = self.getParams()
        if abs(tp["alpha_tau"]) < 1e-6:
            return tp["tau_pol_s"]
        return tp["tau_pol_s"] * np.exp(tp["alpha_tau"] * (T - 25.0))

    def removeThermalCorrectionOCV(self, V_cell, T):
        tp = self.getParams()
        dT = T - 25.0
        V_cell -= tp["dVocv_dT"] * dT
        if abs(tp["d2Vocv_dT2"]) > 1e-9:
            V_cell -= 0.5 * tp["d2Vocv_dT2"] * dT * dT
        return V_cell

    def soc_from_ocv_poly(self, V_cell):
        p = self.getParams()
        A, B, Cv = p["A"], p["B"], p["C"] - V_cell
        if abs(A) < 1e-6:
            soc = (-Cv / B) * 100.0 if abs(B) > 1e-6 else 50.0
        else:
            D = B * B - 4.0 * A * Cv
            if D < 0.0:
                soc = 100.0 if (V_cell > p["C"] + p["B"] * 0.5 + p["A"] * 0.25) else 0.0
            else:
                sqrtD = np.sqrt(D)
                r1 = (-B + sqrtD) / (2.0 * A)
                r2 = (-B - sqrtD) / (2.0 * A)
                if 0.0 <= r1 <= 1.0:
                    soc = r1 * 100.0
                elif 0.0 <= r2 <= 1.0:
                    soc = r2 * 100.0
                else:
                    soc = np.clip(r1, 0.0, 1.0) * 100.0
        return np.clip(soc, 0.0, 100.0)

    def ocvToSoc(self, voltage, T):
        V_cell = voltage / N_CELLS
        V_cell = self.removeThermalCorrectionOCV(V_cell, T)
        return self.soc_from_ocv_poly(V_cell)

    def calculateRsocWithSaturation(self, s_ref):
        p = self.getParams()
        max_ratio = p["R_soc_max"]
        if s_ref <= 0.0:
            return max_ratio
        s_ref = np.clip(s_ref, 0.001, 1.0)
        unbounded = 1.0 + p["gamma_soc"] * (1.0 / s_ref - 1.0)
        if unbounded <= 1.0:
            return 1.0
        overshoot = unbounded - 1.0
        max_overshoot = max_ratio - 1.0
        if overshoot >= max_overshoot:
            return max_ratio
        sat = overshoot / max_overshoot
        smoothing = 1.0 - np.exp(-2.0 * sat * sat)
        return 1.0 + max_overshoot * smoothing

    def correctVoltageToOCV(self, voltage, current, T, dt_s):
        tp = self.getParams()
        V_cell = voltage / N_CELLS
        R25 = self.R_int_nominal if self.R_int_nominal > 0 else tp["R25_mOhm"]
        R_T = self.getResistanceAtTemp(R25, T)
        R_T = max(R_T, 0.001)
        soc_est = self.ocvToSoc(voltage, T)
        s_ref = soc_est / 100.0
        R_soc = self.calculateRsocWithSaturation(s_ref)
        R_int = R_T * R_soc / 1000.0
        R_pol = (tp["Rpol0_mOhm"] / 1000.0) * (1.0 + tp["delta_pol"] * np.exp(-soc_est / 20.0))
        V_pol_inf = current * R_pol
        if dt_s > 0.0 and tp["tau_pol_s"] > 0.0:
            tau_th = self.getTauPol_thermal(T)
            alpha_rc = np.clip(1.0 - np.exp(-dt_s / tau_th), 0.0, 1.0)
            self.V_pol_state += alpha_rc * (V_pol_inf - self.V_pol_state)
        else:
            self.V_pol_state = 0.0
        V_drop_ohm = R_int * current
        V_corrected = V_cell - (V_drop_ohm + self.V_pol_state)
        # applyThermalCorrectionOCV
        dT = T - 25.0
        V_corrected += tp["dVocv_dT"] * dT
        if abs(tp["d2Vocv_dT2"]) > 1e-9:
            V_corrected += 0.5 * tp["d2Vocv_dT2"] * dT * dT
        r_int_eff = R_int
        self.last_current = current
        return V_corrected * N_CELLS, r_int_eff

    def detectChargeState(self, voltage, current, capacity=-1.0):
        tp = self.getParams()
        cap = capacity if capacity > 0.0 else self.C_nominal
        if cap <= 0.0:
            cap = 100.0
        V_float_ref = tp["V_float_per_cell"] * N_CELLS
        V_abs_ref = tp["V_abs_per_cell"] * N_CELLS
        I_float_max = max(0.6, cap * 0.02)
        V_tol = self.getVoltageToleranceBand()
        in_rest = abs(current) < 0.05
        charging = current > 0.05
        discharging = current < -0.05
        near_abs = (voltage >= V_abs_ref - V_tol) and (voltage <= V_abs_ref + V_tol)
        near_float = (voltage >= V_float_ref - V_tol) and (voltage <= V_float_ref + V_tol * 0.5)
        if discharging:
            return State_DISCHARGE
        if near_float and abs(current) < I_float_max:
            return State_FLOAT
        if in_rest:
            return State_REST
        if near_abs and charging:
            return State_ABSORPTION
        if charging:
            return State_BULK
        return State_UNKNOWN

    def isAutoDetect(self):
        return self.auto_detect

    def isOcvReliable(self):
        return True


class CoulombMeter:
    def __init__(self):
        self.ah = 0.0
        self.last_interval = 0.0

    def feed(self, I, dt_s):
        self.last_interval = dt_s
        self.ah += I * dt_s / 3600.0

    def getAmpereHours(self):
        return self.ah

    def getLastInterval(self):
        return self.last_interval

    def reset(self, *args):
        self.ah = 0.0


class KalmanState2D:
    def __init__(self):
        self.C_hat = 0.0
        self.dC_dCycle = -0.0005
        self.P = [[KALMAN_P_INIT_C, 0.0], [0.0, KALMAN_P_INIT_AGING]]
        self.R_estimated = R_INIT
        self.R_measured = R_INIT
        self.n_updates = 0
        self.initialized = False
        self.outlier_streak = 0
        self.confidence = 0.0
        self.innovation_sum_sq = 0.0
        self.innovation_count = 0


class SoCData:
    def __init__(self):
        self.SoC_coulomb = 50.0
        self.SoC_voltage = 50.0
        self.SoC_voltage_raw = 50.0
        self.SoC_fused = 50.0
        self.SoC_uncertainty = 10.0
        self.soc_is_raw = True
        self.Ah_total_charged = 0.0
        self.Ah_total_discharged = 0.0
        self.cycles_partial = 0.0
        self.cycles_full = 0
        self.DoD_accumulated = 0.0
        self.last_sync_time = 0
        self.coulomb_initialized = False


class BatteryKalman:
    def __init__(self, model, coulomb):
        self.data = SoCData()
        self.kalman = KalmanState2D()
        self.model = model
        self.coulomb = coulomb
        self.mppt_state = State_UNKNOWN
        self.mppt_state_prev = State_UNKNOWN
        self.state_entry_ms = 0
        self.dVdt = 0.0
        self.V_prev = 0.0
        self.I_prev = 0.0
        self.phase = PHASE_BOOTSTRAP
        self.seg = dict(active=False, Ah_start=0.0, SoC_start=0.0, conf_start=0.0,
                        start_ms=0, I_sq_sum=0.0, I_sum=0.0, n=0,
                        I_min=999.0, I_max=0.0, had_discharge=False, had_charge=False)
        self.R_int_eff = 0.0
        self.last_Ah = 0.0
        self.last_cycles = 0.0
        self.alpha_fused = 0.5
        self.cycle = dict(SoC_max=0.0, SoC_min=100.0, was_charging=False,
                          was_discharging=False, cycle_start_time=0)
        self.events = []
        self._init_cycle = False

    # ---------- Kalman core ----------
    def predictKalman(self, delta_cycles):
        if not self.kalman.initialized:
            return
        if delta_cycles < 0:
            delta_cycles = 0.0
        self.kalman.C_hat -= self.kalman.dC_dCycle * delta_cycles
        F01 = -delta_cycles
        P00, P01 = self.kalman.P[0][0], self.kalman.P[0][1]
        P10, P11 = self.kalman.P[1][0], self.kalman.P[1][1]
        FP00 = P00 + F01 * P10
        FP01 = P01 + F01 * P11
        FP10 = P10
        FP11 = P11
        P00_new = FP00 + FP01 * F01
        P01_new = FP01
        P10_new = FP10 + FP11 * F01
        P11_new = FP11
        Q_C = Q_AGING_UNCERTAINTY * delta_cycles + Q_CAPACITY_DRIFT
        Q_aging = Q_AGING_UNCERTAINTY
        self.kalman.P[0][0] = min(P00_new + Q_C, KALMAN_P_INIT_C)
        self.kalman.P[0][1] = P01_new
        self.kalman.P[1][0] = P10_new
        self.kalman.P[1][1] = min(P11_new + Q_aging, KALMAN_P_INIT_AGING)

    def updateREstimate(self, innovation, S):
        self.kalman.innovation_sum_sq += innovation * innovation
        self.kalman.innovation_count += 1
        if self.kalman.innovation_count >= 5:
            innovation_var = self.kalman.innovation_sum_sq / self.kalman.innovation_count
            R_est = max(R_MIN, innovation_var - self.kalman.P[0][0])
            self.kalman.R_estimated = R_SMOOTH_ALPHA * R_est + (1.0 - R_SMOOTH_ALPHA) * self.kalman.R_estimated
            self.kalman.R_estimated = np.clip(self.kalman.R_estimated, R_MIN, R_MAX)
            self.kalman.R_measured = self.kalman.R_estimated
            self.kalman.innovation_sum_sq = 0.0
            self.kalman.innovation_count = 0

    def updatePhaseFromConfidence(self):
        if not self.kalman.initialized:
            self.phase = PHASE_BOOTSTRAP
        elif self.kalman.confidence < 0.3:
            self.phase = PHASE_COARSE
        elif self.kalman.confidence < 0.6:
            self.phase = PHASE_REFINE
        else:
            self.phase = PHASE_TRACK

    def applyKalmanUpdate2D(self, C_measured, R, delta_cycles):
        if not self.kalman.initialized:
            self.kalman.C_hat = C_measured
            self.kalman.dC_dCycle = -0.0005
            self.kalman.P[0][0] = R
            self.kalman.P[1][1] = Q_AGING_UNCERTAINTY
            self.kalman.initialized = True
            self.kalman.n_updates = 1
            self.kalman.confidence = 0.2
            self.updatePhaseFromConfidence()
            return
        self.predictKalman(delta_cycles)
        innovation = C_measured - self.kalman.C_hat
        S = self.kalman.P[0][0] + R
        if S <= 0:
            S = R
        sigma = np.sqrt(S)
        if abs(innovation) > 3.0 * sigma:
            self.kalman.outlier_streak += 1
            if (self.kalman.outlier_streak >= BATTERY_CHANGE_COUNT and
                    abs(innovation) > BATTERY_CHANGE_THR * self.kalman.C_hat):
                self.kalman.C_hat = C_measured
                self.kalman.dC_dCycle = -0.0005
                self.kalman.P[0][0] = R * 2.0
                self.kalman.P[1][1] = Q_AGING_UNCERTAINTY * 5.0
                self.kalman.n_updates = 3
                self.kalman.confidence = 0.1
                self.kalman.outlier_streak = 0
                self.events.append(("BATTERY_REPLACED", C_measured))
                return
            self.kalman.P[0][0] *= 2.0
            self.kalman.P[1][1] = min(self.kalman.P[1][1] * 1.5, Q_AGING_UNCERTAINTY * 10.0)
            self.updatePhaseFromConfidence()
            return
        self.kalman.outlier_streak = 0
        K_C = self.kalman.P[0][0] / S
        K_aging = self.kalman.P[1][0] / S
        self.kalman.C_hat += K_C * innovation
        self.kalman.dC_dCycle += K_aging * innovation
        P_CC_new = (1.0 - K_C) * self.kalman.P[0][0]
        P_Ca_new = (1.0 - K_C) * self.kalman.P[0][1]
        P_aC_new = self.kalman.P[1][0] - K_aging * self.kalman.P[0][0]
        P_aa_new = self.kalman.P[1][1] - K_aging * self.kalman.P[0][1]
        self.kalman.P[0][0] = max(KALMAN_P_MIN_C, P_CC_new)
        self.kalman.P[0][1] = P_Ca_new
        self.kalman.P[1][0] = P_aC_new
        self.kalman.P[1][1] = max(KALMAN_P_MIN_AGING, P_aa_new)
        self.kalman.n_updates += 1
        self.updateREstimate(innovation, S)
        n_factor = min(1.0, self.kalman.n_updates / 10.0)
        p_factor = 1.0 - min(1.0, self.kalman.P[0][0] / KALMAN_P_INIT_C)
        self.kalman.confidence = (n_factor + p_factor) * 0.5
        self.updatePhaseFromConfidence()

    # ---------- model / state ----------
    def getEffectiveCapacity(self):
        if self.kalman.initialized and self.kalman.C_hat > 1.0:
            return self.kalman.C_hat
        if self.model.getNominalCapacity() > 0.0:
            return self.model.getNominalCapacity()
        return 0.0

    def updateDerivatives(self, V, I, dt_s):
        if dt_s <= 0.0 or self.V_prev <= 0.0:
            self.V_prev = V
            self.I_prev = I
            return
        alpha_f = dt_s / (20.0 + dt_s)
        self.dVdt = alpha_f * ((V - self.V_prev) / dt_s) + (1.0 - alpha_f) * self.dVdt
        self.V_prev = V
        self.I_prev = I

    def updateMpptState(self, V, I, now_ms):
        C_ref = self.getEffectiveCapacity()
        self.mppt_state = self.model.detectChargeState(V, I, C_ref)
        if self.mppt_state != self.mppt_state_prev:
            self.state_entry_ms = now_ms
        if self.mppt_state == State_REST and (now_ms - self.state_entry_ms) >= 7200000:
            self.mppt_state = State_REST_LONG

    def doSync(self, new_SoC, confidence, reason, now_ms):
        self.data.SoC_coulomb = new_SoC
        self.data.SoC_fused = new_SoC
        self.data.last_sync_time = now_ms
        self.coulomb.reset(False)
        self.last_Ah = self.coulomb.getAmpereHours()
        self.events.append(("SYNC", f"{reason} conf={confidence*100:.0f}%"))
        self.startNewSegment(new_SoC, confidence, now_ms)

    def handleSyncEvents(self, V_cell_ocv, now_ms):
        total_ocv = V_cell_ocv * self.model.getCellCount()
        if self.mppt_state == State_FLOAT and self.mppt_state_prev != State_FLOAT:
            self.doSync(100.0, 0.95, "Float charge", now_ms)
            return
        if self.mppt_state == State_REST_LONG and (now_ms - self.data.last_sync_time) > 1800000:
            extra = now_ms - self.state_entry_ms - 7200000
            conf = 0.70 + 0.15 * min(1.0, extra / 7200000.0)
            self.doSync(self.data.SoC_voltage, conf, "Long rest OCV", now_ms)
            return
        min_voltage = self.model.getParams()["V_min_per_cell"] * self.model.getCellCount()
        if (self.mppt_state == State_REST or self.mppt_state == State_REST_LONG) and total_ocv < min_voltage:
            self.doSync(3.0, 0.75, "Low voltage cutoff", now_ms)
            return
        if self.mppt_state == State_REST and (now_ms - self.data.last_sync_time) > 300000:
            self.data.SoC_coulomb = 0.90 * self.data.SoC_coulomb + 0.10 * self.data.SoC_voltage
            self.data.last_sync_time = now_ms

    def fuseSoC(self):
        if self.phase == PHASE_BOOTSTRAP:
            self.alpha_fused = 0.0
            self.data.soc_is_raw = True
            self.data.SoC_voltage_raw = (self.model.ocvToSoc(self.V_prev, TEMP)
                                         if self.V_prev > 1.0 else self.data.SoC_voltage)
            return self.data.SoC_voltage_raw
        self.data.soc_is_raw = False
        kalman_conf = self.kalman.confidence
        st = self.mppt_state
        if st == State_FLOAT:      alpha_base = 0.00
        elif st == State_REST_LONG: alpha_base = 0.02
        elif st == State_REST:     alpha_base = 0.10
        elif st == State_ABSORPTION: alpha_base = 0.70
        elif st == State_BULK:     alpha_base = 0.95
        elif st == State_DISCHARGE: alpha_base = 0.97
        else:                       alpha_base = 0.50
        if self.phase == PHASE_COARSE:
            self.alpha_fused = alpha_base * kalman_conf
        else:
            self.alpha_fused = alpha_base
        return self.alpha_fused * self.data.SoC_coulomb + (1.0 - self.alpha_fused) * self.data.SoC_voltage

    def computeUncertainty(self):
        ph = self.phase
        base = 15.0 if ph == PHASE_BOOTSTRAP else (10.0 if ph == PHASE_COARSE else (6.0 if ph == PHASE_REFINE else 3.0))
        if self.mppt_state in (State_BULK, State_DISCHARGE):
            base += 2.0
        return base

    # ---------- segments ----------
    def startNewSegment(self, SoC_start, conf, now_ms):
        s = self.seg
        s["active"] = True
        s["Ah_start"] = self.coulomb.getAmpereHours()
        s["SoC_start"] = SoC_start
        s["conf_start"] = conf
        s["start_ms"] = now_ms
        s["I_sq_sum"] = 0.0; s["I_sum"] = 0.0; s["n"] = 0
        s["I_min"] = 999.0; s["I_max"] = 0.0
        s["had_discharge"] = False; s["had_charge"] = False

    def updateSegmentAndKalman(self, I, delta_cycles, now_ms):
        s = self.seg
        if not s["active"]:
            if self.mppt_state in (State_FLOAT, State_REST_LONG):
                conf = 0.95 if self.mppt_state == State_FLOAT else 0.75
                self.startNewSegment(self.data.SoC_fused, conf, now_ms)
            return
        I_abs = abs(I)
        s["I_sum"] += I_abs
        s["I_sq_sum"] += I_abs * I_abs
        s["n"] += 1
        s["I_min"] = min(s["I_min"], I_abs)
        s["I_max"] = max(s["I_max"], I_abs)
        if I < -0.05: s["had_discharge"] = True
        if I > 0.05: s["had_charge"] = True
        SoC_end = 0.0; conf_end = 0.0; close = False
        if self.mppt_state == State_FLOAT and self.mppt_state_prev != State_FLOAT:
            SoC_end = 100.0; conf_end = 0.95; close = True
        elif self.mppt_state == State_REST_LONG and self.mppt_state_prev != State_REST_LONG:
            extra = now_ms - self.state_entry_ms - 7200000
            SoC_end = self.data.SoC_voltage
            conf_end = 0.70 + 0.15 * min(1.0, extra / 7200000.0)
            close = True
        if not close:
            return
        dAh = abs(self.coulomb.getAmpereHours() - s["Ah_start"])
        dSoC = abs(SoC_end - s["SoC_start"])
        if self.phase == PHASE_BOOTSTRAP:
            min_dSoC = SEG_MIN_DSOC_P0
        elif self.phase in (PHASE_REFINE, PHASE_TRACK):
            min_dSoC = SEG_MIN_DSOC_P2
        else:
            min_dSoC = SEG_MIN_DSOC_P1
        if dAh < SEG_MIN_DAH or dSoC < min_dSoC or s["n"] == 0:
            self.events.append(("SEG_REJECTED", f"dAh={dAh:.3f} dSoC={dSoC:.1f} n={s['n']} state={STATE_NAMES[self.mppt_state]}"))
            self.startNewSegment(SoC_end, conf_end, now_ms)
            return
        C_measured = dAh / (dSoC / 100.0)
        if C_measured < 1.0 or C_measured > 2000.0:
            self.events.append(("SEG_REJECTED", f"C_out_of_bounds C={C_measured:.1f}"))
            self.startNewSegment(SoC_end, conf_end, now_ms)
            return
        R = self.kalman.R_measured
        self.events.append(("SEG_ACCEPTED",
            f"dAh={dAh:.3f} dSoC={dSoC:.1f} C_measured={C_measured:.2f} phase={PHASE_NAMES[self.phase]}"))
        self.applyKalmanUpdate2D(C_measured, R, delta_cycles)
        self.startNewSegment(SoC_end, conf_end, now_ms)

    def updateCycleDetection(self, I, now_ms):
        if not self._init_cycle:
            self.cycle["cycle_start_time"] = now_ms
            self._init_cycle = True
        self.cycle["SoC_max"] = max(self.cycle["SoC_max"], self.data.SoC_fused)
        self.cycle["SoC_min"] = min(self.cycle["SoC_min"], self.data.SoC_fused)
        if I > 0.05: self.cycle["was_charging"] = True
        if I < -0.05: self.cycle["was_discharging"] = True
        if (self.cycle["was_discharging"] and self.mppt_state == State_FLOAT
                and self.mppt_state_prev != State_FLOAT and self.cycle["SoC_min"] < 85.0):
            DoD = self.cycle["SoC_max"] - self.cycle["SoC_min"]
            self.data.cycles_partial += DoD / 100.0
            self.data.DoD_accumulated += DoD
            if DoD > 80.0:
                self.data.cycles_full += 1
            self.cycle = dict(SoC_max=0.0, SoC_min=100.0, was_charging=False,
                              was_discharging=False, cycle_start_time=now_ms)
        if now_ms - self.cycle["cycle_start_time"] > 604800000:
            self.cycle = dict(SoC_max=0.0, SoC_min=100.0, was_charging=False,
                              was_discharging=False, cycle_start_time=now_ms)

    # ---------- main ----------
    def update(self, V, I, T, now_ms, dt_s):
        if dt_s <= 0:
            dt_s = 0.1
        dt_h = dt_s / 3600.0
        self.updateDerivatives(V, I, dt_s)
        V_corrected, r_int_eff = self.model.correctVoltageToOCV(V, I, T, dt_s)
        self.R_int_eff = r_int_eff
        V_cell_ocv = V_corrected / self.model.getCellCount()
        self.data.SoC_voltage = self.model.ocvToSoc(V_corrected, T)
        self.updateMpptState(V, I, now_ms)
        current_Ah = self.coulomb.getAmpereHours()
        dAh = current_Ah - self.last_Ah
        self.last_Ah = current_Ah
        if I < 0.0:
            self.data.Ah_total_discharged += (-I) * dt_h
        else:
            self.data.Ah_total_charged += I * dt_h * 0.98
        C_ref = self.getEffectiveCapacity()
        if C_ref > 0.0:
            self.data.SoC_coulomb += (dAh / C_ref) * 100.0
            self.data.SoC_coulomb = np.clip(self.data.SoC_coulomb, 0.0, 100.0)
        delta_cycles = self.data.cycles_partial - self.last_cycles
        self.updateSegmentAndKalman(I, delta_cycles, now_ms)
        self.last_cycles = self.data.cycles_partial
        self.handleSyncEvents(V_cell_ocv, now_ms)
        self.data.SoC_fused = self.fuseSoC()
        self.data.SoC_fused = np.clip(self.data.SoC_fused, 0.0, 100.0)
        self.data.SoC_uncertainty = self.computeUncertainty()
        self.updateCycleDetection(I, now_ms)
        self.mppt_state_prev = self.mppt_state


def main():
    df = pd.read_csv("evo_bat.csv", sep=";")
    df = df.dropna(subset=["courant (A)"]).reset_index(drop=True)
    df = df.dropna(subset=["tension (V)"]).reset_index(drop=True)
    epoch0 = df["epoch secondes utc"].iloc[0]

    model = BatteryModel(cap=0.0)   # auto-detect -> Kalman learning active
    coulomb = CoulombMeter()
    bk = BatteryKalman(model, coulomb)

    rows = []
    prev_epoch = None
    for _, r in df.iterrows():
        V = float(r["tension (V)"])
        I = float(r["courant (A)"])
        ep = int(r["epoch secondes utc"])
        if prev_epoch is None:
            dt_s = 300.0
        else:
            dt_s = float(ep - prev_epoch)
        prev_epoch = ep
        now_ms = (ep - epoch0) * 1000
        coulomb.feed(I, dt_s)
        bk.update(V, I, TEMP, now_ms, dt_s)
        rows.append(dict(
            epoch=ep,
            V=V, I=I,
            state=STATE_NAMES[bk.mppt_state],
            phase=PHASE_NAMES[bk.phase],
            SoC_fused=bk.data.SoC_fused,
            SoC_coulomb=bk.data.SoC_coulomb,
            SoC_voltage=bk.data.SoC_voltage,
            C_hat=bk.kalman.C_hat if bk.kalman.initialized else np.nan,
            dC_dCycle=bk.kalman.dC_dCycle if bk.kalman.initialized else np.nan,
            P_CC=bk.kalman.P[0][0] if bk.kalman.initialized else np.nan,
            P_aging=bk.kalman.P[1][1] if bk.kalman.initialized else np.nan,
            n_updates=bk.kalman.n_updates,
            confidence=bk.kalman.confidence,
            R_est=bk.kalman.R_estimated,
            outlier_streak=bk.kalman.outlier_streak,
        ))

    out = pd.DataFrame(rows)
    out.to_csv("evo_bat_kalman.csv", sep=";", index=False, float_format="%.6f")

    print("rows processed:", len(out))
    print("kalman initialized at n_updates>0 count:", (out['n_updates'] > 0).sum())
    if out['C_hat'].notna().any():
        print("first init:", out.dropna(subset=['C_hat']).iloc[0][['epoch','C_hat','n_updates']].to_dict())
    print("final C_hat:", out['C_hat'].dropna().iloc[-1] if out['C_hat'].notna().any() else None)
    print("final n_updates:", out['n_updates'].iloc[-1])
    print("final confidence:", out['confidence'].iloc[-1])
    print("final phase:", out['phase'].iloc[-1])
    print("cycles_partial:", bk.data.cycles_partial, "cycles_full:", bk.data.cycles_full)
    print("Ah charged:", bk.data.Ah_total_charged, "Ah discharged:", bk.data.Ah_total_discharged)
    print("R_est final:", out['R_est'].iloc[-1])
    print()
    print("--- events ---")
    for ev in bk.events:
        print(ev)


if __name__ == "__main__":
    main()
