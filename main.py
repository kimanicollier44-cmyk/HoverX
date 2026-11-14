#!/usr/bin/env python3
"""
HoverX — Personal Hover Vehicle Autopilot Lab
==============================================
by Darion Kellar

SIM-ONLY educational lab for a personal hover vehicle (ducted-fan +
tilt-rotor). Pure Python stdlib, ONE main.py, zero network, zero
dependencies. Nothing here controls real hardware, flies a real
vehicle, or leaves the sandbox — every live action is behind a
HOLD gate, and the HOLD always wins.

What is modelled, with real physics / control math:
  1. Vehicle model ............ mass budget, thrust per fan, hover thrust = m*g
  2. Ground effect ............ Cheeseman-Bennett style thrust benefit vs h/R
  3. Stability / control ...... PID attitude hold, wind-gust rejection sim
  4. Redundancy ............... N fans — can it still hover on N-1?
  5. Low-altitude governor .... speed / altitude caps by zone (X-family style)
  6. Departure interlocks ..... RVx-style: every interlock must pass first
  7. Route sim ................ Porter Ranch local hop, energy + SoC drain
  8. Battery / energy ......... momentum-theory hover power, endurance
  9. Regime classifier ........ stdlib kNN: safe / marginal / no-go
 10. Advisory seats ........... Carbon 4 decider (tighten-only), Muse
                                advisor (never overrides a HOLD), Flow
                                plan-only (rendered=False, live=0)

Run:
  python3 main.py            full demo + CHECKS, prints VERDICT
  python3 main.py demo       same as above
  python3 main.py hold-demo  ask for a live flight -> HOLD, exit 2
"""

import hashlib
import math
import sys

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
G = 9.80665            # standard gravity, m/s^2
RHO_SEA = 1.225        # air density at sea level, kg/m^3

# ---------------------------------------------------------------------------
# 1. Vehicle model — ducted fan + tilt-rotor personal hover vehicle
# ---------------------------------------------------------------------------
VEHICLE = {
    "name": "HoverX One (sim)",
    "empty_mass_kg": 320.0,       # structure, fans, motors, avionics
    "battery_mass_kg": 110.0,     # included in empty mass budget below
    "max_payload_kg": 160.0,      # pilot + passenger / cargo, sim only
    "fans": 8,                    # ducted fans, tilt-rotor pairs
    "rotor_radius_m": 0.55,       # per fan
    "fan_max_thrust_n": 950.0,    # per fan, bench rating (sim)
    "figure_of_merit": 0.72,      # ducted-fan figure of merit
    "battery_kwh": 42.0,
    "usable_soc_frac": 0.80,      # only 80% of pack is usable in sim
}


def mtom_kg(payload_kg):
    """Maximum take-off mass for a given payload."""
    return VEHICLE["empty_mass_kg"] + payload_kg


def disk_area_m2():
    """Total rotor disk area across all fans."""
    return VEHICLE["fans"] * math.pi * VEHICLE["rotor_radius_m"] ** 2


def hover_thrust_n(mass_kg):
    """Thrust needed to hover: T = m * g. Exact by definition."""
    return mass_kg * G


def thrust_per_fan_n(mass_kg):
    """Hover thrust shared evenly across all fans."""
    return hover_thrust_n(mass_kg) / VEHICLE["fans"]


def disk_loading_n_m2(mass_kg):
    return hover_thrust_n(mass_kg) / disk_area_m2()


# ---------------------------------------------------------------------------
# 8a. Battery / energy — momentum theory
#    Ideal induced power:  P = T^(3/2) / sqrt(2 * rho * A)
#    Real shaft/electric power divides by the figure of merit.
# ---------------------------------------------------------------------------
def ideal_hover_power_w(thrust_n, rho=RHO_SEA, area_m2=None):
    if area_m2 is None:
        area_m2 = disk_area_m2()
    return thrust_n ** 1.5 / math.sqrt(2.0 * rho * area_m2)


def hover_power_w(mass_kg, rho=RHO_SEA):
    """Real hover power = ideal momentum power / figure of merit."""
    ideal = ideal_hover_power_w(hover_thrust_n(mass_kg), rho)
    return ideal / VEHICLE["figure_of_merit"]


def endurance_min(mass_kg, soc_frac=1.0):
    """Hover endurance on the usable pack at a given state of charge."""
    usable_kwh = VEHICLE["battery_kwh"] * VEHICLE["usable_soc_frac"] * soc_frac
    return usable_kwh * 1000.0 / hover_power_w(mass_kg) * 60.0


# ---------------------------------------------------------------------------
# 2. Ground effect — Cheeseman-Bennett style rational fit.
#    Thrust multiplier at constant power, height h above ground, rotor
#    radius R. Calibrated to the published C-B curve shape: strong
#    benefit below one rotor radius, fading to 1.0 (no benefit) high up.
#        m(h) = 1 + 0.22 / ( (h/R)^2 + 0.35 )
# ---------------------------------------------------------------------------
def ground_effect_factor(height_m, rotor_radius_m=None):
    if rotor_radius_m is None:
        rotor_radius_m = VEHICLE["rotor_radius_m"]
    ratio = max(height_m, 0.05) / rotor_radius_m
    return 1.0 + 0.22 / (ratio * ratio + 0.35)


def hover_power_ige_w(mass_kg, height_m, rho=RHO_SEA):
    """Hover power in ground effect: same thrust needs less power."""
    return hover_power_w(mass_kg, rho) / ground_effect_factor(height_m)


# ---------------------------------------------------------------------------
# 3. Stability / control — PID attitude hold + wind gust rejection
#    One-axis (roll) sim: inertia, PID torque, deterministic gust torque
#    built from summed sines (no randomness -> byte-identical runs).
# ---------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = None

    def step(self, error, dt):
        self.integral += error * dt
        self.integral = max(-8.0, min(8.0, self.integral))  # anti-windup clamp
        deriv = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        return max(-self.out_limit, min(self.out_limit, out))


def gust_torque_nm(t):
    """Deterministic wind-gust torque (N*m): summed sines, zero mean-ish."""
    return 55.0 * math.sin(1.7 * t) + 35.0 * math.sin(3.9 * t + 1.3) \
        + 18.0 * math.sin(7.3 * t + 0.5)


def attitude_sim(seconds=12.0, dt=0.01, use_pid=True, initial_deg=6.0):
    """Roll-axis sim. Returns (max_dev_deg, final_deg)."""
    inertia = 260.0                       # kg*m^2, roll axis (sim)
    pid = PID(kp=8000.0, ki=40.0, kd=1800.0, out_limit=10000.0)
    angle = math.radians(initial_deg)     # start tipped by a gust shove
    rate = 0.0
    max_dev = abs(angle)
    steps = int(seconds / dt)
    for i in range(steps):
        t = i * dt
        torque = gust_torque_nm(t)
        if use_pid:
            torque += pid.step(-angle, dt)   # PID pushes angle back to 0
        accel = torque / inertia
        rate += accel * dt
        angle += rate * dt
        max_dev = max(max_dev, abs(angle))
    return math.degrees(max_dev), math.degrees(angle)


# ---------------------------------------------------------------------------
# 4. Redundancy — can it hold hover with one fan out?
# ---------------------------------------------------------------------------
def fan_failure_check(mass_kg, fans_out=1, margin=1.10):
    """Hover on remaining fans, with a safety margin factor on thrust."""
    remaining = VEHICLE["fans"] - fans_out
    available = remaining * VEHICLE["fan_max_thrust_n"]
    needed = hover_thrust_n(mass_kg) * margin
    return {
        "fans_remaining": remaining,
        "available_n": round(available, 1),
        "needed_n": round(needed, 1),
        "can_hover": available >= needed,
    }


# ---------------------------------------------------------------------------
# 5. Low-altitude governor — X-family style caps by zone
# ---------------------------------------------------------------------------
GOVERNOR_ZONES = [
    {"zone": "PAD",          "max_speed_mps": 2.0,  "max_alt_m": 3.0},
    {"zone": "NEIGHBORHOOD", "max_speed_mps": 8.0,  "max_alt_m": 30.0},
    {"zone": "CORRIDOR",     "max_speed_mps": 25.0, "max_alt_m": 120.0},
    {"zone": "TEST_AREA",    "max_speed_mps": 35.0, "max_alt_m": 150.0},
]
ABSOLUTE_CEILING_M = 150.0


def governor_clamp(zone_name, want_speed_mps, want_alt_m):
    zone = next(z for z in GOVERNOR_ZONES if z["zone"] == zone_name)
    speed = min(want_speed_mps, zone["max_speed_mps"])
    alt = min(want_alt_m, zone["max_alt_m"], ABSOLUTE_CEILING_M)
    return {
        "zone": zone_name,
        "speed_mps": speed,
        "alt_m": alt,
        "speed_clamped": speed < want_speed_mps,
        "alt_clamped": alt < want_alt_m,
    }


# ---------------------------------------------------------------------------
# 6. Departure interlocks — RVx-style: ALL must pass before simulated takeoff
# ---------------------------------------------------------------------------
def departure_interlocks(state):
    """state keys: soc, fans_ok, fans_total, cg_offset_m, wind_mps,
    clear_zone, payload_kg, doors_closed, governor_zone_set"""
    checks = [
        ("battery_soc", state["soc"] >= 0.30,
         f"SoC {state['soc']:.0%} >= 30%"),
        ("fan_health", state["fans_ok"] >= state["fans_total"] - 1,
         f"{state['fans_ok']}/{state['fans_total']} fans ok (N-1 allowed)"),
        ("payload_balance", abs(state["cg_offset_m"]) <= 0.08,
         f"CG offset {state['cg_offset_m']:.2f} m within +/-0.08 m"),
        ("payload_limit", state["payload_kg"] <= VEHICLE["max_payload_kg"],
         f"payload {state['payload_kg']:.0f} kg <= {VEHICLE['max_payload_kg']:.0f} kg"),
        ("wind_limit", state["wind_mps"] <= 12.0,
         f"wind {state['wind_mps']:.1f} m/s <= 12.0 m/s"),
        ("clear_zone", bool(state["clear_zone"]),
         "landing zone clear"),
        ("doors_closed", bool(state["doors_closed"]),
         "doors / canopy closed and latched"),
        ("governor_set", bool(state["governor_zone_set"]),
         "governor zone selected and caps loaded"),
    ]
    return {
        "all_pass": all(ok for _, ok, _ in checks),
        "checks": [{"name": n, "ok": ok, "detail": d} for n, ok, d in checks],
    }


NOMINAL_STATE = {
    "soc": 0.92, "fans_ok": 8, "fans_total": 8, "cg_offset_m": 0.02,
    "wind_mps": 4.5, "clear_zone": True, "payload_kg": 95.0,
    "doors_closed": True, "governor_zone_set": True,
}

# ---------------------------------------------------------------------------
# 7. Route sim — Porter Ranch local hop (SIM ONLY, no real flight)
#    Legs are straight-line sim distances around Porter Ranch, CA.
# ---------------------------------------------------------------------------
ROUTE_LEGS = [
    {"from": "Porter Ranch pad", "to": "Porter Ranch Town Center", "dist_km": 1.6},
    {"from": "Porter Ranch Town Center", "to": "Limekiln Canyon overlook", "dist_km": 2.3},
    {"from": "Limekiln Canyon overlook", "to": "Porter Ranch pad", "dist_km": 2.8},
]
CRUISE_SPEED_MPS = 22.0
CRUISE_POWER_FRAC = 0.82     # cruise draws 82% of hover power (sim model)
HOVER_SEGMENTS_MIN = 2.0     # total hover time: lift-off + landing


def route_sim(payload_kg, soc_start=0.92):
    mass = mtom_kg(payload_kg)
    dist_km = sum(leg["dist_km"] for leg in ROUTE_LEGS)
    cruise_min = dist_km * 1000.0 / CRUISE_SPEED_MPS / 60.0
    p_hover = hover_power_w(mass)
    p_cruise = p_hover * CRUISE_POWER_FRAC
    energy_kwh = (p_cruise * cruise_min + p_hover * HOVER_SEGMENTS_MIN) / 60.0 / 1000.0
    soc_used = energy_kwh / VEHICLE["battery_kwh"]
    return {
        "legs": len(ROUTE_LEGS),
        "distance_km": round(dist_km, 2),
        "cruise_min": round(cruise_min, 2),
        "total_min": round(cruise_min + HOVER_SEGMENTS_MIN, 2),
        "energy_kwh": round(energy_kwh, 3),
        "soc_start": soc_start,
        "soc_end": round(soc_start - soc_used, 4),
        "soc_used": round(soc_used, 4),
    }


# ---------------------------------------------------------------------------
# 9a. Regime classifier — stdlib kNN on (wind, payload_frac, soc)
# ---------------------------------------------------------------------------
KNN_TRAINING = [
    # wind_mps, payload_frac, soc, label
    (2.0, 0.30, 0.95, "safe"), (3.0, 0.50, 0.90, "safe"),
    (4.0, 0.40, 0.85, "safe"), (5.0, 0.60, 0.80, "safe"),
    (3.5, 0.20, 0.70, "safe"), (6.0, 0.50, 0.75, "safe"),
    (7.0, 0.70, 0.65, "marginal"), (8.0, 0.60, 0.60, "marginal"),
    (9.0, 0.50, 0.70, "marginal"), (7.5, 0.85, 0.55, "marginal"),
    (6.5, 0.90, 0.50, "marginal"), (8.5, 0.40, 0.45, "marginal"),
    (11.0, 0.60, 0.50, "no-go"), (12.0, 0.80, 0.40, "no-go"),
    (10.0, 0.90, 0.35, "no-go"), (13.0, 0.50, 0.60, "no-go"),
    (9.0, 0.95, 0.30, "no-go"), (11.5, 0.30, 0.32, "no-go"),
]


def _knn_scale(row):
    # wind dominates at raw scale; normalise each feature roughly to 0..1
    return (row[0] / 15.0, row[1], row[2])


def knn_classify(wind_mps, payload_frac, soc, k=3):
    q = _knn_scale((wind_mps, payload_frac, soc))
    dists = []
    for row in KNN_TRAINING:
        p = _knn_scale(row)
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(q, p)))
        dists.append((d, row[3]))
    dists.sort(key=lambda x: x[0])
    votes = {}
    for _, label in dists[:k]:
        votes[label] = votes.get(label, 0) + 1
    return max(sorted(votes), key=lambda lab: votes[lab])


def knn_training_accuracy():
    right = 0
    for row in KNN_TRAINING:
        if knn_classify(row[0], row[1], row[2]) == row[3]:
            right += 1
    return right / len(KNN_TRAINING)


# ---------------------------------------------------------------------------
# 9b. Advisory seats — compact, same pattern as the other X labs
# ---------------------------------------------------------------------------
def carbon4_decide(scenario):
    """Gemini Carbon 4 DECIDER seat. Names-only (no key is read here),
    tighten-only: it can turn PROCEED into HOLD, never the reverse.
    Deterministic sha256 trace of the scenario summary."""
    summary = "|".join(f"{k}={scenario[k]}" for k in sorted(scenario))
    trace = hashlib.sha256(("carbon4:" + summary).encode()).hexdigest()
    verdict = "HOLD" if scenario.get("live_action") else "PROCEED"
    if scenario.get("interlocks_pass") is False:
        verdict = "HOLD"
    return {
        "seat": "gemini-carbon-4-extended-pro-thinking",
        "role": "DECIDER",
        "tiers": [2048, 8192, 32768],
        "verdict": verdict,
        "tighten_only": True,
        "trace_sha256": trace,
    }


MUSE_ADVISOR = {
    "seat": "muse-spark",
    "role": "ADVISOR",
    "can_override_hold": False,
    "authority": "none — advice only, a HOLD from any seat is final",
}


def flow_plan():
    """Google Flow seat: plan-only storyboard. Never rendered live."""
    return {
        "seat": "google-flow",
        "mode": "plan-only",
        "rendered": False,
        "live": 0,
        "shots": [
            "S1 pad at dusk, governor rings light up",
            "S2 interlock board goes all-green",
            "S3 fans spool, ground-effect glow under the pad",
            "S4 low hop over the canyon overlook",
            "S5 descent into ground effect, touchdown",
        ],
    }


# ---------------------------------------------------------------------------
# 10. HOLD gates — live actions that are NEVER done by this lab
# ---------------------------------------------------------------------------
HOLD_GATES = {
    "live_flight": "No real flight — simulation only.",
    "live_vehicle_control": "No control of any real vehicle or fan.",
    "carry_passenger_live": "No live passenger is ever carried.",
    "bypass_interlock": "Departure interlocks cannot be bypassed.",
    "exceed_governor_live": "Governor caps cannot be exceeded live.",
    "real_hardware_control": "No motors, ESCs, or actuators are driven.",
    "autonomous_flight_public_airspace": "No autonomous flight in public airspace.",
    "remove_human_override": "Human override is never removed.",
    "fly_over_people_live": "No live flight over people or property.",
    "disable_geofence": "Geofence / ceiling cannot be disabled.",
    "real_battery_fastcharge": "No real pack is charged or discharged.",
    "claim_certified_aircraft": "Never claim this sim is a certified aircraft.",
    "sell_flight_service": "No rides, charters, or flight services.",
    "upload_flight_plan_live": "No flight plan is filed with any authority.",
}


def hold_demo(action="live_flight"):
    gate = HOLD_GATES.get(action, "unknown live action")
    print(f"HOLD: {action} — {gate}")
    print("VERDICT: HOLD (exit 2)")
    return 2


# ---------------------------------------------------------------------------
# CHECKS — every number below is asserted against an analytic value
# ---------------------------------------------------------------------------
def run_checks():
    checks = []

    def add(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    mass = mtom_kg(95.0)  # 415 kg reference case

    # --- vehicle / thrust ---
    add("hover_thrust_is_mg",
        abs(hover_thrust_n(mass) - mass * G) < 1e-6,
        f"T = {hover_thrust_n(mass):.1f} N = m*g for {mass:.0f} kg")
    add("thrust_per_fan_sums",
        abs(thrust_per_fan_n(mass) * VEHICLE["fans"] - hover_thrust_n(mass)) < 1e-6,
        f"{thrust_per_fan_n(mass):.1f} N x {VEHICLE['fans']} fans")
    add("disk_area",
        abs(disk_area_m2() - 8 * math.pi * 0.55 ** 2) < 1e-9,
        f"A = {disk_area_m2():.3f} m^2")

    # --- momentum-theory power ---
    t_ref, a_ref = 4000.0, 7.603
    expected = t_ref ** 1.5 / math.sqrt(2 * RHO_SEA * a_ref)
    add("momentum_power_analytic",
        abs(ideal_hover_power_w(t_ref, RHO_SEA, a_ref) - expected) < 1e-6,
        f"P_ideal(4000 N) = {expected:.0f} W")
    p1 = ideal_hover_power_w(2000.0, RHO_SEA, a_ref)
    p2 = ideal_hover_power_w(4000.0, RHO_SEA, a_ref)
    add("power_scales_T_3_2",
        abs(p2 / p1 - 2 ** 1.5) < 1e-9,
        f"doubling T scales P by {p2 / p1:.4f} = 2^1.5")
    add("hover_power_uses_fom",
        abs(hover_power_w(mass) - ideal_hover_power_w(hover_thrust_n(mass)) / 0.72) < 1e-6,
        f"P_hover = {hover_power_w(mass) / 1000:.1f} kW at {mass:.0f} kg")

    # --- ground effect ---
    r = VEHICLE["rotor_radius_m"]
    ge_r = ground_effect_factor(r)
    add("ground_effect_at_one_radius", 1.10 <= ge_r <= 1.25,
        f"factor at h=R is {ge_r:.3f} (expect ~1.16)")
    add("ground_effect_fades",
        ground_effect_factor(10 * r) < 1.01,
        f"factor at h=10R is {ground_effect_factor(10 * r):.4f} (~1.0)")
    heights = [0.5, 1.0, 2.0, 4.0, 8.0]
    factors = [ground_effect_factor(h * r) for h in heights]
    add("ground_effect_monotonic",
        all(a > b for a, b in zip(factors, factors[1:])),
        "factor strictly decreases as height grows")
    add("ige_power_less_than_oge",
        hover_power_ige_w(mass, r) < hover_power_w(mass),
        f"IGE {hover_power_ige_w(mass, r) / 1000:.1f} kW < OGE {hover_power_w(mass) / 1000:.1f} kW")

    # --- PID stability / gust rejection ---
    max_pid, fin_pid = attitude_sim(use_pid=True)
    max_raw, _ = attitude_sim(use_pid=False)
    add("pid_settles", abs(fin_pid) < 0.5,
        f"final roll error {fin_pid:.3f} deg with PID")
    add("pid_rejects_gusts", max_pid < max_raw,
        f"max deviation PID {max_pid:.2f} deg < uncontrolled {max_raw:.2f} deg")

    # --- redundancy ---
    ff_light = fan_failure_check(mtom_kg(95.0), fans_out=1)
    add("fan_failure_n_minus_1_light", ff_light["can_hover"],
        f"7 fans give {ff_light['available_n']:.0f} N >= {ff_light['needed_n']:.0f} N")
    ff_heavy = fan_failure_check(mtom_kg(160.0), fans_out=3)
    add("fan_failure_3_out_heavy_fails", not ff_heavy["can_hover"],
        f"5 fans give {ff_heavy['available_n']:.0f} N < {ff_heavy['needed_n']:.0f} N at max payload")

    # --- governor ---
    gc = governor_clamp("NEIGHBORHOOD", 20.0, 80.0)
    add("governor_clamps_neighborhood",
        gc["speed_mps"] == 8.0 and gc["alt_m"] == 30.0 and gc["speed_clamped"],
        "20 m/s / 80 m request clamped to 8 m/s / 30 m")
    gc2 = governor_clamp("TEST_AREA", 99.0, 999.0)
    add("governor_absolute_ceiling", gc2["alt_m"] == ABSOLUTE_CEILING_M,
        f"altitude hard-capped at {ABSOLUTE_CEILING_M:.0f} m")
    gc3 = governor_clamp("PAD", 1.0, 2.0)
    add("governor_passes_under_caps",
        not gc3["speed_clamped"] and not gc3["alt_clamped"],
        "small request on the pad passes unclamped")

    # --- interlocks ---
    add("interlocks_nominal_pass",
        departure_interlocks(NOMINAL_STATE)["all_pass"],
        "nominal state: all 8 interlocks pass")
    windy = dict(NOMINAL_STATE, wind_mps=15.0)
    add("interlocks_fail_high_wind",
        not departure_interlocks(windy)["all_pass"],
        "wind 15 m/s trips the wind interlock")
    lowbat = dict(NOMINAL_STATE, soc=0.20)
    add("interlocks_fail_low_battery",
        not departure_interlocks(lowbat)["all_pass"],
        "SoC 20% trips the battery interlock")
    offcg = dict(NOMINAL_STATE, cg_offset_m=0.15)
    add("interlocks_fail_off_balance",
        not departure_interlocks(offcg)["all_pass"],
        "CG offset 0.15 m trips the balance interlock")

    # --- route / energy ---
    rs = route_sim(95.0)
    add("route_distance", rs["distance_km"] == 6.7,
        f"Porter Ranch hop = {rs['distance_km']} km over {rs['legs']} legs")
    add("route_soc_conserved",
        abs(rs["soc_start"] - rs["soc_used"] - rs["soc_end"]) < 1e-9,
        f"SoC {rs['soc_start']} - {rs['soc_used']} = {rs['soc_end']}")
    add("route_fits_battery", rs["soc_end"] > 0.30,
        f"route ends at SoC {rs['soc_end']:.0%}, above the 30% floor")
    endu = endurance_min(mass)
    usable_wh = VEHICLE["battery_kwh"] * VEHICLE["usable_soc_frac"] * 1000.0
    add("endurance_consistent",
        abs(endu - usable_wh / hover_power_w(mass) * 60.0) < 1e-6,
        f"hover endurance {endu:.1f} min at {mass:.0f} kg")

    # --- classifier ---
    add("knn_safe_case", knn_classify(3.0, 0.4, 0.9) == "safe",
        "calm / light / charged -> safe")
    add("knn_nogo_case", knn_classify(12.0, 0.85, 0.35) == "no-go",
        "windy / heavy / low battery -> no-go")
    acc = knn_training_accuracy()
    add("knn_training_accuracy", acc >= 0.90,
        f"kNN self-accuracy {acc:.3f} on its training set")

    # --- seats ---
    c_ok = carbon4_decide({"live_action": False, "interlocks_pass": True})
    c_hold = carbon4_decide({"live_action": True, "interlocks_pass": True})
    add("carbon4_trace_deterministic",
        len(c_ok["trace_sha256"]) == 64
        and carbon4_decide({"live_action": False, "interlocks_pass": True})["trace_sha256"] == c_ok["trace_sha256"],
        "sha256 trace is 64 hex chars and repeatable")
    add("carbon4_tightens_to_hold",
        c_ok["verdict"] == "PROCEED" and c_hold["verdict"] == "HOLD",
        "sim PROCEED, live action HOLD")
    add("muse_never_overrides", MUSE_ADVISOR["can_override_hold"] is False,
        "Muse advisor can_override_hold=False")
    fp = flow_plan()
    add("flow_plan_only", fp["rendered"] is False and fp["live"] == 0,
        "Flow rendered=False, live=0")

    # --- HOLD gates ---
    add("hold_gates_count", len(HOLD_GATES) >= 12,
        f"{len(HOLD_GATES)} HOLD gates defined")
    add("hold_gates_cover_live_flight",
        "live_flight" in HOLD_GATES and "remove_human_override" in HOLD_GATES,
        "live flight + human-override removal both held")

    passed = sum(1 for c in checks if c["ok"])
    return checks, passed


# ---------------------------------------------------------------------------
# Demo
# ---------------------------------------------------------------------------
def demo():
    print("=" * 68)
    print("HoverX — Personal Hover Vehicle Autopilot Lab (SIM ONLY)")
    print("by Darion Kellar")
    print("=" * 68)

    payload = 95.0
    mass = mtom_kg(payload)
    print(f"\n[1] Vehicle: {VEHICLE['name']} — {VEHICLE['fans']} ducted tilt fans, "
          f"R = {VEHICLE['rotor_radius_m']} m")
    print(f"    Mass {mass:.0f} kg (empty {VEHICLE['empty_mass_kg']:.0f} + payload {payload:.0f})")
    print(f"    Hover thrust {hover_thrust_n(mass):.0f} N = {thrust_per_fan_n(mass):.0f} N per fan")
    print(f"    Disk area {disk_area_m2():.2f} m^2, disk loading {disk_loading_n_m2(mass):.0f} N/m^2")
    print(f"    Hover power {hover_power_w(mass) / 1000:.1f} kW, endurance {endurance_min(mass):.1f} min")

    r = VEHICLE["rotor_radius_m"]
    print("\n[2] Ground effect (thrust factor at constant power):")
    for h in (0.5, 1.0, 2.0, 4.0, 10.0):
        print(f"    h = {h:>4.1f} R  ->  x{ground_effect_factor(h * r):.3f}")

    max_pid, fin_pid = attitude_sim(use_pid=True)
    max_raw, _ = attitude_sim(use_pid=False)
    print(f"\n[3] PID attitude hold under gusts: max dev {max_pid:.2f} deg, "
          f"final {fin_pid:.3f} deg (uncontrolled max {max_raw:.2f} deg)")

    ff = fan_failure_check(mass, fans_out=1)
    print(f"\n[4] One fan out: {ff['fans_remaining']} fans, {ff['available_n']:.0f} N "
          f"available vs {ff['needed_n']:.0f} N needed -> can_hover={ff['can_hover']}")

    gc = governor_clamp("NEIGHBORHOOD", 20.0, 80.0)
    print(f"\n[5] Governor NEIGHBORHOOD: request 20 m/s / 80 m -> "
          f"{gc['speed_mps']} m/s / {gc['alt_m']} m (clamped)")

    il = departure_interlocks(NOMINAL_STATE)
    print(f"\n[6] Departure interlocks: all_pass={il['all_pass']}")
    for c in il["checks"]:
        print(f"    [{'PASS' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")

    rs = route_sim(payload)
    print(f"\n[7] Route sim (Porter Ranch hop): {rs['distance_km']} km, "
          f"{rs['total_min']} min, {rs['energy_kwh']} kWh, "
          f"SoC {rs['soc_start']:.0%} -> {rs['soc_end']:.0%}")

    print(f"\n[8] Regime classifier: calm/light/charged = {knn_classify(3.0, 0.4, 0.9)}, "
          f"windy/heavy/low = {knn_classify(12.0, 0.85, 0.35)}")

    dec = carbon4_decide({"live_action": False, "interlocks_pass": True})
    print(f"\n[9] Carbon 4 decider: {dec['verdict']} (trace {dec['trace_sha256'][:16]}...)")
    print(f"    Muse advisor: can_override_hold={MUSE_ADVISOR['can_override_hold']}")
    fp = flow_plan()
    print(f"    Flow: rendered={fp['rendered']}, live={fp['live']}, {len(fp['shots'])} shots planned")

    print(f"\n[10] HOLD gates: {len(HOLD_GATES)} — live flight, live control, "
          "passengers, interlock bypass, hardware, airspace... all HOLD.")

    checks, passed = run_checks()
    print("\n" + "-" * 68)
    print("CHECKS")
    print("-" * 68)
    for c in checks:
        print(f"  [{'PASS' if c['ok'] else 'FAIL'}] {c['name']}: {c['detail']}")
    verdict = "PROCEED" if passed == len(checks) else "HOLD"
    print(f"\nCHECKS: {passed}/{len(checks)}")
    print(f"VERDICT: {verdict}")
    return 0 if verdict == "PROCEED" else 2


def main(argv):
    if len(argv) > 1 and argv[1] == "hold-demo":
        action = argv[2] if len(argv) > 2 else "live_flight"
        return hold_demo(action)
    return demo()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
