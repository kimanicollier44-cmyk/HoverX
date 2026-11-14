"""HoverX tests — physics and gates, all against analytic values."""
import math

import main as hx


# --- vehicle / thrust -------------------------------------------------------
def test_hover_thrust_is_mg():
    assert abs(hx.hover_thrust_n(415.0) - 415.0 * hx.G) < 1e-6


def test_thrust_per_fan_sums_to_total():
    total = hx.thrust_per_fan_n(415.0) * hx.VEHICLE["fans"]
    assert abs(total - hx.hover_thrust_n(415.0)) < 1e-6


def test_disk_area():
    assert abs(hx.disk_area_m2() - 8 * math.pi * 0.55 ** 2) < 1e-9


def test_mtom():
    assert hx.mtom_kg(160.0) == 480.0


# --- power ------------------------------------------------------------------
def test_momentum_power_formula():
    expected = 4000.0 ** 1.5 / math.sqrt(2 * hx.RHO_SEA * 7.603)
    assert abs(hx.ideal_hover_power_w(4000.0, hx.RHO_SEA, 7.603) - expected) < 1e-6


def test_power_scales_with_T_3_2():
    p1 = hx.ideal_hover_power_w(2000.0, hx.RHO_SEA, 7.603)
    p2 = hx.ideal_hover_power_w(4000.0, hx.RHO_SEA, 7.603)
    assert abs(p2 / p1 - 2 ** 1.5) < 1e-9


def test_heavier_needs_more_power():
    assert hx.hover_power_w(480.0) > hx.hover_power_w(415.0)


def test_endurance_positive_and_bounded():
    e = hx.endurance_min(415.0)
    assert 5.0 < e < 120.0


# --- ground effect ----------------------------------------------------------
def test_ground_effect_at_one_radius():
    assert 1.10 <= hx.ground_effect_factor(hx.VEHICLE["rotor_radius_m"]) <= 1.25


def test_ground_effect_fades_with_height():
    r = hx.VEHICLE["rotor_radius_m"]
    assert hx.ground_effect_factor(10 * r) < 1.01


def test_ground_effect_monotonic():
    r = hx.VEHICLE["rotor_radius_m"]
    vals = [hx.ground_effect_factor(h * r) for h in (0.5, 1.0, 2.0, 4.0, 8.0)]
    assert all(a > b for a, b in zip(vals, vals[1:]))


def test_ige_power_less_than_oge():
    r = hx.VEHICLE["rotor_radius_m"]
    assert hx.hover_power_ige_w(415.0, r) < hx.hover_power_w(415.0)


# --- PID / gusts ------------------------------------------------------------
def test_pid_settles():
    _, final = hx.attitude_sim(use_pid=True)
    assert abs(final) < 0.5


def test_pid_beats_uncontrolled():
    max_pid, _ = hx.attitude_sim(use_pid=True)
    max_raw, _ = hx.attitude_sim(use_pid=False)
    assert max_pid < max_raw


def test_attitude_sim_deterministic():
    assert hx.attitude_sim() == hx.attitude_sim()


# --- redundancy -------------------------------------------------------------
def test_one_fan_out_still_hovers_light():
    assert hx.fan_failure_check(hx.mtom_kg(95.0), fans_out=1)["can_hover"]


def test_three_fans_out_heavy_cannot_hover():
    assert not hx.fan_failure_check(hx.mtom_kg(160.0), fans_out=3)["can_hover"]


# --- governor ---------------------------------------------------------------
def test_governor_clamps_neighborhood():
    gc = hx.governor_clamp("NEIGHBORHOOD", 20.0, 80.0)
    assert gc["speed_mps"] == 8.0 and gc["alt_m"] == 30.0


def test_governor_absolute_ceiling():
    assert hx.governor_clamp("TEST_AREA", 99.0, 999.0)["alt_m"] == hx.ABSOLUTE_CEILING_M


def test_governor_under_caps_unclamped():
    gc = hx.governor_clamp("PAD", 1.0, 2.0)
    assert not gc["speed_clamped"] and not gc["alt_clamped"]


# --- interlocks -------------------------------------------------------------
def test_interlocks_nominal_pass():
    assert hx.departure_interlocks(hx.NOMINAL_STATE)["all_pass"]


def test_interlocks_fail_high_wind():
    state = dict(hx.NOMINAL_STATE, wind_mps=15.0)
    assert not hx.departure_interlocks(state)["all_pass"]


def test_interlocks_fail_low_battery():
    state = dict(hx.NOMINAL_STATE, soc=0.20)
    assert not hx.departure_interlocks(state)["all_pass"]


def test_interlocks_fail_unbalanced():
    state = dict(hx.NOMINAL_STATE, cg_offset_m=0.15)
    assert not hx.departure_interlocks(state)["all_pass"]


# --- route ------------------------------------------------------------------
def test_route_distance():
    assert hx.route_sim(95.0)["distance_km"] == 6.7


def test_route_soc_bookkeeping():
    rs = hx.route_sim(95.0)
    assert abs(rs["soc_start"] - rs["soc_used"] - rs["soc_end"]) < 1e-9
    assert rs["soc_end"] > 0.30


# --- classifier -------------------------------------------------------------
def test_knn_safe():
    assert hx.knn_classify(3.0, 0.4, 0.9) == "safe"


def test_knn_nogo():
    assert hx.knn_classify(12.0, 0.85, 0.35) == "no-go"


def test_knn_accuracy():
    assert hx.knn_training_accuracy() >= 0.90


# --- seats ------------------------------------------------------------------
def test_carbon4_sim_proceed_live_hold():
    assert hx.carbon4_decide({"live_action": False, "interlocks_pass": True})["verdict"] == "PROCEED"
    assert hx.carbon4_decide({"live_action": True, "interlocks_pass": True})["verdict"] == "HOLD"


def test_carbon4_blocks_failed_interlocks():
    assert hx.carbon4_decide({"live_action": False, "interlocks_pass": False})["verdict"] == "HOLD"


def test_carbon4_trace_deterministic():
    a = hx.carbon4_decide({"live_action": False, "interlocks_pass": True})
    b = hx.carbon4_decide({"live_action": False, "interlocks_pass": True})
    assert a["trace_sha256"] == b["trace_sha256"] and len(a["trace_sha256"]) == 64


def test_muse_never_overrides_hold():
    assert hx.MUSE_ADVISOR["can_override_hold"] is False


def test_flow_plan_only():
    fp = hx.flow_plan()
    assert fp["rendered"] is False and fp["live"] == 0


# --- HOLD gates -------------------------------------------------------------
def test_hold_gates():
    assert len(hx.HOLD_GATES) >= 12
    for gate in ("live_flight", "live_vehicle_control", "carry_passenger_live",
                 "bypass_interlock", "exceed_governor_live", "real_hardware_control",
                 "autonomous_flight_public_airspace", "remove_human_override"):
        assert gate in hx.HOLD_GATES


def test_hold_demo_exits_2(capsys):
    assert hx.hold_demo("live_flight") == 2
    assert "HOLD" in capsys.readouterr().out


def test_all_checks_pass():
    checks, passed = hx.run_checks()
    assert passed == len(checks)
