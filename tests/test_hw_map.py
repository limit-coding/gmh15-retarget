"""Sim (full range) -> hardware (verified range) mapping."""
import numpy as np

from bare_hand_teleop import gmh15_spec as spec, hw_map

LO, HI = spec.range_arrays(spec.VERIFIED_RANGE_DEG)


def test_open_maps_to_zero_and_full_fist_to_limits():
    assert np.allclose(hw_map.hw_from_sim(np.zeros(11), LO, HI), 0)
    lo_u, hi_u = spec.range_arrays(spec.URDF_LIMIT_DEG)
    full = np.array([80, 81.6, 80, 81.6, 80, 81.6, 80, 81.6, 0, 0, 0.]) + np.r_[np.zeros(8), hi_u[8:]]
    out = hw_map.hw_from_sim(full, LO, HI)
    assert np.allclose(out, HI)                      # fingers 45, thumb 30, PIP 0


def test_scaling_is_proportional_not_clipping():
    cmd = np.zeros(11)
    cmd[0] = 40.0            # index halfway through MCP phase -> s = 0.25
    cmd[9] = 43.5            # thumb MCP half of 87
    out = hw_map.hw_from_sim(cmd, LO, HI, path="mcp_first")
    assert abs(out[0] - 0.25 * 45) < 0.3
    assert abs(out[9] - 15.0) < 1e-6
    assert all(out[i] == 0 for i in spec.PASSIVE)


def test_output_never_exceeds_hardware_limits():
    rng = np.random.default_rng(0)
    lo_u, hi_u = spec.range_arrays(spec.URDF_LIMIT_DEG)
    for _ in range(200):
        out = hw_map.hw_from_sim(rng.uniform(lo_u - 20, hi_u + 20), LO, HI)
        assert np.all(out >= LO - 1e-9) and np.all(out <= HI + 1e-9)


def test_counts_from_sim_matches_motor_sweep():
    assert hw_map.counts_from_sim(np.zeros(11)) == [0] * 7
    c = hw_map.counts_from_sim([40, 0, 80, 0, 80, 40, 80, 80, 60, 30, 60])
    assert abs(c[0] - 3193) < 60          # MCP 40 deg  (sweep: 3193)
    assert abs(c[1] - 6737) < 60          # MCP 80, PIP 0 (end of MCP phase)
    assert abs(c[2] - 8112) < 60          # PIP 40 deg  (sweep: 8112)
    assert c[3] == 10000
    assert abs(c[4] - 5528) < 5 and abs(c[6] - 5915) < 5


def test_counts_never_exceed_measured_ceilings():
    c = hw_map.counts_from_sim([80, 81.6] * 4 + [120, 90, 130])
    assert c == [10000, 10000, 10000, 10000, 9963, 7200, 7700]
