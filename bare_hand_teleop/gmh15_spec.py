"""GMH15 command layout, as observed through the existing teleop server (see ../hand_client.py
and ../auto_test_full_after_patch.log), cross-checked against vendor SDK 3.9.0 docs and URDF.

Command vector: 11 angles in degrees, teleop-server order.
  0 index MCP   1 index PIP*   2 middle MCP   3 middle PIP*
  4 ring MCP    5 ring PIP*    6 little MCP   7 little PIP*
  8 thumb CMC   9 thumb MCP   10 thumb IP          (* the current teleop server rejects != 0)

Vendor SDK 3.9.0 (H15 API Reference, sec. 4) calls all 11 "adjustable joints" (each finger
joint1 proximal + joint2 distal, 0-80 deg; thumb 0-109.6 / 0-87 / 0-120). There are only 7 motors:
the offline SDK IK (tools/ik_coupling_probe.py) maps BOTH finger joints into ONE motor (tendon),
so each finger has 1 controllable DOF and the MCP/PIP split is set by the mechanism. Thumb
joints map 1:1 to motors 5/6/7. The "PIP passive" rule is a choice in this project's teleop
server (patch_pip_passive.py), not a vendor statement.

Feedback `joints_degrees` has 15 entries (5 fingers x 3); motor-driven ones are at
FEEDBACK_INDEX below. `motor_positions` has 7 raw counts in ACTUATED order.
"""
import numpy as np

N_CMD = 11
JOINT_NAMES = ("index_mcp", "index_pip", "middle_mcp", "middle_pip", "ring_mcp", "ring_pip",
               "little_mcp", "little_pip", "thumb_cmc", "thumb_mcp", "thumb_ip")
PASSIVE = (1, 3, 5, 7)
ACTUATED = (0, 2, 4, 6, 8, 9, 10)
FINGER_CMD = {"index": 0, "middle": 2, "ring": 4, "little": 6}
THUMB_CMD = {"thumb_cmc": 8, "thumb_mcp": 9, "thumb_ip": 10}
# command index -> index in the 15-entry feedback list
FEEDBACK_INDEX = {0: 0, 2: 3, 4: 6, 6: 9, 8: 12, 9: 13, 10: 14}

# Ranges that have actually been commanded on the real right hand without faults
# (auto_test logs: fingers 45 deg half-fist, thumb joints 30 deg). These are deliberately
# conservative defaults, NOT mechanical limits. Widen only after checking the server's
# `limits_degrees` and testing on hardware. The HTTP driver further intersects with the
# server-reported limits.
VERIFIED_RANGE_DEG = {
    0: (0.0, 45.0), 2: (0.0, 45.0), 4: (0.0, 45.0), 6: (0.0, 45.0),
    8: (0.0, 30.0), 9: (0.0, 30.0), 10: (0.0, 30.0),
}


# Vendor URDF / SDK real-angle limits (deg). Mechanical envelope, not a safe default.
URDF_LIMIT_DEG = {0: (0.0, 80.0), 1: (0.0, 81.6), 2: (0.0, 80.0), 3: (0.0, 81.6), 4: (0.0, 80.0),
                  5: (0.0, 81.6), 6: (0.0, 80.0), 7: (0.0, 81.6), 8: (0.0, 109.6), 9: (0.0, 87.0),
                  10: (0.0, 120.0)}


# Measured hardware envelope (2026-11-07 motor sweep): fingers full; thumb MCP/IP below their
# mechanical end stops. Used for the sim guard so the sim never shows an unreachable pose.
HARDWARE_LIMIT_DEG = {**URDF_LIMIT_DEG, 8: (0.0, 109.2), 9: (0.0, 63.0), 10: (0.0, 84.3)}


def range_arrays(ranges=None):
    """Return (lo, hi) arrays of length N_CMD; passive joints pinned to 0."""
    ranges = ranges or VERIFIED_RANGE_DEG
    lo = np.zeros(N_CMD)
    hi = np.zeros(N_CMD)
    for i, (a, b) in ranges.items():
        lo[int(i)], hi[int(i)] = float(a), float(b)
    return lo, hi


def measured_cmd_from_feedback(joints15):
    """Map 15-entry feedback to the 11-entry command layout (passive -> 0)."""
    q = np.zeros(N_CMD)
    for c, f in FEEDBACK_INDEX.items():
        q[c] = float(joints15[f])
    return q
