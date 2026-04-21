#!/usr/bin/env python3
"""
prototype_dls_arm_ik.py — offline prototype for position-based ASSIST mode.

Goal: replace the rate-based ASSIST arm pipeline (B/S/G) with a position-based
one driven by the head quaternion, using Damped Least Squares (DLS) with
adaptive lambda (Nakamura/Hanafusa singularity-robust inverse) for stability
over precision.

Scope:
  - Arm only: 3 DoF (BASE/Z, SPALLA/Y, GOMITO/Y). Wrist (Y/P/R) unchanged.
  - Offline: no robot interaction. Pure math on POE model + scripted inputs.

Deliverable:
  1. DLS IK core (resolved-rate one-shot per frame).
  2. Head-quat → target-position mapper (gaze direction × reach).
  3. Battery of tests: continuity, singularity behavior, sign-convention
     verification (user observed: head DOWN → IMU pitch +, head UP → IMU pitch -).
  4. Metrics printed + written to /tmp/dls_ik_prototype.json.

No commits, no live-config changes, no service restarts.
"""
import math
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# POE model (copied from config_runtime/kinematics/j5_poe_params.json + ik_solver.py)
# ---------------------------------------------------------------------------
POE_SCREWS = np.array([
    [0, 0, 1, 0.0,    0.0,   0.0],   # BASE   — Z rotation at origin
    [0, 1, 0, -0.094, 0.0,   0.0],   # SPALLA — Y rotation at z=94mm
    [0, 1, 0, -0.154, 0.0,   0.0],   # GOMITO — Y rotation at z=154mm
    [0, 0, 1, 0.0,    0.0,   0.0],   # YAW
    [0, 1, 0, -0.311, 0.0,   0.0],   # PITCH
    [1, 0, 0, 0.0,    0.311, 0.0],   # ROLL
], dtype=float)
POE_M = np.array([
    [1, 0, 0, 0.060],
    [0, 1, 0, 0.000],
    [0, 0, 1, 0.311],
    [0, 0, 0, 1.000],
], dtype=float)

# Joint limits (virtual degrees → math-space radians, 0 rad = virtual 90°)
JOINT_LIMITS_VIRT = np.array([
    [45.0, 135.0],   # BASE
    [30.0, 145.0],   # SPALLA
    [30.0, 145.0],   # GOMITO
], dtype=float)
JOINT_LIMITS_RAD = np.radians(JOINT_LIMITS_VIRT - 90.0)

HOME_Q_RAD = np.zeros(3, dtype=float)  # math-space HOME for B/S/G

# ---------------------------------------------------------------------------
# POE FK (minimal port from ik_solver.py, position only for 3-DoF subproblem)
# ---------------------------------------------------------------------------
def _skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], dtype=float)

def _matrix_exp6(se3):
    W = se3[:3, :3]
    v = se3[:3, 3]
    wt = np.array([W[2, 1], W[0, 2], W[1, 0]], dtype=float)
    th = float(np.linalg.norm(wt))
    T = np.eye(4, dtype=float)
    if th < 1e-12:
        T[:3, 3] = v
        return T
    Wn = W / th
    Rm = np.eye(3) + math.sin(th) * Wn + (1 - math.cos(th)) * (Wn @ Wn)
    G = np.eye(3) * th + (1 - math.cos(th)) * Wn + (th - math.sin(th)) * (Wn @ Wn)
    T[:3, :3] = Rm
    T[:3, 3] = G @ (v / th)
    return T

def fk_full_math(q6_rad):
    """Full 6-DoF FK in math space (0 rad = virtual 90°). Returns 4x4 SE(3) m."""
    q = np.asarray(q6_rad, dtype=float).ravel()
    T = np.eye(4, dtype=float)
    for i in range(6):
        se3 = np.zeros((4, 4), dtype=float)
        se3[:3, :3] = _skew(POE_SCREWS[i, :3])
        se3[:3, 3] = POE_SCREWS[i, 3:]
        T = T @ _matrix_exp6(se3 * float(q[i]))
    return T @ POE_M

def fk_arm_pos(q3_rad):
    """EE position from arm joints only (wrist at HOME). Returns (3,) m."""
    q6 = np.zeros(6, dtype=float)
    q6[:3] = q3_rad
    return fk_full_math(q6)[:3, 3]

def jacobian_pos_arm(q3_rad, eps=1e-5):
    """Numerical 3x3 ∂p/∂q for position, arm only."""
    p0 = fk_arm_pos(q3_rad)
    J = np.zeros((3, 3), dtype=float)
    for i in range(3):
        qp = q3_rad.copy(); qp[i] += eps
        J[:, i] = (fk_arm_pos(qp) - p0) / eps
    return J

def manipulability(J):
    """Yoshikawa's measure: sqrt(det(J J^T)). =0 at singular."""
    return float(math.sqrt(max(0.0, np.linalg.det(J @ J.T))))

# ---------------------------------------------------------------------------
# Head quaternion → target EE position  (RELATIVE formulation)
# ---------------------------------------------------------------------------
# Identity head quat → target = HOME_ee (arm stays at home, no net motion).
# Non-identity head quat → target = HOME_ee + K · (head_dir − forward_base_at_home).
# This keeps every target inside a reachable sphere of radius K around HOME_ee,
# mirrors the wrist pipeline (head identity → wrist identity = no motion),
# and gives a linear "reactivity" parameter K (m of EE displacement per unit
# of direction-vector deviation) that the operator can tune to taste.
SHOULDER_M = np.array([0.0, 0.0, 0.094], dtype=float)
FORWARD_BASE_AT_HOME = np.array([1.0, 0.0, 0.0], dtype=float)  # EE forward axis at HOME

# VR quat arrives as (w, x, y, z). scipy expects (x, y, z, w).
def head_quat_to_direction_base(head_wxyz):
    w, x, y, z = head_wxyz
    Rh = R.from_quat([x, y, z, w]).as_matrix()
    return Rh @ FORWARD_BASE_AT_HOME

def head_quat_to_target(head_wxyz, origin_m, gain_m):
    """Relative mapping: target = origin + gain * (head_dir − forward_base_at_home).
    At head identity head_dir = forward_base_at_home → target = origin."""
    d = head_quat_to_direction_base(head_wxyz)
    return np.asarray(origin_m, dtype=float) + float(gain_m) * (d - FORWARD_BASE_AT_HOME)

# ---------------------------------------------------------------------------
# DLS IK step (resolved-rate with adaptive damping)
# ---------------------------------------------------------------------------
def dls_step(
    q_cur_rad,
    p_target_m,
    *,
    lambda_max=0.08,           # damping at deep singularity (higher = more stable, less precise)
    manip_thresh=5e-4,         # below this, ramp lambda up to lambda_max (Nakamura/Hanafusa)
    max_dx_m=0.03,             # clamp cartesian step per frame (30 mm)
    max_dq_rad=math.radians(4),# clamp per-joint step per frame (4°)
    joint_lo=JOINT_LIMITS_RAD[:, 0],
    joint_hi=JOINT_LIMITS_RAD[:, 1],
):
    p_cur = fk_arm_pos(q_cur_rad)
    J = jacobian_pos_arm(q_cur_rad)
    dx = np.asarray(p_target_m, dtype=float) - p_cur
    mag = float(np.linalg.norm(dx))
    if mag > max_dx_m:
        dx = dx * (max_dx_m / mag)

    JJT = J @ J.T
    manip = math.sqrt(max(0.0, np.linalg.det(JJT)))
    if manip < manip_thresh:
        # Smooth ramp: λ² grows quadratically as manip → 0.
        k = 1.0 - manip / manip_thresh
        lam_sq = (lambda_max * k) ** 2
    else:
        lam_sq = 0.0

    try:
        dq = J.T @ np.linalg.solve(JJT + lam_sq * np.eye(3), dx)
    except np.linalg.LinAlgError:
        # Absolute fallback: heavy damping, guaranteed solvable.
        dq = J.T @ np.linalg.solve(JJT + (lambda_max ** 2) * np.eye(3), dx)

    # Per-joint clamp (even if DLS already bounds via damping, belt & suspenders)
    dq = np.clip(dq, -max_dq_rad, max_dq_rad)
    q_next = np.clip(q_cur_rad + dq, joint_lo, joint_hi)
    return q_next, {
        "manip": manip, "lambda_sq": lam_sq,
        "dq_rad": [float(v) for v in dq],
        "dx_cart_norm_mm": mag * 1000.0,
        "err_pos_mm": float(np.linalg.norm(fk_arm_pos(q_next) - p_target_m) * 1000.0),
    }

# ---------------------------------------------------------------------------
# Trajectory simulator: run DLS for N frames driving a moving head quat.
# ---------------------------------------------------------------------------
def simulate(head_quat_fn, *, n_frames=150, q0_rad=None, reach_m=0.20, origin_m=None):
    q = HOME_Q_RAD.copy() if q0_rad is None else np.asarray(q0_rad, dtype=float).copy()
    if origin_m is None:
        origin_m = fk_arm_pos(HOME_Q_RAD)  # HOME_ee as reference → identity head = no motion
    traj = []
    for k in range(n_frames):
        hq = head_quat_fn(k)
        tgt = head_quat_to_target(hq, origin_m, reach_m)
        q, info = dls_step(q, tgt)
        traj.append({
            "k": k,
            "q_virt_deg": [round(math.degrees(float(q[i])) + 90.0, 2) for i in range(3)],
            **{k_: round(v, 5) if isinstance(v, float) else v for k_, v in info.items()},
            "p_target_mm": [round(float(v) * 1000.0, 2) for v in tgt],
        })
    return traj

# ---------------------------------------------------------------------------
# Trajectory summaries
# ---------------------------------------------------------------------------
def traj_summary(traj, name):
    dq_max = max(max(abs(v) for v in r["dq_rad"]) for r in traj)
    manip_min = min(r["manip"] for r in traj)
    lam_max = max(r["lambda_sq"] for r in traj)
    err_final = traj[-1]["err_pos_mm"]
    err_mean_second_half = np.mean([r["err_pos_mm"] for r in traj[len(traj)//2:]])
    q_final = traj[-1]["q_virt_deg"]
    # Joint step continuity (max single-step change in q vs previous frame)
    step_max_deg = 0.0
    for i in range(1, len(traj)):
        prev = traj[i-1]["q_virt_deg"]; cur = traj[i]["q_virt_deg"]
        step_max_deg = max(step_max_deg, max(abs(cur[j]-prev[j]) for j in range(3)))
    return {
        "name": name,
        "n_frames": len(traj),
        "dq_max_rad": round(dq_max, 4),
        "step_max_virt_deg": round(step_max_deg, 3),
        "manip_min": round(manip_min, 5),
        "lambda_sq_max": round(lam_max, 5),
        "err_final_mm": round(err_final, 2),
        "err_mean_2nd_half_mm": round(float(err_mean_second_half), 2),
        "q_final_virt_deg": q_final,
    }

# ---------------------------------------------------------------------------
# Test scenarios
# ---------------------------------------------------------------------------
def q_identity(_k): return (1.0, 0.0, 0.0, 0.0)

def q_pitch_fixed(deg):
    x, y, z, w = R.from_euler("Y", deg, degrees=True).as_quat()
    return lambda _k: (w, x, y, z)

def q_yaw_fixed(deg):
    x, y, z, w = R.from_euler("Z", deg, degrees=True).as_quat()
    return lambda _k: (w, x, y, z)

def q_pitch_ramp(amp_deg, n_frames):
    def fn(k):
        # Linear ramp 0 → amp over first half, hold for second half
        if k < n_frames // 2:
            a = amp_deg * (k / (n_frames // 2))
        else:
            a = amp_deg
        x, y, z, w = R.from_euler("Y", a, degrees=True).as_quat()
        return (w, x, y, z)
    return fn

def q_pitch_sine(amp_deg, freq_hz, dt=0.02):
    def fn(k):
        a = amp_deg * math.sin(2 * math.pi * freq_hz * k * dt)
        x, y, z, w = R.from_euler("Y", a, degrees=True).as_quat()
        return (w, x, y, z)
    return fn

def q_combined_yaw_pitch(yaw_deg, pitch_deg):
    x, y, z, w = R.from_euler("ZY", [yaw_deg, pitch_deg], degrees=True).as_quat()
    return lambda _k: (w, x, y, z)

def q_yaw_ramp(amp_deg, n_frames):
    def fn(k):
        a = amp_deg * min(1.0, k / (n_frames // 2))
        x, y, z, w = R.from_euler("Z", a, degrees=True).as_quat()
        return (w, x, y, z)
    return fn

# ---------------------------------------------------------------------------
# Sign-convention verification
# ---------------------------------------------------------------------------
def verify_sign_convention():
    """User-observed rule: head DOWN (pitch +) → IMU pitch increases (EE pitches down).
    In our model: head_quat pitch +θ (R.from_euler('Y', +θ)) rotates the direction
    vector (1,0,0) such that it gains -Z component → target drops in Z → arm should
    extend downward. The IMU on the EE would see the same pitch since the wrist
    pipeline (position-based, unchanged) already follows the head quat 1:1.

    Here we just verify our *target mapping* matches that intent.
    """
    out = {}
    origin = fk_arm_pos(HOME_Q_RAD)
    gain = 0.25
    for deg in (-20, -10, 0, +10, +20):
        qx, qy, qz, qw = R.from_euler("Y", deg, degrees=True).as_quat()
        hq = (qw, qx, qy, qz)
        d = head_quat_to_direction_base(hq)
        tgt = head_quat_to_target(hq, origin, gain)
        delta = tgt - origin
        out[f"pitch_{deg:+d}"] = {
            "dir_base_xyz": [round(float(v), 4) for v in d],
            "target_mm": [round(float(v) * 1000.0, 2) for v in tgt],
            "delta_from_home_mm": [round(float(v) * 1000.0, 2) for v in delta],
            "comment": (
                "down → dZ<0" if deg > 0 else
                "up → dZ>0"   if deg < 0 else
                "flat → no motion"
            ),
        }
    return out

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 88)
    print("DLS ARM-IK PROTOTYPE — offline validation")
    print("=" * 88)
    print(f"POE HOME EE position: {fk_arm_pos(HOME_Q_RAD) * 1000} mm")
    print(f"Joint limits (virtual deg): BASE {JOINT_LIMITS_VIRT[0]}, SPALLA {JOINT_LIMITS_VIRT[1]}, GOMITO {JOINT_LIMITS_VIRT[2]}")
    print()

    # --- Sign-convention sanity ---
    print("-" * 88)
    print("SIGN-CONVENTION (head pitch mapping → target Z displacement)")
    print("User observation: head DOWN → IMU pitch POSITIVE. Model should match.")
    print("-" * 88)
    sc = verify_sign_convention()
    for k, v in sc.items():
        print(f"  {k:10s} dir={v['dir_base_xyz']}  target={v['target_mm']}  [{v['comment']}]")
    print()

    # --- Tests ---
    # Relative mapping: gain_m is the per-unit deviation in EE displacement.
    # With gain=0.25 m, a pitch of 30° produces |d-forward|=2·sin(15°)=0.52 →
    # EE displacement 130 mm. Tunable in live config later.
    GAIN = 0.25
    tests = [
        ("identity_hold",            q_identity,                            150, GAIN),
        ("pitch_down_+20_ramp",      q_pitch_ramp(+20, 120),                200, GAIN),
        ("pitch_up_-20_ramp",        q_pitch_ramp(-20, 120),                200, GAIN),
        ("yaw_+30_ramp",             q_yaw_ramp(+30, 120),                  200, GAIN),
        ("yaw_-30_ramp",             q_yaw_ramp(-30, 120),                  200, GAIN),
        ("pitch_sine_10_0.4Hz",      q_pitch_sine(10, 0.4),                 300, GAIN),
        ("yaw+pitch_combined",       q_combined_yaw_pitch(25, 15),          200, GAIN),
        ("extreme_pitch_+45",        q_pitch_fixed(+45),                    200, GAIN),
        ("extreme_pitch_-45",        q_pitch_fixed(-45),                    200, GAIN),
        ("extreme_yaw_+50",          q_yaw_fixed(+50),                      200, GAIN),
        ("higher_gain_0.40m",        q_pitch_fixed(+20),                    200, 0.40),
    ]
    results = {}
    for name, fn, n, reach in tests:
        traj = simulate(fn, n_frames=n, reach_m=reach)
        s = traj_summary(traj, name)
        results[name] = {"summary": s, "last5": traj[-5:]}
        print(f"-- {name}")
        print(f"   n={s['n_frames']}  step_max={s['step_max_virt_deg']}°  "
              f"manip_min={s['manip_min']}  lam²_max={s['lambda_sq_max']}  "
              f"err_final={s['err_final_mm']}mm  err_2nd_half={s['err_mean_2nd_half_mm']}mm")
        print(f"   q_final_virtual_deg={s['q_final_virt_deg']}")

    # --- Overall verdict ---
    print()
    print("=" * 88)
    print("VERDICT CHECKS")
    print("=" * 88)
    any_nan = False
    step_bust = []
    stuck_at_limit = []
    for name, r in results.items():
        s = r["summary"]
        if any(not math.isfinite(v) for v in s["q_final_virt_deg"]): any_nan = True
        if s["step_max_virt_deg"] > 5.0: step_bust.append((name, s["step_max_virt_deg"]))
        for ji, lim in enumerate(JOINT_LIMITS_VIRT):
            if abs(s["q_final_virt_deg"][ji] - lim[0]) < 0.5 or abs(s["q_final_virt_deg"][ji] - lim[1]) < 0.5:
                stuck_at_limit.append((name, ji, s["q_final_virt_deg"][ji]))
    print(f"  NaN encountered:                {'YES (FAIL)' if any_nan else 'no'}")
    print(f"  per-frame step > 5° cases:      {len(step_bust)}")
    for n, v in step_bust: print(f"     {n}: {v}°")
    print(f"  joint saturation at limits:     {len(stuck_at_limit)}")
    for n, ji, v in stuck_at_limit: print(f"     {n}: joint {ji} @ {v}°")

    with open("/tmp/dls_ik_prototype.json", "w") as f:
        json.dump(results, f, indent=2, default=str)
    print()
    print("Full trajectory data: /tmp/dls_ik_prototype.json")

if __name__ == "__main__":
    main()
