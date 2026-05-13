# PA1 Report — Shape-Drawing Robot

**Author:** Christian Nyamekye · **Course:** COSC 81/281 · **Date:** 2026-04-18

## Implementation

`shape_drawer.py` publishes `/cmd_vel`, subscribes to `/odom`, and uses `tf2` for `odom → base_footprint`. Three primitives (`rotate_in_place`, `drive_straight`, `drive_arc`) are the only code paths touching `/cmd_vel`; every shape is a short script over them. `drive_arc` computes `ω = v/R` and `T = |Δθ|·R/v` from the ICC model. `drive_straight` handles the `R → ∞` case directly, since `R = (L/2)(v_r+v_l)/(v_r−v_l)` diverges as `v_r → v_l`.

- **Trapezoid (simplified D):** 4-sided isosceles trapezoid. Path `start → B₁(0,r) → V₁ → V₂ → B₂(0,−r) → start` with rotations summing to `−2π`.
- **D-shape:** vertical stroke of length `2r` + semicircle of radius `r` centered at start.
- **Polygon:** each leg reads current pose via `lookup_transform`, computes `Δθ = atan2(Δy,Δx) − yaw` and `d = √(Δx²+Δy²)`, then rotates and drives.

**Closed-loop (EC):** replaces the timed loop with `ω = Kθ·eθ`, `v = min(Kd·ed, v_max)`, saturated; linear term zeroed when `|eθ| > π/6`. Tolerances 2 cm / 0.02 rad.

## Velocity Evaluation (Trapezoid, `r = 1.0 m`)

| `v` (m/s) | Final x | Final y | Final θ |
|---|---|---|---|
| 0.05 | +0.01 | −0.02 | +0.03 |
| 0.10 | +0.03 | −0.05 | +0.06 |
| 0.20 | +0.09 | −0.12 | +0.15 |

Higher `v` amplifies error because (i) the constant-velocity assumption ignores accel/decel ramps, (ii) wheel slip grows at arc/straight transitions, and (iii) at fixed 20 Hz each tick covers more ground, so start/stop rounding is larger. Open-loop has no channel to observe or correct these — so semicircle drift is a model limit, not a code bug. During development I also hit a sim-clock-staleness bug where a long blocking `input()` left `_drive_for_duration`'s cached sim time stale; the first `spin_once` inside the loop jumped time past `end_time`, so the loop exited with no motion. Fixed by pumping `spin_once` for 50 ms before computing `end_time`.

## Closed-Loop Comparison (same trapezoid, `v = 0.10`)

| Mode | Final x | Final y | Final θ |
|---|---|---|---|
| Open-loop | +0.03 | −0.05 | +0.06 |
| Closed-loop | +0.004 | −0.006 | +0.011 |

Closed-loop returns to within 1 cm / 0.01 rad because the P-controller re-measures error every tick rather than trusting a precomputed duration. Cost: ~10–15% slower and sensitive to odom noise below ~1 cm tolerance.

## Credits

Lec 02/06 node template · Lec 04 differential-drive kinematics (ICC) · Lec 05 `tf2_ros.Buffer.lookup_transform` · PA0 `RandomWalk` FSM as style template.
