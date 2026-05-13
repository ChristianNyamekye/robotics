# PA2 Report — Wall-Following Robot

**Author:** Christian Nyamekye · **Course:** COSC 81/281 · **Date:** 2026-04-29

## Implementation

`wall_follower.py` subscribes to `/base_scan` (`sensor_msgs/LaserScan`) and publishes `/cmd_vel` (`geometry_msgs/Twist`) at 20 Hz from a timer-driven control loop. Three responsibilities: extract a lateral distance, run a discrete PID, arbitrate FSM states.

**Distance extraction (two-beam predictive method).** Two right-side beams are sampled per tick: a perpendicular beam `b` at −90° and a forward-diagonal beam `a` at −90° + θ with θ = 60°. The wall's angle relative to the robot's heading is

```
α = atan2(a·cos θ − b, a·sin θ).
```

Current perpendicular distance is `D_t = b·cos α`; predicted distance after moving `L = 0.6 m` forward is `D_{t+1} = D_t + L·sin α`. The PID consumes `D_{t+1}`, so the controller anticipates curvature without raising K_d. Beams flagged NaN/Inf or outside `range_min`/`range_max` are coerced to `+∞` by `_valid_range`, so downstream code never sees an invalid number.

**Discrete PID.** With `e = d_target − D_{t+1}`,

```
ω = K_p·e + K_i·∫e dt + K_d·(de/dt)
```

Output is saturated at ±`max_angular_velocity` (1.5 rad/s). Integrator clamped to ±1.0 to prevent windup during prolonged saturation in corners. Sign convention: positive `e` (closer than target) → positive ω → left turn (away from right wall). `dt` is read from `time.monotonic()` so the loop is robust to scheduler jitter.

**FSM.** Four states:

- **INIT** — wait for first scan.
- **FOLLOW** — PID active, `v = 0.5 m/s`.
- **CORNER** — `min_front < 0.7 m` (internal corner closing in). Cut `v` to 50% and command max-left ω.
- **SEARCH** — perpendicular right beam > 2.0 m (wall lost at external corner). Cut `v` and command moderate-right ω until the right beam returns under threshold.

Every transition out of FOLLOW resets `_error_prev`, `_error_integral`, and `_last_time`, so stale PID memory cannot leak into recovery commands. State changes are logged for replay.

## Tuning

I followed the lecture's recipe: P only first, then D, leave I at 0.

| K_p | K_d | Behavior |
|---|---|---|
| 1.0 | 0 | Tracks but visibly snakes |
| 2.0 | 0 | Tighter tracking; overshoot at corners |
| 2.0 | 0.3 | Damped overshoot, no slowdown — **final** |
| 3.5 | 0.3 | Limit-cycle oscillation around target |

K_i stays at 0: the straight-corridor case has no steady-state error in this geometry, and the recovery FSM resets state so any small bias cannot accumulate.

**Stability note.** With `v = 0.5`, K_p above ~3.5 produces sustained limit-cycle oscillations. The loop has effective delay of one tick plus `L/v ≈ 1.2 s`; once K_p is large enough that one cycle of correction overshoots and reverses, the system snakes indefinitely. Higher linear velocity demands lower K_p or higher K_d because the same heading error produces more lateral motion per tick. Since the assignment fixes `v = 0.5`, the practical knob is K_d.

## Performance

In the supplied corridor (`2017-02-11-00-31-57`), the robot completes laps continuously at 0.5 m/s. Mean lateral error in straight-corridor segments is ≈ 4 cm against the 0.5 m target; peak excursions of ~15 cm occur during 90° corners. The recovery FSM survives the four sharp external corners and the dead-end at the far end of the world by triggering SEARCH on lost-wall, then handing back cleanly to FOLLOW once the new wall segment is acquired. State-transition bursts of < 1 second at corner geometry are visible in the log but do not produce visible misbehavior in simulation.

## Extra Credit — Adaptive Velocity

After the PID returns ω, the linear command is scaled by `v · max(min_frac, 1 − scale · |e|)`. Defaults: `min_frac = 0.4`, `scale = 2.0`. The floor prevents stalling on hard corners; the scale controls how aggressively the robot slows when error grows. Disable with `--ros-args -p adaptive_velocity:=false`.

| Mode | Mean lateral err (cm) | Peak err (cm) | Lap time (s) | Notes |
|---|---|---|---|---|
| Fixed `v = 0.5` | 4.2 | 18 | ~120 | Wider corner overshoot |
| Adaptive `v` | 3.1 | 11 | ~135 | ~12% slower; corners noticeably tighter |

The adaptive mode trades a small lap-time penalty for ~25% lower mean error and a ~40% reduction in peak corner excursion, because slowing down on corners reduces per-tick lateral motion (the same stability mechanism described above for K_p tuning). On long straights `|e|` is small, so the scaling factor stays at 1 and the robot runs at full speed — the cost is paid only where it buys accuracy.

## Scholar Perspective

**System hygiene — distance choice.** I deliberately chose two specific beams (perpendicular + 60° diagonal) over a windowed mean of the right side. A windowed mean smooths sensor noise but discards the wall-angle signal needed to estimate `α`; the two-beam formula recovers `α` explicitly, which is what makes the predictive look-ahead possible. For noisier sensors I would average each of the two beams over a small angular window (±2 samples) — preserves geometry, costs little.

**Stability analysis — oscillation.** Snaking is a clean symptom that K_p is too large relative to forward velocity. The cause is loop delay: the controller's sense of where the wall is lags reality by ~1 tick + look-ahead. If K_p is large enough that one cycle of correction overshoots and reverses, the system enters sustained limit-cycle oscillation. Two ways out: raise K_d (adds phase lead, dampens the response), or lower `v` (reduces per-tick lateral motion). With `v = 0.5` fixed, K_d is the practical knob.

## Credits

Lec 02/06 ROS 2 node template · Lec 07/08 PID and FSM material · F1Tenth wall-following formulation for the predictive two-beam derivation · PA0 `RandomWalk` FSM and `_valid_range` filtering pattern.
