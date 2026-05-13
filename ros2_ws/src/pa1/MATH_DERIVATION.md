# Math Derivation Notes (working document)

Use this as your "show your work" reference — it maps each shape's motion
to the lecture's differential-drive / ICC equations.

## Differential-Drive Kinematics (Lec 04)

For a robot with wheel base `L`, left wheel speed `v_l`, right wheel speed
`v_r`:

```
v     = (v_r + v_l) / 2                  # linear speed along body +x
ω     = (v_r - v_l) / L                  # angular speed about body z
R     = (L/2) · (v_r + v_l) / (v_r - v_l)  # ICC radius (signed)
```

### Three motion regimes

| Regime | Condition | Trajectory |
|---|---|---|
| Straight    | `v_r = v_l` | line along +x |
| In-place    | `v_r = -v_l`| zero-radius rotation |
| Arc         | otherwise   | circle of radius `R` about the ICC |

## Trapezoid (r) — "simplified D"

**Geometry.** Robot at (0,0,0), facing +x. The trapezoid has 4 vertices:

```
B_1 = (0,  r)                              (top of longer base)
V_1 = (r cos  45°, r sin  45°) = ( r√2/2,  r√2/2)
V_2 = (r cos -45°, r sin -45°) = ( r√2/2, -r√2/2)
B_2 = (0, -r)                              (bottom of longer base)
```

The longer base runs from `B_2` to `B_1` along the y-axis (length `2r`);
the shorter base runs from `V_1` to `V_2` at `x = r√2/2` (length `r√2`);
the two legs `B_1 → V_1` and `B_2 → V_2` each have length
`|B_1 - V_1| = √((r√2/2)² + (r - r√2/2)²) = r√(2 - √2) ≈ 0.765 r`.

Since the robot starts at the *midpoint* of the longer base, the path
traces the base in two halves. Full closed path (CW):

```
start → B_1 → V_1 → V_2 → B_2 → start
```

**Leg 1 — start → B_1.**
- Direction: `atan2(r, 0) = +π/2`.
- Rotation: `Δθ_1 = +π/2` (heading 0 → +π/2).
- Distance: `r`.

**Leg 2 — B_1 → V_1.**
- Direction from `B_1` to `V_1`: `atan2(r√2/2 - r, r√2/2) = atan2(-(1-√2/2), √2/2) = -π/8`.
- Rotation from current heading (+π/2): `Δθ_2 = -π/8 - π/2 = -5π/8`.
- Distance: `r √(2 - √2)`.

**Leg 3 — V_1 → V_2 (shorter base).**
- Direction: `atan2(-r√2, 0) = -π/2`.
- Rotation from current heading (-π/8): `Δθ_3 = -π/2 - (-π/8) = -3π/8`.
- Distance: `r √2`.

**Leg 4 — V_2 → B_2.**
- Direction from `V_2` to `B_2`: `atan2(-r + r√2/2, -r√2/2) = atan2(-(1-√2/2), -√2/2) = -7π/8`.
- Rotation from current heading (-π/2): `Δθ_4 = -7π/8 - (-π/2) = -3π/8`.
- Distance: `r √(2 - √2)`.

**Leg 5 — B_2 → start.**
- Direction: `atan2(r, 0) = +π/2`.
- Rotation from current heading (-7π/8): `Δθ_5 = +π/2 - (-7π/8) = 11π/8` → normalize to `-5π/8`.
- Distance: `r`.

**Final rotation to restore heading.** After leg 5 heading = `+π/2`. Rotate `-π/2` to return to `0`.

**Sanity check.** Sum of rotations: `π/2 - 5π/8 - 3π/8 - 3π/8 - 5π/8 - π/2 = -2π`. Consistent with a CW loop (exterior-angle-sum identity).

**Expected end pose.** `(x, y, θ) = (0, 0, 0)` — closed path.

## D-shape (r)

**Geometry (in starting frame, start at origin facing +x).**
- Straight-stroke endpoints: top `(0, r)`, bottom `(0, -r)`.
- Semicircle: radius `r`, center `(0, 0)`, traversed from top to bottom along +x side.

**Step 1.** Rotate `+π/2` to face +y.
**Step 2.** Drive straight `r` to `(0, r)`.
**Step 3.** Rotate `-π/2` to face +x. (This is the tangent direction at `(0, r)` for CW motion around origin.)
**Step 4.** Arc: radius `r`, Δθ `= -π` (CW semicircle).
- `ω = v / r`, sign negative → `ω = -v/r`.
- Duration `T = π · r / v`.
- Wheel speeds: `v_r = v - v·L/(2r) = v(1 - L/(2r))`, `v_l = v(1 + L/(2r))` — left wheel faster (correct for right turn).
- End position `(0, -r)`, end heading `-π` = `+x` rotated by `-π` = facing `-x`.

**Step 5.** Rotate `+π/2` to face +y.
**Step 6.** Drive straight `r` to `(0, 0)` — closes the D.
**Step 7.** Rotate `-π/2` to restore heading `+x`.

**Why handle R → ∞ separately.** `R = (L/2)(v_r + v_l)/(v_r - v_l)` diverges
as `v_r → v_l`. `drive_straight` branches to command `(v, ω=0)` directly.

## Polygon

Given vertices `{p_1, p_2, ..., p_n}` in odom:

For each leg `p_i → p_{i+1}` (with `p_{n+1} = p_1`):
1. Read current pose `(r_x, r_y, r_θ)` from `tf2.lookup_transform(odom, base_footprint)`.
2. `Δx = p_{i+1,x} - r_x`, `Δy = p_{i+1,y} - r_y`.
3. Desired heading `θ* = atan2(Δy, Δx)`.
4. Rotation `Δθ = normalize(θ* - r_θ)`.
5. Distance `d = √(Δx² + Δy²)`.
6. Execute `rotate_in_place(Δθ)` then `drive_straight(d)`.

Re-reading the transform on every iteration means open-loop drift within a
leg is naturally re-observed on the *next* leg, so errors don't compound.

## Closed-loop variants (extra credit)

### `rotate_in_place_closed(Δθ)`
Target yaw `θ* = normalize(current_yaw + Δθ)`.
Loop: `ω = K_θ · normalize(θ* - current_yaw)`, saturate, publish, sleep.
Stop when `|e_θ| < HEADING_TOLERANCE`.

### `drive_to_point_closed(gx, gy)`
1. Call `rotate_in_place_closed` to point at `(gx, gy)`.
2. Loop:
   - `e_d = ‖(gx, gy) - (x, y)‖`
   - `e_θ = normalize(atan2(gy - y, gx - x) - θ)`
   - `v = min(K_d · e_d, v_max)`, but set `v = 0` if `|e_θ| > π/6`
   - `ω = sat(K_θ · e_θ, ω_max)`
   - Publish, sleep.
3. Stop when `e_d < POSITION_TOLERANCE`.

Gains & tolerances: see constants at the top of `shape_drawer.py`.
