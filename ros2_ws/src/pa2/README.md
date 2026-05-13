# PA2 — Wall-Following Robot

COSC 81/281 · Spring 2026 · Christian Nyamekye

ROS 2 node that drives the Stage `rosbot` along the right-hand wall of a corridor. Discrete PID on the predicted lateral distance produces angular velocity; an FSM handles internal corners and lost-wall recovery.

## Files

- `wall_follower.py` — ROS 2 node
- `report.pdf` — write-up
- `demo.mp4` — corridor traversal recording
- `2017-02-11-00-31-57.world`, `empty.world`, `*.png`, `*.pgm`, `rosbot.inc` — Stage assets
- `install_stage.sh` — Stage installer

## Build (one-time, inside the container)

```bash
cd /root/ros2_ws/src/pa2
bash install_stage.sh
source /root/ros2_ws/install/setup.bash
```

## Run

In every container shell, first:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

**Terminal 1 — Stage corridor world:**
```bash
ros2 launch stage_ros2 stage.launch.py \
  world:=/root/ros2_ws/src/pa2/2017-02-11-00-31-57 \
  enforce_prefixes:=false one_tf_tree:=true
```
Use `world:=/root/ros2_ws/src/pa2/empty` for the empty world.

**Terminal 2 — wall follower:**
```bash
python3 /root/ros2_ws/src/pa2/wall_follower.py
```

`Ctrl+C` to stop; the node publishes a zero `Twist` on shutdown.

## Parameters

Override any of these with `--ros-args -p name:=value`, e.g.
`python3 wall_follower.py --ros-args -p kp:=2.5 -p kd:=0.4`.

| Name | Default | Meaning |
|---|---|---|
| `kp` / `ki` / `kd` | 2.0 / 0.0 / 0.3 | PID gains |
| `linear_velocity` | 0.5 | Forward speed (m/s) |
| `target_distance` | 0.5 | Wall stand-off (m) |
| `look_ahead` | 0.6 | Predictive look-ahead distance (m) |
| `beam_theta_deg` | 60 | Angle between perp and forward right beams |
| `max_angular_velocity` | 1.5 | rad/s saturation |
| `front_threshold` | 0.7 | CORNER trigger (m) |
| `lost_wall_threshold` | 2.0 | SEARCH trigger (m) |
| `front_fov_deg` | 20 | Forward FOV half-width for corner check |
| `right_beam_deg` | -90 | Perpendicular right beam angle |
| `scan_topic` | `base_scan` | Override to `scan` for Gazebo |
| `adaptive_velocity` | `True` | EC: scale `v` down as \|error\| grows |
| `adaptive_min_fraction` | 0.4 | EC: floor for adaptive `v` (× `linear_velocity`) |
| `adaptive_error_scale` | 2.0 | EC: how aggressively \|error\| pulls `v` down |

## FSM

- **FOLLOW** — PID active.
- **CORNER** — front beam < `front_threshold`; reduce `v`, command max-left ω.
- **SEARCH** — perpendicular right beam > `lost_wall_threshold`; reduce `v`, command moderate-right ω until wall is re-acquired.

State transitions reset PID memory.
