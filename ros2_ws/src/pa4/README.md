# PA4 — Occupancy Grid Mapping

COSC 81/281 · Spring 2026 · Christian Nyamekye

ROS 2 node that builds an occupancy grid in the odom frame from `/base_scan` and `/odom`. Bresenham raycasting clears cells along each beam; the endpoint is marked occupied. Log-odds update by default; binary available via parameter. Grid grows automatically when beams land outside.

## Files

- `occupancy_mapper.py` — ROS 2 node
- `report.pdf` — write-up
- `demo.mp4` — recording of the map being built

## Run

In every container shell:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

**T1 — Stage:**
```bash
ros2 launch stage_ros2 stage.launch.py world:=/root/ros2_ws/src/pa2/2017-02-11-00-31-57 enforce_prefixes:=false one_tf_tree:=true
```
Or the PA3 maze: `world:=/root/ros2_ws/src/pa3/maze`.

**T2 — mapper:**
```bash
python3 /root/ros2_ws/src/pa4/occupancy_mapper.py
```

**T3 — drive.** Autonomous:
```bash
python3 /root/ros2_ws/src/pa2/wall_follower.py
```
Or manual (one-time install `apt install -y ros-humble-teleop-twist-keyboard`):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**T4 — RViz:**
```bash
rviz2
```
Fixed Frame: `rosbot1/odom`. Add `/map` (Map) and `/base_scan` (LaserScan). If the map doesn't render, use the frame_id printed by the mapper's `Adopting odom frame_id '...'` log line.

## Parameters

Override with `--ros-args -p name:=value`.

| Name | Default | Meaning |
|---|---|---|
| `scan_topic` | `base_scan` | Override to `scan` for Gazebo |
| `odom_topic` | `odom` | |
| `map_topic` | `map` | Output topic |
| `resolution` | 0.05 | m/cell |
| `width` / `height` | 400 / 400 | Initial grid size |
| `origin_x` / `origin_y` | −10 / −10 | Lower-left corner (m) |
| `publish_period` | 1.0 | Seconds between `/map` publishes |
| `scan_stride` | 1 | Process every Nth beam |
| `growth_padding` | 200 | Cells of headroom when expanding |
| `use_log_odds` | `True` | Probabilistic update (EC) |
| `l_occ` / `l_free` | +0.619 / −0.619 | Log-odds increments |
| `l_max` / `l_min` | +5 / −5 | Saturation clamps |
| `l_thresh_occ` / `l_thresh_free` | +0.85 / −0.85 | Binary output thresholds |
| `treat_max_range_as_free` | `True` | Clear along max-range beams; don't mark endpoint |

## Stop

`Ctrl+C` each terminal.
