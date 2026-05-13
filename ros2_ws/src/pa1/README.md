# PA1 — Shape-Drawing Robot

COSC 81/281 · Spring 2026 · Christian Nyamekye

ROS 2 node that drives a TurtleBot3 along a trapezoid, D-shape, or arbitrary polygon using the differential-drive kinematic model. Supports open-loop (time-based) and closed-loop (odom feedback, extra credit).

## Files

- `shape_drawer.py` — ROS 2 node
- `report.pdf` — 1-page write-up
- `demo.mp4` — screen recording of all three shapes

## Run

**Terminal 1** — empty Gazebo world:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

**Terminal 2** — the shape drawer:
```bash
python3 shape_drawer.py
```

Follow the prompts:

| Shape | Input |
|---|---|
| 1. Trapezoid | radius `r`, e.g. `1.0` |
| 2. D-shape   | radius `r`, e.g. `1.0` |
| 3. Polygon   | space-separated `x,y` pairs, e.g. `0.0,0.0 1.0,0.0 1.0,1.0 0.0,1.0` |

Answer `y` at the closed-loop prompt to enable odom-feedback control (extra credit).

**Polygon note:** vertices are in the `odom` frame. Make the first vertex match the robot's current odom position (e.g. `(0,0)` after a fresh spawn) to avoid an extra approach leg.

## Reset between runs

```bash
ros2 service call /reset_world std_srvs/srv/Empty
```

Use `/reset_world` rather than `/reset_simulation` — it resets pose without resetting the ROS clock (which breaks RViz).

## Tuning

Constants at the top of `shape_drawer.py`: `LINEAR_VELOCITY`, `ANGULAR_VELOCITY`, `WHEEL_BASE`, `KP_LINEAR`, `KP_ANGULAR`, `POSITION_TOLERANCE`, `HEADING_TOLERANCE`.
