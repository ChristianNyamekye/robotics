# PA0 — Random Walk

A ROS 2 node that drives a TurtleBot3 forward until it sees an obstacle within a forward field of view, then rotates a random angle and resumes.

## Files
- [pa0_random_walk.py](pa0_random_walk.py)

## Behavior
1. Drive forward at `LINEAR_VELOCITY`.
2. On every laser scan, check the minimum range within `[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD]`.
3. If the closest reading is below `MIN_THRESHOLD_DISTANCE`, set the obstacle flag.
4. Pick a random angle in `[RANDOM_TURN_MIN_DEG, RANDOM_TURN_MAX_DEG]` and rotate in place at `ANGULAR_VELOCITY` for the time required to sweep that angle.
5. Clear the flag and resume forward motion.

## Tunable Constants
| Constant | Default | Meaning |
|---|---|---|
| `FREQUENCY` | 10 Hz | Control loop rate |
| `LINEAR_VELOCITY` | 0.2 m/s | Forward speed |
| `ANGULAR_VELOCITY` | π/4 rad/s | Rotation speed |
| `MIN_THRESHOLD_DISTANCE` | 0.5 m | Obstacle distance trigger |
| `MIN/MAX_SCAN_ANGLE_RAD` | ±10° | Forward field of view |
| `RANDOM_TURN_MIN/MAX_DEG` | ±180° | Random rotation range |

## Run

**Terminal 1 — Gazebo:**
```bash
docker exec -it vnc-ros-ros-1 bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 — Random walk:**
```bash
docker exec -it vnc-ros-ros-1 bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/pa0_random_walk.py
```

View at [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html). Stop with `Ctrl+C`.
