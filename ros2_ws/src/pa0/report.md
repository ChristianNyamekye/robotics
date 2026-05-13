# PA-0: Reactive Random Walk

**COSC 81/281** · Christian Nyamekye · 2026-04-07

## 1. Overview

A ROS 2 node (`rclpy`) that drives a TurtleBot3 in Gazebo using a two-state reactive controller: drive forward until an obstacle enters the front FOV, rotate a random angle, resume. Subscribes to `/scan`, publishes to `/cmd_vel`.

## 2. Architecture

Two callbacks decouple sensing from control:

- **`_laser_callback`** — extracts the closest obstacle within the front FOV (`±10°`) via `_closest_obstacle_in_front()`. Sets `_close_obstacle = True` when below `MIN_THRESHOLD_DISTANCE` (0.5 m).
- **`_control_loop_callback`** — runs at 10 Hz. Implements the state machine.

Sim time (`use_sim_time=True`) is used so timing tracks Gazebo's clock.

## 3. State Machine

| State | Behavior | Transition |
|---|---|---|
| **Cruise** | Publish `linear.x = 0.2`, `angular.z = 0` | → Recovery when `_close_obstacle` set |
| **Recovery** | Pick random angle in `[-180°, 180°]`, rotate at `±π/4 rad/s` for `|angle|/ω` seconds | → Verify on rotation completion |
| **Verify** *(extra credit)* | Wait for fresh scan, re-check FOV | → Cruise if clear, → Recovery if still blocked |

## 4. FOV Processing

`_closest_obstacle_in_front(msg)` converts the configured angle range to indices via `(angle - msg.angle_min) / msg.angle_increment`, clamps to valid bounds, filters NaN/inf and out-of-`range_min/max` readings, returns the minimum.

## 5. Extra Credit — Dynamic Recovery

**Problem.** The baseline rotation is open-loop: a random angle could leave the robot still facing a wall (e.g. in corners), causing immediate re-trigger and oscillation.

**Solution.** A third state (`_verifying_clearance`) is entered after each rotation. The laser callback continues caching the latest forward distance (`_latest_front_distance`) during verification. The control loop only clears `_close_obstacle` when the cached distance exceeds `MIN_THRESHOLD_DISTANCE`; otherwise it triggers another random rotation. This guarantees the robot only resumes forward motion when the path is actually clear.

**Key change to control loop:**

```python
if self._verifying_clearance:
    if self._latest_front_distance is None:
        return  # wait for fresh scan
    if self._latest_front_distance >= self.min_threshold_distance:
        self._verifying_clearance = False
        self._close_obstacle = False  # resume cruise
    else:
        self._verifying_clearance = False
        # _close_obstacle stays True; next tick re-rotates
    return
```

**Observed effect.** The robot escapes corners reliably. In runs without dynamic recovery, the robot occasionally oscillated against walls when a small random angle was drawn. With it enabled, log lines like `"Still blocked at 0.31 m. Re-rotating."` confirm the robot retries until truly clear, then logs `"Path clear at 2.6 m. Resuming."`.

## 6. How to Run

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

View at <http://localhost:8080/vnc.html>.

## 7. Files

- [pa0_random_walk.py](pa0_random_walk.py) — node implementation
- [pa0_README.md](pa0_README.md) — build/run instructions
