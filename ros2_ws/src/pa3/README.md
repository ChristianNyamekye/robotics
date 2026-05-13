# PA3 — Path Planning and Global Navigation

COSC 81/281 · Spring 2026 · Christian Nyamekye

ROS 2 node that smooths the `OccupancyGrid` with a Gaussian kernel, builds an inflated C-space grid, plans with BFS / DFS / A* (Manhattan heuristic), and drives the rosbot through the resulting waypoints.

## Files

- `path_planner.py` — ROS 2 node
- `report.pdf` — write-up
- `demo.mp4` — recordings for BFS, DFS, A*
- `maze.world`, `maze.png`, `maze.yml`, `rosbot.inc` — Stage + map assets

## Build (one-time, inside the container)

Stage was built in PA2. Install the map server:
```bash
apt update && apt install -y ros-humble-nav2-map-server
```

## Run

In every container shell, first:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

**T1 — Stage:**
```bash
ros2 launch stage_ros2 stage.launch.py world:=/root/ros2_ws/src/pa3/maze enforce_prefixes:=false one_tf_tree:=true
```

**T2 — map server (run once, then leave it):**
```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/root/ros2_ws/src/pa3/maze.yml -p use_sim_time:=true
```
In a temp shell, activate it:
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

**T3 — static `map → odom` (robot starts at (2, 2)):**
```bash
ros2 run tf2_ros static_transform_publisher --x 2 --y 2 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id odom
```

**T4 — planner:**
```bash
python3 /root/ros2_ws/src/pa3/path_planner.py
```
Choose 1/2/3 (BFS/DFS/A*), then enter goal `(x, y)` in the map frame.

`Ctrl+C` to stop. Reset Stage between algorithm runs (`Ctrl+C` T1, relaunch) so the robot teleports back to (2, 2).

## Parameters

Override with `--ros-args -p name:=value`.

| Name | Default | Meaning |
|---|---|---|
| `blur_kernel_size` | 5 | Gaussian kernel (3 or 5) for `/map_smoothed` |
| `inflation_radius_cells` | 7 | Dilation radius for the planning grid (35 cm) |
| `occupancy_threshold` | 50 | Cells > threshold treated as blocked |
| `use_smoothed_for_planning` | `False` | Plan on Gaussian-smoothed grid before dilation |
| `linear_velocity` / `angular_velocity` | 0.20 / π/4 | Cruise speeds |
| `max_linear` / `max_angular` | 0.30 / 1.5 | Saturation |
| `position_tolerance` / `heading_tolerance` | 0.10 / 0.08 | Waypoint accept thresholds |
| `waypoint_stride` | 6 | Path subsample (cells between waypoints) |
| `eight_connected` | `False` | EC: 8-neighbor expansion (`√2` diagonals) |
| `weighted_astar` | `False` | EC: A* `g(n)` weighted by smoothed occupancy |
| `weight_gain` | 5.0 | EC: corridor-centering aggressiveness |

## Topics

| Topic | Direction | Type |
|---|---|---|
| `/map` | sub | `nav_msgs/OccupancyGrid` (latched) |
| `/map_smoothed` | pub | `nav_msgs/OccupancyGrid` (latched) |
| `/pose_sequence` | pub | `geometry_msgs/PoseArray` |
| `/cmd_vel` | pub | `geometry_msgs/Twist` |
| `/odom` | sub | `nav_msgs/Odometry` (live pose) |
