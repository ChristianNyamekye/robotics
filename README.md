# Robotics — COSC 81/281 Programming Assignments

Christian Nyamekye · Spring 2026 · Dartmouth

ROS 2 Humble assignments for *Principles of Robot Design and Programming*. Each `paN/` folder under `ros2_ws/src/` is self-contained: code, README, report, and demo recording. The repo also ships the Docker-based dev environment forked from the course template.

## Assignments

| PA | Topic | Folder |
|---|---|---|
| 0 | Random walk on TurtleBot3 | [`ros2_ws/src/pa0`](ros2_ws/src/pa0) |
| 1 | Shape-drawing kinematic controller (trapezoid / D / polygon) | [`ros2_ws/src/pa1`](ros2_ws/src/pa1) |
| 2 | Reactive wall-follower on Stage rosbot | [`ros2_ws/src/pa2`](ros2_ws/src/pa2) |
| 3 | A\* / wavefront path planner on a maze | [`ros2_ws/src/pa3`](ros2_ws/src/pa3) |
| 4 | Occupancy grid mapping with log-odds | [`ros2_ws/src/pa4`](ros2_ws/src/pa4) |

Each folder has its own README with run instructions and a `report.pdf`. Demos are `demo.mp4` in the same folder.

## Environment

Docker + noVNC, so the ROS 2 Humble stack and GUI tools (Stage, RViz, Gazebo) run in a Linux container viewable through a browser tab. Works the same on macOS and Windows.

### Prerequisites

[Docker Desktop](https://docs.docker.com/desktop/) installed and running. On Windows enable the WSL 2 backend.

### Bring up

```bash
git clone https://github.com/ChristianNyamekye/robotics
cd robotics
docker compose up -d
```

Open <http://localhost:8080/vnc.html?autoconnect=true&resize=remote> in a browser for the GUI desktop.

### Enter the container

```bash
docker exec -it vnc-ros-ros-1 bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

Then `cd /root/ros2_ws/src/paN` and follow that PA's README.

### Tear down

```bash
docker compose down
```

## Repo layout

```
robotics/
├── Dockerfile              # ROS 2 Humble + Stage + tools
├── docker-compose.yml      # ros + novnc services, mounts ros2_ws/src
├── ros.env / novnc.env     # container env vars
├── ros2_ws/src/
│   ├── pa0/                # random walk
│   ├── pa1/                # shape drawer
│   ├── pa2/                # wall follower
│   ├── pa3/                # path planner
│   ├── pa4/                # occupancy mapper
│   └── lec02_example_go_forward.py
└── README.md               # this file
```

`ros2_ws/src/Stage/` and `stage_ros2/` are upstream vendored packages and are gitignored — they're built fresh from the Dockerfile.

## Credits

Course infrastructure forked from [quattrinili/vnc-ros](https://github.com/quattrinili/vnc-ros) (Prof. Alberto Quattrini Li, Dartmouth Computer Science). All assignment code and write-ups are my own.
