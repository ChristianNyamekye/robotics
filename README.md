# ROS on Mac/Windows

Use this guide to run a TurtleBot3 Gazebo simulation in Docker on macOS or Windows.

## Prerequisites
Install Docker Desktop first:
- macOS: [Install Docker Desktop for Mac](https://docs.docker.com/docker-for-mac/install/)
- Windows: [Install Docker Desktop for Windows (WSL2 backend)](https://docs.docker.com/docker-for-windows/install/#system-requirements-for-wsl-2-backend)

## 1. Start the containers
Open a terminal (macOS Terminal or Windows PowerShell) and run:

```bash
git clone https://github.com/quattrinili/vnc-ros
cd vnc-ros
mkdir -p workspace
docker compose up
```

Keep this terminal open. It should stay busy and show container logs.

Notes:
- `ros.env` contains ROS-related environment variables you can adjust before running `docker compose up`.
- Log messages may vary. If there are no errors and the command keeps running, continue.

## 2. Open the desktop in your browser
1. Open your browser and go to `http://localhost:8080/vnc.html`.
2. Click `Connect`.

You should see the container desktop.

## 3. Start the Gazebo simulation
Open a second terminal and run:

```bash
cd vnc-ros
docker compose exec ros bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Keep this second terminal open while Gazebo is running.

## 4. Control the robot with your keyboard
Open a third terminal and run:

```bash
cd vnc-ros
docker compose exec ros bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keys shown in the terminal to move the robot. You should see movement in the browser window.

## 5. Stop everything cleanly
1. In terminal 3 (keyboard control), press `Ctrl+C`, then type `exit`.
2. In terminal 2 (Gazebo), press `Ctrl+C`, then type `exit`.
3. In terminal 1 (`docker compose up`), press `Ctrl+C` to stop the containers.

After that, you can close all terminals.

## Editing your workspace
The local `workspace` folder is where you can create and edit your ROS packages.
It is mounted inside the container at `~/catkin_ws`, so changes are shared between your host machine and the container.

## Installing additional packages
1. Edit the package install line in `Dockerfile`.
2. Rebuild the image:

```bash
docker compose build
```
