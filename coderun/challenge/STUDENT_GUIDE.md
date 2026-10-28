# CodeRun 2026 - Student Setup Guide

Path planning challenge for a differential-drive robot in the Stage simulator (ROS 2 Humble).

## 1. Install WSL2 (Windows)

Open **PowerShell as Administrator**:
```powershell
wsl --install
```
Restart when prompted. Ubuntu will ask for a username/password.

## 2. Install Docker Desktop

1. Download from https://www.docker.com/products/docker-desktop/
2. Enable **WSL 2 backend** during install
3. Restart and wait for Docker Desktop to start

## 3. Start the Container

Open a WSL terminal. Each team uses a unique **TEAM_ID** (1–99) to isolate ROS 2 topics.

```bash
xhost +local:root

export ROS_DOMAIN_ID=TEAM_ID

docker run -it --name coderun \
  --privileged --network=host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -w /root/ros_ws \
  laurentiupopa/ros2-efac:ros2 bash
```

## 4. Additional Terminals

```bash
export ROS_DOMAIN_ID=TEAM_ID
docker exec -it coderun bash
```

Inside the container:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
```

## 5. Build

```bash
source /opt/ros/humble/setup.bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

## 6. Run

**Terminal 1** — arena + judge:
```bash
ros2 launch coderun_challenge challenge.launch.py task:=easy team:=MyTeam
```

**Terminal 2** — your solution:
```bash
ros2 run coderun_student solution
```

## 7. Your Task

Edit `~/ros_ws/src/coderun_student/coderun_student/solution.py`:
- Read `/goals` (PoseArray, latched) and `/map` (OccupancyGrid, latched)
- Use `/odom` for position, `/base_scan` for obstacles
- Publish to `/cmd_vel` (max 1 m/s, 0.78 rad/s)
- Reach each goal **in order, as fast as you can**, without collisions

Goal = within 0.3 m AND heading within 20° of goal yaw.

## 8. Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goals` | PoseArray | Goal poses in order (latched) |
| `/map` | OccupancyGrid | 0.04 m cells (latched) |
| `/odom` | Odometry | Robot pose |
| `/base_scan` | LaserScan | 270°, 5 m, 1° steps |
| `/cmd_vel` | Twist | Velocity command |

## 9. Scoring

| Mode | Max | Baseline | Requirements | Time Bonus |
|------|-----|----------|--------------|------------|
| EASY | 110 | 10 | 90 | 0.5/min, max 10 |
| HARD | 220 | 20 | 180 | 1.0/min, max 20 |

Result: `/tmp/coderun_result.json`

## 10. Container Management

```bash
docker stop coderun      # pause
docker start -i coderun  # resume
docker rm coderun        # delete
```

## Troubleshooting

- **X server error:** `xhost +local:root` and `export DISPLAY=:0`
- **Container exists:** `docker rm coderun`
- **Topics not visible:** same `ROS_DOMAIN_ID` in all terminals
