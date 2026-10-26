# CodeRun 2026 - Robotics.AI challenge

Path planning for a differential-drive robot in the Stage simulator (ROS 2 Humble), built on the
Lab 2 "wander" demo. Image: `laurentiupopa/ros2-efac:ros2`.

## Layout

```
challenge/
  tools/gen_maps.py                  generates bitmaps/*.png from the code and checks them against the goal files
  src/coderun_challenge/             organizers: worlds, bitmaps, goal files (config/), launch, scorer
  src/coderun_student/               students: starter node (solution.py)
  reference/                         organizers only: reference_solution.py + routes.py
```

`src/coderun_challenge/config/{easy,hard}_goals.yaml` is the single source of truth for the start
pose and the goals. The scorer, the student template, the reference and `gen_maps.py` all read it.
The start pose in `worlds/*.world` must match it.

## Task statement (for students)

The judge gives you a list of **goal poses** (x, y, yaw) in `config/<task>_goals.yaml`. Reach them
**in order**, **as fast as you can**, without touching anything. Look at the goal poses and the map,
decide on waypoints / a path, and drive it with `/cmd_vel`.

- Inputs: `/odom` (pose, starts at the start pose, world frame) and `/base_scan` (270 deg, 5 m).
- Output: `/cmd_vel`, max 1 m/s and 0.78 rad/s.
- A goal counts when the robot is within 0.3 m of it **and** its heading is within 20 deg of the goal yaw.
- Touching a wall or obstacle is a collision.
- Time limit: 20 minutes per task. The run ends when all goals are reached or time runs out.

```bash
# terminal 1 - arena + judge (window shown)
ros2 launch coderun_challenge challenge.launch.py task:=easy team:=MyTeam
# terminal 2 - your node (edit coderun_student/solution.py)
ros2 run coderun_student solution --ros-args -p task:=easy
```

|  | Goals | Map |
|---|---|---|
| EASY | 5 | 12x12 m, a few blocks. Nothing blocks the way if you go x-then-y from goal to goal. |
| HARD | 4 | 12x12 m, three staggered walls with gaps on alternating sides + pillars. A straight line to the next goal is blocked, you must plan a path through the gaps. |

### Scoring (CodeRun guidelines)

| | Max | Free | Requirements | Time bonus |
|---|---|---|---|---|
| EASY | 110 | 10 | 90 | 0.5 / min saved, max 10 |
| HARD | 220 | 20 | 180 | 1 / min saved, max 20 |

- Requirements = max * goals_reached / goals_total, minus 5 (EASY) / 10 (HARD) per collision, min 0.
- Time bonus = rate * (20 min - time needed), only if all goals were reached and at least 50 % of the
  requirements are solved. The faster the better.
- The result is written to `/tmp/coderun_result.json`: team, completed, collisions, time, the time
  each goal was reached, score breakdown.

## Setup (inside the container)

```bash
xhost local:root
docker run -it --rm --privileged --network=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v ~/efac_labs/coderun/challenge:/challenge \
  laurentiupopa/ros2-efac:ros2 bash
```

```bash
source /opt/ros/humble/setup.bash
cp -r /challenge/src/coderun_challenge /challenge/src/coderun_student ~/ros_ws/src/
cd ~/ros_ws && colcon build --packages-select coderun_challenge coderun_student
source install/setup.bash
```

A second terminal in the same container: `docker exec -it <container> bash`, then source the same two files.

## Organizer test procedure

**Status:** nothing in this version has been run in Docker yet. The scorer pipeline (build, launch,
headless Stage, JSON at the time limit) worked in an earlier version. `gen_maps.py` was run and passes
on the host. The files compile.

1. **Maps (host, optional)**: `python3 tools/gen_maps.py` regenerates the bitmaps and fails if a
   goal is unreachable, closer than 0.5 m to a wall, or the reference route is closer than 0.45 m
   to a wall. After changing a map or a goal file, rebuild and keep `worlds/*.world` start poses in sync.
2. **Idle robot**
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy gui:=false time_limit:=20
   cat /tmp/coderun_result.json     # expect total 10 (baseline), 0 goals, 0 collisions
   ```
3. **Odometry frame check (important)**: the reference and the students assume `/odom` starts at the
   start pose (world frame). With Stage running:
   ```bash
   ros2 topic echo --once /odom --field pose.pose.position
   ros2 topic echo --once /ground_truth --field pose.pose.position
   ```
   Both should be about (-5, -5) for EASY. If `/odom` starts at (0, 0), the goal poses are not usable as
   given: either add `localization_origin [-5 -5 0 0]` to the robot in the world file or tell students
   goals are relative to the start.
4. **Reference solution, EASY** (same time limit as the real run, with the window)
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy team:=ref
   python3 /challenge/reference/reference_solution.py --ros-args -p task:=easy
   ```
   Expect `goal 1/5` ... `goal 5/5 reached`, `FINISHED completed=True`, 0 collisions, total about 100-110.
   If the robot does not move, read the first lines of the reference's output for a Python error
   (it needs to be run from a shell where both setup files are sourced).
5. **Reference solution, HARD**: same with `task:=hard` and `-p task:=hard`. Expect 4/4 goals. Also try
   driving straight from goal to goal by hand (teleop) to confirm the walls block it.
6. **Collision threshold**: drive into a wall with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
   The scorer must log exactly one `collision #n` per contact and none when passing about 30 cm away.
   Tune `COLLIDE_DIST` / `CLEAR_DIST` in `scorer.py` if not.
7. **Goal tolerance**: check that the reference's final turn registers every goal (radius 0.3 m, yaw
   20 deg). If a goal is missed by the reference, the tolerance is too tight for odometry drift: relax
   `goal_radius` / `yaw_tolerance_deg` in the YAML or lower `odom_error` in the world file.
8. **Grading a team**: `gui:=false team:=<name> result_file:=/tmp/<name>.json`, then collect the JSON
   files and send the totals to the organizers.

## Open points

- Students can read `/ground_truth` (a free perfect pose). For grading, run the scorer on a remapped topic.
- Stage's odometry noise is random, so grade each team on the same machine, and re-run if in doubt.
- Do not publish the `challenge/` folder before the event: the guidelines require that the training
  does not reveal the actual statement.
- The AI policy (allowed / forbidden) is still to be stated in the final statement.
