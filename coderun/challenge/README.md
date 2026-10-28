# CodeRun 2026 - Robotics.AI challenge

Path planning for a differential-drive robot in the Stage simulator (ROS 2 Humble), built on the
Lab 2 "wander" demo. Image: `laurentiupopa/ros2-efac:ros2`.

## Layout

```
challenge/
  tools/gen_maps.py                  generates bitmaps/*.png, then stress-tests the random goals
  src/coderun_challenge/             organizers: bitmaps, goal config (config/), launch, goal server, scorer
  src/coderun_student/               students: starter node (solution.py)
  reference/                         organizers only: reference_solution.py + routes.py (planner)
```

**Random goals.** `config/{easy,hard}_goals.yaml` holds the *static* goals and the sampling rules.
At every launch the launch file appends `random_goals` sampled ones (`coderun_challenge/sampler.py`):

| | Static | Random | Rules for a random goal |
|---|---|---|---|
| EASY | goals 1-4 | goal 5 | reachable, >= 0.6 m from walls, >= 3 m from the previous goal, and a clear x-then-y route from the previous goal (so the simple reference works) |
| HARD | none | all 4 | same, but the straight line from the previous goal must be **blocked** by a wall (a path must be planned) |

The seed is random by default and is printed at launch and saved in the result JSON (`seed:=42`
repeats a run). Because of that, goals can't be read from a file: they are published on `/goals`.
The Stage world (with the goal markers) is generated at launch into a temporary directory.

**Frames.** The robot starts at (0, 0, 0) and the map is shifted instead: `map_origin` in the YAML
is where the map centre is in that frame. So `/odom`, `/ground_truth`, `/goals` and `/map` all
share one frame and start at the same pose. (Obstacles in `gen_maps.py` are written in the map
frame, centred on the bitmap.)

## Task statement (for students)

Reach a list of **goal poses** (x, y, yaw) **in order**, **as fast as you can**, without touching
anything. The goals are random in every run, so read them at runtime; don't hardcode them.

- `/goals` (`geometry_msgs/PoseArray`): the goal poses, in order. Published once, latched: subscribe
  with a transient-local QoS (see the template).
- `/map` (`nav_msgs/OccupancyGrid`): the arena, 0.04 m cells, 100 = wall. Same latched QoS.
- `/odom`: the robot pose, starts at (0, 0, 0), same frame as the goals and the map.
- `/base_scan` (270 deg, 5 m) and `/cmd_vel` (max 1 m/s and 0.78 rad/s).
- A goal counts when the robot is within 0.3 m of it **and** its heading is within 20 deg of the goal yaw.
- Touching a wall or obstacle is a collision. Time limit: 20 minutes per task.

The goals are drawn in the Stage window (a cyan rectangle at each goal pose, its nose along the goal
yaw, n small white dots above goal n; the start is green) and in RViz (rectangle, arrow, tolerance
ring and number, plus the map). They are visual only: invisible to the laser and to collisions.

```bash
# terminal 1 - arena + judge (Stage window + RViz)
ros2 launch coderun_challenge challenge.launch.py task:=easy team:=MyTeam
# terminal 2 - your node (edit coderun_student/solution.py)
ros2 run coderun_student solution
```

| | Goals | Map |
|---|---|---|
| EASY | 5 (the last one random) | 12x12 m, a few blocks. Going x-then-y from goal to goal works. |
| HARD | 4, all random | 12x12 m, three staggered walls with gaps on alternating sides + pillars. A straight line to the next goal is blocked, so plan a path (e.g. A* on `/map`). |

### Scoring (CodeRun guidelines)

| | Max | Free | Requirements | Time bonus |
|---|---|---|---|---|
| EASY | 110 | 10 | 90 | 0.5 / min saved, max 10 |
| HARD | 220 | 20 | 180 | 1 / min saved, max 20 |

- Requirements = max * goals_reached / goals_total, minus 5 (EASY) / 10 (HARD) per collision, min 0.
- Time bonus = rate * (20 min - time needed), only if all goals were reached and at least 50 % of the
  requirements are solved. The faster the better.
- The result is written to `/tmp/coderun_result.json`: team, mode, seed, the goals of the run,
  completed, collisions, time, the time each goal was reached, score breakdown.

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

**Status:** not yet run in Docker in this version. On the host (no ROS) `tools/gen_maps.py` passes:
random goals are sampled for many seeds and the reference planner finds a route to every goal
(EASY routes about 22-27 m, HARD 33-91 m). The files parse. The earlier scorer pipeline (build, launch,
headless Stage, JSON at the time limit) worked in the first version.

1. **Maps and goals (host, optional)**: `python3 tools/gen_maps.py [--seeds 100]` regenerates the bitmaps
   and fails if a static goal is unreachable or too close to a wall, or if some seed gives goals that
   break the rules or that the reference planner can't route to (needs numpy, scipy, PIL, PyYAML).
2. **Idle robot**
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy gui:=false rviz:=false time_limit:=20 seed:=1
   cat /tmp/coderun_result.json     # expect total 10 (baseline), 0 goals, 0 collisions, seed 1, 5 goals
   ```
3. **Goals and map topics**
   ```bash
   ros2 topic echo --once /goals --qos-durability transient_local --qos-reliability reliable
   ros2 topic echo --once /odom --field pose.pose.position
   ros2 topic echo --once /ground_truth --field pose.pose.position
   ```
   `/goals` must list 5 (EASY) or 4 (HARD) poses; odom and ground truth must both be about (0, 0).
   In RViz and in Stage the markers must be at the same places, on free floor, and the map must
   cover the arena. Launching twice without `seed` must give different last (EASY) / all (HARD) goals.
4. **Reference solution, EASY** (with the windows)
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy team:=ref
   python3 /challenge/reference/reference_solution.py --ros-args -p task:=easy
   ```
   Expect `goal 1` ... `goal 5 done`, `FINISHED completed=True`, 0 collisions, total about 100-110.
   If the robot does not move, read the first lines of the reference's output (it must be run from a
   shell where both setup files are sourced).
5. **Reference solution, HARD**: same with `task:=hard` and `-p task:=hard`. Expect 4/4 goals. Repeat with
   a few seeds (`seed:=1`, `2`, ...). Also drive straight to the first goal by hand (teleop) to confirm
   the walls block it.
6. **Collision threshold**: drive into a wall with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
   The scorer must log exactly one `collision #n` per contact and none when passing about 30 cm away.
   Tune `COLLIDE_DIST` / `CLEAR_DIST` in `scorer.py` if not.
7. **Goal tolerance**: check that the reference's final turn registers every goal (radius 0.3 m, yaw
   20 deg). If a goal is missed by the reference, the tolerance is too tight for odometry drift: relax
   `goal_radius` / `yaw_tolerance_deg` in the YAML or lower `odom_error` in `worldgen.py`.
8. **Grading a team**: `gui:=false rviz:=false team:=<name> result_file:=/tmp/<name>.json seed:=<S>`, then
   collect the JSON files and send the totals to the organizers.

## Open points

- Fairness: random goals differ between teams. Use the same `seed` for every team of a division (pick it
  at the start of the slot, so it isn't known beforehand), or different ones if teams don't share a room.
- Students can read `/ground_truth` (a free perfect pose). For grading, run the scorer on a remapped topic.
- Stage's odometry noise is random, so grade each team on the same machine, and re-run if in doubt.
- Do not publish the `challenge/` folder before the event: the guidelines require that the training
  does not reveal the actual statement.
- The AI policy (allowed / forbidden) is still to be stated in the final statement.
