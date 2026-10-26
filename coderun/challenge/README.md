# CodeRun 2026 - Robotics.AI challenge

Reactive robot navigation in the Stage simulator (ROS 2 Humble), built on the Lab 2 "wander" demo
(`/base_scan` in, `/cmd_vel` out). Image: `laurentiupopa/ros2-efac:ros2`.

## Layout

```
challenge/
  tools/gen_maps.py                  generates bitmaps/easy.png and hard.png (deterministic)
  src/coderun_challenge/             organizers: worlds, launch file, automatic scorer
  src/coderun_student/               students: starter node (solution.py)
  reference/reference_solution.py    organizers only: reference controller for testing
```

## Setup (inside the container)

Start the container as in Lab 1, mounting this folder:

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

## Task statement (for students)

The robot (Pioneer 2DX, 270 deg laser, 5 m range) starts at a fixed pose. You get `/base_scan`,
`/odom` and publish `/cmd_vel`. Touching a wall or obstacle counts as a collision. Max speed
1 m/s, 0.78 rad/s. Time limit per task: 20 minutes. Edit `coderun_student/solution.py`.

```bash
# terminal 1 - arena + judge (window shown)
ros2 launch coderun_challenge challenge.launch.py task:=easy team:=MyTeam
# terminal 2 - your node
ros2 run coderun_student solution
```

| | Max | Free | Requirements | Time bonus |
|---|---|---|---|---|
| EASY | 110 | 10 | 90 | 0.5 / min saved, max 10 |
| HARD | 220 | 20 | 180 | 1 / min saved, max 20 |

The time bonus counts only if at least 50 % of the requirements are solved. "Minutes saved" is
the 20-minute limit minus the time you needed to finish.

- **EASY** (12x12 m, obstacles): pass within 0.5 m of waypoints W1 (4,-4), W2 (4,4), W3 (-4,4),
  in this order, in the world frame. 30 points per waypoint. -5 points per collision.
- **HARD** (16x16 m, 4x4 rooms, 1.2 m doors, some closed sides): the layout is not given.
  Explore the maze: the floor is split into 1x1 m cells, and you need to visit 80 % of the
  reachable ones. Points are proportional to coverage / 80 %. -10 points per collision.

The run ends when the task is complete or after 20 minutes; the result is written to
`/tmp/coderun_result.json` (team, completed, collisions, time, score breakdown).

## Organizer test procedure

Run each step in the container after the setup above. Use `time_limit:=...` to keep tests short.

1. **Maps** (host, optional): `python3 tools/gen_maps.py` regenerates the bitmaps; it asserts
   that the start and all EASY waypoints are connected, and that the HARD maze is connected.
   If you change a map, update the matching `worlds/*.world` and `TASKS` in
   `launch/challenge.launch.py`.
2. **Idle robot, scorer runs and ends**
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy gui:=false time_limit:=20
   cat /tmp/coderun_result.json     # expect score total 10 (baseline only), 0 collisions
   ```
3. **Reference solution, EASY** (second terminal in the same container: `docker exec -it <name> bash`,
   then source the same files)
   ```bash
   ros2 launch coderun_challenge challenge.launch.py task:=easy gui:=false time_limit:=300 team:=ref
   python3 /challenge/reference/reference_solution.py --ros-args -p mode:=easy
   ```
   Expect: three `waypoint n/3 reached` lines, `FINISHED completed=True`, total about 100-110.
4. **Reference solution, HARD**: same with `task:=hard` and `-p mode:=hard`. Check the logged
   coverage grows. If it stalls well below 80 %, lower `coverage_target` in `scorer.py`
   (parameter, default 0.8) or widen the doors in `tools/gen_maps.py`.
5. **Collision threshold**: drive into a wall with `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
   The scorer must log exactly one `collision #n` per contact and none when passing at about
   30 cm. Tune `COLLIDE_DIST` / `CLEAR_DIST` in `scorer.py` if not.
6. **Odometry drift**: with the reference solution, compare `/odom` and `/ground_truth` near the
   last EASY waypoint (`ros2 topic echo --once <topic> --field pose.pose.position`). Students
   steer by `/odom`, the judge uses `/ground_truth`; if the gap is above about 0.3 m, reduce
   `odom_error` in `worlds/easy.world`.
7. **Grading a team**: `gui:=false team:=<name> result_file:=/tmp/<name>.json`, then collect the
   JSON files and send the totals to the organizers.

## Known status (not yet verified)

My one automated test run built both packages and launched Stage plus the scorer successfully
(headless, scorer finished at the time limit and wrote the JSON). The reference solution did not
move the robot in that run (Stage logged `watchdog timeout on robot_0`, i.e. no `/cmd_vel`), and I
did not find out why before the run was stopped. So:

- steps 3-6 above are untested, and the reference solution may have a bug;
- run it on its own first and read the first lines of its output for a Python exception.
- the EASY/HARD difficulty and the 80 % coverage target are untuned guesses.
