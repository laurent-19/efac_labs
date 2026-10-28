"""
Stage world generation (pure python, no ROS). The launch file calls `write_world` at every
launch, because the random goals change from run to run and the goal markers are part of the
world.

Frames: the robot starts at (0, 0, 0); the floorplan is shifted by `map_origin` so that
/odom and /ground_truth start at the same pose.
"""

import shutil
from pathlib import Path

WORLD = '''# {task}.world - CodeRun {TASK} arena ({size:g} x {size:g} m), based on the Lab 2 "cave" world.
# GENERATED at launch by coderun_challenge/worldgen.py (goals are random) - do not edit.
# The robot starts at (0, 0, 0); the floorplan is shifted by map_origin so that /odom and
# /ground_truth start at the same pose.

include "include/robots.inc"

resolution 0.02
interval_sim 100
speedup 1.0

define floorplan model
(
  color "gray30"
  boundary 1
  gui_nose 0
  gui_grid 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  laser_return 1
)

window
(
  size [ 700.000 700.000 ]
  scale 55.0
  center [ {ox:.3f} {oy:.3f} ]
  rotate [ 0 0 ]
  show_data 1
)

floorplan
(
  name "{task}"
  size [{size:.3f} {size:.3f} 0.800]
  pose [{ox:.3f} {oy:.3f} 0 0]
  bitmap "../bitmaps/{task}.png"
  gui_move 0
)

# Goal markers: visual only (invisible to the laser and to collisions). A rectangle at each goal
# pose, its nose shows the yaw; goal n has n small dots next to it. The start is green.
define marker model
(
  size [0.5 0.36 0.02]
  gui_nose 1
  gui_outline 0
  gui_move 0
  obstacle_return 0
  laser_return 0
  ranger_return 0
  blob_return 0
  fiducial_return 0
  gripper_return 0
)

define tally model
(
  size [0.07 0.07 0.02]
  gui_nose 0
  gui_outline 0
  gui_move 0
  obstacle_return 0
  laser_return 0
  ranger_return 0
  blob_return 0
  fiducial_return 0
  gripper_return 0
)

{markers}
pioneer2dx_with_laser
(
  name "robot_0"
  color "red"
  pose [ 0.0 0.0 0 0 ]
  odom_error [ 0.01 0.01 0.0 0.02 ]
)
'''


def markers_block(start, goals):
    """Stage model lines for the start and the goals (start frame, yaw in degrees)."""
    lines = [f'marker( name "start" pose [ {start["x"]:.3f} {start["y"]:.3f} 0 {start["yaw_deg"]:.1f} ] color "green" )']
    for i, g in enumerate(goals, start=1):
        lines.append(f'marker( name "goal{i}" pose [ {g["x"]:.3f} {g["y"]:.3f} 0 {g["yaw_deg"]:.1f} ] color "cyan" )')
        for k in range(i):   # tally dots above the rectangle: i dots = goal i
            dx = (k - (i - 1) / 2) * 0.11
            lines.append(f'tally( name "goal{i}_dot{k + 1}" pose [ {g["x"] + dx:.3f} {g["y"] + 0.4:.3f} 0 0 ] color "white" )')
    return '\n'.join(lines) + '\n'


def write_world(task, cfg, goals, share, out_dir):
    """Write <out_dir>/worlds/<task>.world next to copies of the includes and the bitmap
    (same layout as the installed package, so relative paths resolve). Returns the path."""
    out = Path(out_dir)
    (out / 'worlds' / 'include').mkdir(parents=True, exist_ok=True)
    (out / 'bitmaps').mkdir(parents=True, exist_ok=True)
    for f in (Path(share) / 'worlds' / 'include').glob('*.inc'):
        shutil.copy(f, out / 'worlds' / 'include' / f.name)
    shutil.copy(Path(share) / 'bitmaps' / f'{task}.png', out / 'bitmaps' / f'{task}.png')
    path = out / 'worlds' / f'{task}.world'
    path.write_text(WORLD.format(
        task=task, TASK=task.upper(), size=cfg['world_size'],
        ox=cfg['map_origin']['x'], oy=cfg['map_origin']['y'],
        markers=markers_block(cfg['start'], goals)))
    return str(path)
