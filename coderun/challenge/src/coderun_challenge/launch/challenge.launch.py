"""
Launch the CodeRun arena (Stage) together with the goal server and the automatic scorer.

    ros2 launch coderun_challenge challenge.launch.py task:=easy team:=Alpha
    ros2 launch coderun_challenge challenge.launch.py task:=hard seed:=42 gui:=false rviz:=false

Arguments:
    task        easy | hard
    team        team name written into the result file
    seed        seed of the random goals (default: a new random one at every launch; it is
                printed and written to the result, so a run can be repeated)
    gui         show the Stage window (false = headless, for grading)
    rviz        open RViz with the map and the goal rectangles (false for grading / headless)
    time_limit  seconds before the run is cut off (default 1200)
    result_file where the JSON result is written

The goals (static ones from config/<task>_goals.yaml + random ones) are sampled HERE, at
launch; the world file with the goal markers is generated into a temporary directory.
"""

import math
import os
import random
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from coderun_challenge import sampler, worldgen

TASKS = ('easy', 'hard')


def _setup(context):
    share = get_package_share_directory('coderun_challenge')
    task = LaunchConfiguration('task').perform(context)
    if task not in TASKS:
        raise RuntimeError(f"task must be one of {TASKS}, got '{task}'")
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    seed = LaunchConfiguration('seed').perform(context) or str(random.SystemRandom().randrange(10 ** 9))

    with open(os.path.join(share, 'config', f'{task}_goals.yaml')) as f:
        cfg = yaml.safe_load(f)
    goals = sampler.sample_goals(cfg, sampler.bitmap_path(share, task), int(seed))
    flat = [float(v) for g in goals for v in (g['x'], g['y'], math.radians(g['yaw_deg']))]
    print(f'[coderun] task={task} seed={seed} goals=' +
          ' '.join(f"({g['x']:.2f},{g['y']:.2f},{g['yaw_deg']:.0f})" for g in goals))

    world = worldgen.write_world(task, cfg, goals, share, tempfile.mkdtemp(prefix='coderun_world_'))

    stage = Node(
        package='stage_ros2', executable='stage_ros2', name='stage', output='screen',
        parameters=[{'world_file': world, 'enable_gui': gui, 'publish_ground_truth': True}])
    server = Node(
        package='coderun_challenge', executable='goal_server', name='goal_server', output='screen',
        parameters=[{'task': task, 'goals': flat}])
    scorer = Node(
        package='coderun_challenge', executable='scorer', name='coderun_scorer', output='screen',
        parameters=[{
            'mode': task,
            'team': LaunchConfiguration('team').perform(context),
            'time_limit': float(LaunchConfiguration('time_limit').perform(context)),
            'result_file': LaunchConfiguration('result_file').perform(context),
            'goals': flat,
            'seed': seed,
        }])
    actions = [stage, server, scorer]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'challenge.rviz')]))
    # when the scorer is done (task finished / time out) take the whole launch down
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=scorer, on_exit=[EmitEvent(event=Shutdown(reason='run finished'))])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='easy'),
        DeclareLaunchArgument('team', default_value='unnamed'),
        DeclareLaunchArgument('seed', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('time_limit', default_value='1200'),
        DeclareLaunchArgument('result_file', default_value='/tmp/coderun_result.json'),
        OpaqueFunction(function=_setup),
    ])
