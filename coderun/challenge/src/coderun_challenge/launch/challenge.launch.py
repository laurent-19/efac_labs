"""
Launch the CodeRun arena (Stage) together with the automatic scorer.

    ros2 launch coderun_challenge challenge.launch.py task:=easy team:=Alpha
    ros2 launch coderun_challenge challenge.launch.py task:=hard gui:=false

Arguments:
    task        easy | hard
    team        team name written into the result file
    gui         show the Stage window (false = headless, for grading)
    time_limit  seconds before the run is cut off (default 1200)
    result_file where the JSON result is written

Start pose and goals come from config/<task>_goals.yaml (read by the scorer).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TASKS = ('easy', 'hard')


def _setup(context):
    share = get_package_share_directory('coderun_challenge')
    task = LaunchConfiguration('task').perform(context)
    if task not in TASKS:
        raise RuntimeError(f"task must be one of {TASKS}, got '{task}'")
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'

    stage = Node(
        package='stage_ros2', executable='stage_ros2', name='stage', output='screen',
        parameters=[{
            'world_file': os.path.join(share, 'worlds', f'{task}.world'),
            'enable_gui': gui,
            'publish_ground_truth': True,
        }])
    scorer = Node(
        package='coderun_challenge', executable='scorer', name='coderun_scorer', output='screen',
        parameters=[{
            'mode': task,
            'team': LaunchConfiguration('team').perform(context),
            'time_limit': float(LaunchConfiguration('time_limit').perform(context)),
            'result_file': LaunchConfiguration('result_file').perform(context),
        }])
    # when the scorer is done (task finished / time out) take the whole launch down
    stop = RegisterEventHandler(OnProcessExit(
        target_action=scorer, on_exit=[EmitEvent(event=Shutdown(reason='run finished'))]))
    return [stage, scorer, stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='easy'),
        DeclareLaunchArgument('team', default_value='unnamed'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('time_limit', default_value='1200'),
        DeclareLaunchArgument('result_file', default_value='/tmp/coderun_result.json'),
        OpaqueFunction(function=_setup),
    ])
