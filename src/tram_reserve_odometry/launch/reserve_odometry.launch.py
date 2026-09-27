"""Прогон решения одной командой: нода + чекер организаторов (если установлен) + bag play.

    ros2 launch tram_reserve_odometry reserve_odometry.launch.py bag:=/path/to/bag

Аргументы:
    bag         — rosbag для проигрывания в 1x (пусто — только нода, bag проигрывается вручную)
    bag_delay   — задержка перед проигрыванием bag, с (по умолчанию 5.0): нода должна
                   подписаться до первых GNSS-фиксов
    checker     — true: запустить hackathon_solution_checker, если пакет установлен
                   (по умолчанию true)

Когда bag доигрывает, launch завершается; чекер при остановке печатает итоговые метрики.
"""
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    from tram_reserve_odometry import configs

    actions = [Node(
        package='tram_reserve_odometry',
        executable='tram_reserve_odometry_node',
        name='reserve_odometry',
        output='screen',
        parameters=configs.config_set(),
    )]

    if LaunchConfiguration('checker').perform(context).lower() in ('true', '1'):
        try:
            get_package_share_directory('hackathon_solution_checker')
            actions.append(Node(
                package='hackathon_solution_checker',
                executable='metrics',
                output='screen',
            ))
        except PackageNotFoundError:
            actions.append(LogInfo(msg='hackathon_solution_checker не установлен: без чекера'))

    bag = LaunchConfiguration('bag').perform(context)
    if bag:
        play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag],
            output='screen',
        )
        actions.append(TimerAction(
            period=float(LaunchConfiguration('bag_delay').perform(context)),
            actions=[play]))
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=play,
            on_exit=[LogInfo(msg='ros2 bag play завершён, остановка'),
                     TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown())])])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='',
                              description='rosbag для проигрывания (пусто — вручную)'),
        DeclareLaunchArgument('bag_delay', default_value='5.0',
                              description='Задержка перед проигрыванием bag, с'),
        DeclareLaunchArgument('checker', default_value='true',
                              description='true — запустить hackathon_solution_checker, если есть'),
        OpaqueFunction(function=_start),
    ])
