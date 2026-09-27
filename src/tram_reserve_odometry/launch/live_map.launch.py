"""Запуск отрисовки карты в реальном времени: нода + рисовалка + rviz2 (+ bag play).

    ros2 launch tram_reserve_odometry live_map.launch.py bag:=/path/to/bag

Аргументы:
    bag         — rosbag для проигрывания (пусто — проиграть вручную: ros2 bag play)
    rate        — скорость проигрывания bag (по умолчанию 1.0)
    bag_delay   — задержка перед проигрыванием bag, с (по умолчанию 3.0): нода должна
                   подписаться до первых GNSS-фиксов
    with_node   — true: запустить ноду резервной одометрии (по умолчанию true);
                   false: нода уже запущена в другом терминале
    frame       — система координат ноды и рисовалки: mgrs или enu; пусто — output.frame
                   из конфига
    rviz        — true: запустить rviz2 (по умолчанию true)
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Область отрисовки rviz2 в сгенерированном конфиге (окно 1600x1000 минус панели).
VIEW_PX = (1260, 900)


def _route_view(frame_override=''):
    """Центр и Scale камеры TopDownOrtho на весь маршрут в выходной системе ноды."""
    from tram_reserve_odometry import configs, live_map_core as core

    cfg = configs.read_node_config(configs.config_set())
    if frame_override in ('mgrs', 'enu'):
        cfg['frame'] = frame_override
    if cfg.get('frame') == 'relative':
        cfg['frame'] = 'enu'
    maps = os.path.join(get_package_share_directory('tram_reserve_odometry'), 'maps')
    bbox = core.bounds(core.map_polylines(cfg['frame'], cfg, core.load_map(maps)))
    return core.top_down_view(bbox, VIEW_PX), bbox


def _write_rviz_config(view, bbox):
    """Генерация .rviz: top-down view на весь маршрут, все /viz/* маркеры.

    Fixed Frame — 'map' (frame_id всех маркеров); система координат (mgrs/enu)
    влияет только на сами координаты, а не на имя фрейма.
    """
    x, y, scale = view
    # Сетка 100 м под маршрутом.
    cell = 100.0
    cells = int(max(bbox[2] - bbox[0], bbox[3] - bbox[1]) / cell) + 4
    gx, gy = round(x / cell) * cell, round(y / cell) * cell
    cfg = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.25
      Cell Size: {cell:.0f}
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid 100 m
      Normal Cell Count: 0
      Offset:
        X: {gx:.1f}
        Y: {gy:.1f}
        Z: -0.5
      Plane: XY
      Plane Cell Count: {cells}
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Map
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /viz/map
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Trails
      Topic:
        Value: /viz/trails
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Predicted
      Topic:
        Value: /viz/predicted
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: GT
      Topic:
        Value: /viz/gt
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Error
      Topic:
        Value: /viz/error
      Value: true
    - Class: rviz_default_plugins/Marker
      Enabled: true
      Name: Status
      Topic:
        Value: /viz/status
      Value: true
  Enable Stereo Rendering:
    Value: false
  Global Options:
    Background Color: 48; 50; 49
    Fixed Frame: map
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/Measure
      Line color: 255; 85; 255
  Views:
    Current:
      Angle: 0
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      # Scale — пикселей на метр: весь маршрут в окне.
      Scale: {scale:.4f}
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz_default_plugins)
      X: {x:.3f}
      Y: {y:.3f}
    Saved: ~
Window Geometry:
  Height: 1000
  Width: 1600
  X: 40
  Y: 40
"""
    path = os.path.join(tempfile.gettempdir(), 'tram_live_map.rviz')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(cfg)
    return path


def _start(context):
    from tram_reserve_odometry import configs

    with_node = LaunchConfiguration('with_node').perform(context).lower() in ('true', '1')
    frame = LaunchConfiguration('frame').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context).lower() in ('true', '1')

    actions = []
    if with_node:
        params = configs.config_set()
        if frame in ('mgrs', 'enu'):
            # Та же система, что у рисовалки.
            params.append({'output.frame': frame})
        actions.append(Node(
            package='tram_reserve_odometry',
            executable='tram_reserve_odometry_node',
            name='reserve_odometry',
            output='screen',
            parameters=params,
        ))

    viewer_args = []
    if frame in ('mgrs', 'enu'):
        viewer_args += ['--frame', frame]
    actions.append(Node(
        package='tram_reserve_odometry',
        executable='live_map_viewer',
        name='live_map_viewer',
        output='screen',
        arguments=viewer_args,
    ))

    if rviz:
        view, bbox = _route_view(frame)
        actions.append(ExecuteProcess(
            cmd=['rviz2', '-d', _write_rviz_config(view, bbox)],
            output='screen',
        ))

    bag = LaunchConfiguration('bag').perform(context)
    if bag:
        # Задержка: нода и рисовалка должны подписаться до первых GNSS-фиксов.
        actions.append(TimerAction(
            period=float(LaunchConfiguration('bag_delay').perform(context)),
            actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag,
                     '--rate', LaunchConfiguration('rate').perform(context)],
                output='screen',
            )]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='',
                              description='rosbag для проигрывания (пусто — вручную)'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='Скорость проигрывания bag'),
        DeclareLaunchArgument('bag_delay', default_value='3.0',
                              description='Задержка перед проигрыванием bag, с'),
        DeclareLaunchArgument('with_node', default_value='true',
                              description='true — запустить ноду резервной одометрии'),
        DeclareLaunchArgument('frame', default_value='',
                              description='Система координат ноды и рисовалки: mgrs или enu '
                                          '(пусто — output.frame из конфига ноды)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='true — запустить rviz2'),
        OpaqueFunction(function=_start),
    ])
