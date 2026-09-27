"""Отрисовка карты в реальном времени (rviz2): маршрут + предсказанная и GT-позиции.

Нода публикует visualization_msgs для rviz2:
  /viz/map       — маршрут: кольцо + тупиковые ветки + остановки (MarkerArray, latched)
  /viz/predicted — предсказание из /result/position: синяя стрелка-«вагон»
  /viz/gt        — эталонная base_link из двух GNSS-антенн bag: зелёная стрелка-«вагон»
  /viz/trails    — траектории обеих точек (LINE_LIST, с разрывами на скачках)
  /viz/error     — отрезок и подпись ошибки предсказания относительно GT
                   в один и тот же момент времени (MarkerArray)
  /viz/status    — строка состояния над картой (режим ноды, ошибка, наличие GNSS)

GT — из /sensing/gnss/{master,rover}/fix (live_map_core.GtTracker); без GNSS GT и
ошибка скрываются.

Example:
    ros2 launch tram_reserve_odometry live_map.launch.py bag:=/path/to/bag
    # или вручную (нода + bag play в разных терминалах):
    ros2 run tram_reserve_odometry live_map_viewer
    ros2 bag play /path/to/bag
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess

import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA
from tram_vehicle_msgs.msg import VelocitySensor
from visualization_msgs.msg import Marker, MarkerArray

from . import configs
from . import live_map_core as core

C_RING = (0.78, 0.80, 0.85, 1.0)
C_BRANCH = (0.95, 0.62, 0.25, 1.0)
C_STOP = (1.0, 0.85, 0.20, 1.0)
C_PRED = (0.15, 0.45, 1.0, 1.0)
C_PRED_TRAIL = (0.15, 0.45, 1.0, 0.6)
C_GT = (0.10, 0.90, 0.30, 1.0)
C_GT_TRAIL = (0.10, 0.90, 0.30, 0.6)
C_ERROR = (1.0, 0.15, 0.15, 0.95)
C_TEXT = (0.95, 0.95, 0.95, 1.0)

# Размеры, м (умножаются на --width-scale).
W_ROUTE = 4.0
W_TRAIL = 3.0
W_ERROR = 2.0
ARROW_SHAFT, ARROW_HEAD, ARROW_HEAD_LEN = 4.0, 9.0, 10.0
STOP_SIZE, STOP_TEXT = 8.0, 12.0
# Слои по высоте: всё рисуется в плоскости карты.
Z_MAP, Z_TRAIL, Z_ERROR, Z_ARROW, Z_TEXT = 0.0, 0.1, 0.2, 0.3, 1.0

GT_TIMEOUT_S = 1.5        # нет свежего GT дольше — скрыть GT и ошибку
UNBOUND_MARGIN_M = 2000.0  # предсказание дальше от карты — нода не привязана к ней


def _rgba(rgb) -> ColorRGBA:
    r, g, b, a = rgb
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _vec3(x, y, z) -> Vector3:
    return Vector3(x=float(x), y=float(y), z=float(z))


def _point(x, y, z=0.0) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def _default_map_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('tram_reserve_odometry'), 'maps')


load_map = core.load_map
route_xy = core.route_xy


class LiveMapViewer(Node):
    """Маршрут + две точки (предсказание/GT) со стрелками направления для rviz2."""

    def __init__(self, cfg: dict, map_dir: str, trail_len: int = 4000,
                 show_trails: bool = True, show_error: bool = True,
                 show_stops: bool = True, relative_frame: bool = False,
                 width_scale: float = 1.0):
        super().__init__('live_map_viewer')
        self.ws = float(width_scale)
        self.cfg = cfg
        self.frame_name, self.to_frame = core.make_transformer(cfg)
        self.frame_id = 'map'
        self.show_trails = show_trails
        self.show_error = show_error
        self.show_stops = show_stops
        # output.frame=relative: координаты ноды не совпадают ни с какой картой.
        self.relative_frame = relative_frame

        data = core.load_map(map_dir)
        self.polylines = core.map_polylines(self.frame_name, cfg, data)
        self.bbox = core.bounds(self.polylines)
        self.map_markers = self._build_map(data)

        self.clock = core.BagClock()
        self.gt_tracker = core.GtTracker(cfg)
        self.pred = None            # (t, x, y, yaw)
        self.gt = None              # (t, x, y, yaw)
        self.pred_bound = None      # None — ещё не было /result/position
        self.pred_hist = core.History(maxlen=400)
        # Точка хвоста — через >= 2 м пути.
        self.trail_pred = core.Trail(maxlen=trail_len, min_step=2.0)
        self.trail_gt = core.Trail(maxlen=trail_len, min_step=2.0)
        self._trails_dirty = False
        self.error = None           # (метры, (x_pred, y_pred), (x_gt, y_gt))
        self._shown = {'gt': False, 'error': False, 'pred': False}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_map = self.create_publisher(MarkerArray, '/viz/map', latched)
        self.pub_pred = self.create_publisher(Marker, '/viz/predicted', 10)
        self.pub_gt = self.create_publisher(Marker, '/viz/gt', 10)
        self.pub_trails = self.create_publisher(MarkerArray, '/viz/trails', 10)
        self.pub_error = self.create_publisher(MarkerArray, '/viz/error', 10)
        self.pub_status = self.create_publisher(Marker, '/viz/status', 10)
        self.create_subscription(Odometry, '/result/position', self.on_position, 10)
        self.create_subscription(NavSatFix, '/sensing/gnss/master/fix',
                                 self._make_fix_cb('master'), 10)
        self.create_subscription(NavSatFix, '/sensing/gnss/rover/fix',
                                 self._make_fix_cb('rover'), 10)
        # Часы bag для отбраковки устаревших GNSS-фиксов.
        self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity',
                                 lambda m: self.clock.update(_stamp_s(m.header.stamp)), 10)

        self.pub_map.publish(self.map_markers)
        # latched + периодический повтор.
        self.create_timer(5.0, lambda: self.pub_map.publish(self.map_markers))
        self.create_timer(0.2, self._on_timer)
        if relative_frame:
            self.get_logger().warn(
                'output.frame=relative: позиция ноды не привязана к карте — '
                'предсказание рисоваться не будет (используйте mgrs или enu).')
        self.get_logger().info(
            f'Live map viewer: frame={self.frame_name}, map={map_dir}, '
            f'bbox=({self.bbox[0]:.0f}, {self.bbox[1]:.0f})-({self.bbox[2]:.0f}, {self.bbox[3]:.0f})')

    # ------------------------------------------------------------ helpers

    def _marker(self, ns, mid, mtype, rgb, scale, stamp=None) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = stamp or self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation = yaw_to_quaternion(0.0)
        m.color = _rgba(rgb)
        m.scale = _vec3(*scale)
        return m

    def _delete(self, ns, mid=0) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        return m

    # ------------------------------------------------------------------ map

    def _build_map(self, data) -> MarkerArray:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for mid, (name, (xs, ys)) in enumerate(self.polylines.items()):
            rgb = C_RING if name == 'ring' else C_BRANCH
            m = self._marker(name, mid, Marker.LINE_STRIP, rgb, (W_ROUTE * self.ws, 0.0, 0.0), stamp)
            m.points = [_point(x, y, Z_MAP) for x, y in zip(xs, ys)]
            arr.markers.append(m)
        if self.show_stops:
            pts = core.stop_points(data['ring'], data['stops'], data['b_start'],
                                   master_x=float(self.cfg.get('master_antenna_x', -9.873)))
            for i, (label, lat, lon) in enumerate(pts):
                x, y, _ = self.to_frame(lat, lon, 0.0)
                m = self._marker('stop', 100 + i, Marker.SPHERE, C_STOP, (STOP_SIZE * self.ws,) * 3, stamp)
                m.pose.position = _point(x, y, Z_MAP)
                arr.markers.append(m)
                t = self._marker('stop_label', 200 + i, Marker.TEXT_VIEW_FACING, C_STOP,
                                 (0.0, 0.0, STOP_TEXT), stamp)
                t.text = label
                t.pose.position = _point(x, y + 1.2 * STOP_TEXT, Z_TEXT)
                arr.markers.append(t)
        return arr

    # ------------------------------------------------------------- positions

    def _make_fix_cb(self, antenna):
        def cb(msg: NavSatFix):
            t = _stamp_s(msg.header.stamp)
            pose = self.gt_tracker.add(antenna, t, msg.latitude, msg.longitude,
                                       msg.altitude, msg.status.status, self.clock)
            if pose is None:
                return
            if self.frame_name == 'enu':
                x, y, _ = pose['enu']
            else:
                x, y, _ = self.to_frame(pose['lat'], pose['lon'], pose['alt'])
            yaw = core.enu_yaw_to_frame_yaw(self.frame_name, self.cfg, *pose['enu'][:2],
                                            *pose['dir'])
            self.gt = (pose['t'], x, y, yaw)
            self._trails_dirty |= self.trail_gt.add(pose['t'], x, y)
            self._update_error()
            self._publish_arrow(self.pub_gt, 'gt', self.gt, C_GT)
            self._publish_error()
        return cb

    def on_position(self, msg: Odometry):
        t = _stamp_s(msg.header.stamp)
        p = msg.pose.pose.position
        x, y = float(p.x), float(p.y)
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.clock.update(t)
        bound = (not self.relative_frame) and core.near_bounds(x, y, self.bbox, UNBOUND_MARGIN_M)
        if bound != self.pred_bound:
            if bound:
                self.get_logger().info('Предсказание привязано к карте — рисую.')
            else:
                self.get_logger().warn(
                    f'Предсказание ({x:.0f}, {y:.0f}) вне карты в системе {self.frame_name}: '
                    'нода в относительном режиме (нет GNSS-привязки) или другая output.frame. '
                    'На карте не рисую.')
            self.trail_pred.clear()
            self._trails_dirty = True
            self.pred_hist = core.History(maxlen=400)
            self.pred_bound = bound
        if not bound:
            self.pred = None
            return
        self.pred = (t, x, y, yaw)
        self.pred_hist.add(t, x, y)
        self._trails_dirty |= self.trail_pred.add(t, x, y)
        self._update_error()
        self._publish_arrow(self.pub_pred, 'pred', self.pred, C_PRED)
        self._publish_error()

    def _update_error(self):
        """Ошибка в момент последнего GT: предсказание интерполируется к его метке."""
        if self.gt is None:
            return
        tg, xg, yg, _ = self.gt
        p = self.pred_hist.at(tg, max_gap=0.5)
        if p is not None:
            self.error = (math.hypot(p[0] - xg, p[1] - yg), p, (xg, yg))

    def _gt_fresh(self) -> bool:
        return self.gt is not None and self.clock.now - self.gt[0] <= GT_TIMEOUT_S

    # -------------------------------------------------------------- publish

    def _publish_arrow(self, pub, ns, pose, rgb):
        _, x, y, yaw = pose
        m = self._marker(ns, 0, Marker.ARROW, rgb,
                         (ARROW_SHAFT * self.ws, ARROW_HEAD * self.ws, ARROW_HEAD_LEN * self.ws))
        (x0, y0), (x1, y1) = core.tram_arrow(x, y, yaw)
        m.points = [_point(x0, y0, Z_ARROW), _point(x1, y1, Z_ARROW)]
        pub.publish(m)
        self._shown[ns] = True

    def _publish_trails(self):
        """Хвосты — по таймеру и только при изменениях."""
        if self.show_trails and self._trails_dirty:
            self._trails_dirty = False
            arr = MarkerArray()
            for mid, trail, rgb in ((0, self.trail_pred, C_PRED_TRAIL),
                                    (1, self.trail_gt, C_GT_TRAIL)):
                segs = trail.segments()
                if not segs:
                    arr.markers.append(self._delete('trail', mid))
                    continue
                m = self._marker('trail', mid, Marker.LINE_LIST, rgb, (W_TRAIL * self.ws, 0.0, 0.0))
                for x0, y0, x1, y1 in segs:
                    m.points += [_point(x0, y0, Z_TRAIL), _point(x1, y1, Z_TRAIL)]
                arr.markers.append(m)
            self.pub_trails.publish(arr)

    def _publish_error(self):
        if not self.show_error:
            return
        if self.error is None or not self._gt_fresh() or self.pred is None:
            if self._shown['error']:
                self.pub_error.publish(MarkerArray(markers=[self._delete('error', 0),
                                                            self._delete('error', 1)]))
                self._shown['error'] = False
            return
        err, (xp, yp), (xg, yg) = self.error
        line = self._marker('error', 0, Marker.LINE_LIST, C_ERROR, (W_ERROR * self.ws, 0.0, 0.0))
        line.points = [_point(xp, yp, Z_ERROR), _point(xg, yg, Z_ERROR)]
        txt = self._marker('error', 1, Marker.TEXT_VIEW_FACING, C_ERROR, (0.0, 0.0, 12.0))
        txt.text = f'{err:.1f} m'
        txt.pose.position = _point(xg, yg + 20.0, Z_TEXT)
        self.pub_error.publish(MarkerArray(markers=[line, txt]))
        self._shown['error'] = True

    def _on_timer(self):
        # GNSS пропал: не держать GT и ошибку на последнем фиксе.
        if self._shown['gt'] and not self._gt_fresh():
            self.pub_gt.publish(self._delete('gt'))
            self._shown['gt'] = False
            self._publish_error()
        if self._shown['pred'] and self.pred is None:
            self.pub_pred.publish(self._delete('pred'))
            self._shown['pred'] = False
            self._publish_error()
        self._publish_trails()
        self._publish_status()

    def _publish_status(self):
        parts = []
        if self.pred_bound is None:
            parts.append('pred: waiting /result/position')
        elif not self.pred_bound:
            parts.append('pred: NOT bound to map (relative odometry)')
        else:
            parts.append('pred: on map')
        if self._gt_fresh():
            parts.append('GT: GNSS ok')
            if self.error is not None:
                parts.append(f'error {self.error[0]:.1f} m')
        elif self.gt is not None:
            parts.append(f'GT: no GNSS for {self.clock.now - self.gt[0]:.0f} s')
        else:
            parts.append('GT: no GNSS')
        x0, _, x1, y1 = self.bbox
        m = self._marker('status', 0, Marker.TEXT_VIEW_FACING, C_TEXT, (0.0, 0.0, 40.0))
        m.text = '   |   '.join(parts)
        m.pose.position = _point(0.5 * (x0 + x1), y1 + 120.0, Z_TEXT)
        self.pub_status.publish(m)


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Real-time map viewer (rviz2): route + predicted and GT positions')
    p.add_argument('--bag', default='',
                   help='rosbag to play (if not given, play it manually in another terminal)')
    p.add_argument('--rate', type=float, default=1.0, help='Bag play rate')
    p.add_argument('--start-offset', type=float, default=0.0,
                   help='Skip N seconds of the bag before replay')
    p.add_argument('--map-dir', default='',
                   help='Directory with map CSV/YAML (default: package maps/)')
    p.add_argument('--frame', default='', choices=['', 'mgrs', 'enu'],
                   help='Frame of /result/position (default: output.frame from node config). '
                        'Must match what the node publishes.')
    p.add_argument('--trail-len', type=int, default=4000,
                   help='Trail length, points (one point per >= 2 m of travel)')
    p.add_argument('--width-scale', type=float, default=1.0,
                   help='Scale for line widths and arrow sizes (thicker for overview, '
                        'thinner when zoomed in)')
    p.add_argument('--no-trails', action='store_true', help='Do not draw trails')
    p.add_argument('--no-error-line', action='store_true', help='Do not draw error line')
    p.add_argument('--no-stops', action='store_true', help='Do not draw stops')
    # parse_known_args: при запуске через launch/ros2 run приходят --ros-args.
    return p.parse_known_args(argv)[0]


def _stop(proc):
    """Остановить процесс bag play (всю группу: ros2 + дочерние)."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    except ProcessLookupError:
        pass


def main(argv=None):
    rclpy.init()
    args = parse_args(argv)

    cfg = configs.read_node_config(configs.config_set())
    if args.frame:
        cfg['frame'] = args.frame
    relative = cfg.get('frame') == 'relative'
    if relative:
        # Карту рисуем во внутренней ENU; предсказание в этом режиме не привязано.
        cfg['frame'] = 'enu'
    map_dir = args.map_dir or _default_map_dir()

    node = LiveMapViewer(cfg, map_dir, trail_len=args.trail_len,
                         show_trails=not args.no_trails,
                         show_error=not args.no_error_line,
                         show_stops=not args.no_stops,
                         relative_frame=relative,
                         width_scale=args.width_scale)

    bag_proc = None
    if args.bag:
        cmd = ['ros2', 'bag', 'play', args.bag, '--rate', str(args.rate)]
        if args.start_offset > 0:
            cmd += ['--start-offset', str(args.start_offset)]
        bag_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, start_new_session=True)
        node.get_logger().info(f'Playing bag: {args.bag}')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if bag_proc is not None and bag_proc.poll() is not None:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    _stop(bag_proc)
    node.destroy_node()
    # shutdown — только если контекст ещё жив (SIGINT-хендлер rclpy мог его закрыть).
    if rclpy.ok():
        rclpy.shutdown()
    return 0
