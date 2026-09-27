"""Логика отрисовки карты без ROS: карта в выходной системе ноды, эталон base_link из
двух GNSS-антенн, хвосты траекторий, ошибка предсказания, камера rviz2."""
from __future__ import annotations

import math
import os
import re
from collections import deque

import numpy as np
import yaml

from . import geo
from .geo import VehicleTf, enu_to_geodetic

DEFAULT_MGRS_ORIGIN = (300000.0, 6100000.0)


# ------------------------------------------------------------------ карта

def load_map(map_dir: str) -> dict:
    """Маршрут: кольцо, ветки, остановки. Данные — lat/lon/alt + x_m/y_m (ENU карты)."""
    ring_path = os.path.join(map_dir, 'route_ring.csv')
    with open(ring_path, encoding='utf-8') as f:
        header = f.readline()
    m = re.search(r'B: s in \[([\d.]+)', header)
    b_start = float(m.group(1)) if m else 5817.8
    # route_ring.csv начинается с комментария '#' — пропускаем его явно.
    ring = np.genfromtxt(ring_path, delimiter=',', names=True, skip_header=1)
    branches = np.atleast_1d(np.genfromtxt(os.path.join(map_dir, 'route_branches.csv'),
                                           delimiter=',', names=True))
    # stops.yaml содержит названия остановок (UTF-8) в комментариях.
    with open(os.path.join(map_dir, 'stops.yaml'), encoding='utf-8') as f:
        stops = yaml.safe_load(f) or {}
    return {'ring': ring, 'branches': branches, 'stops': stops, 'b_start': b_start}


def stop_points(ring, stops: dict, b_start: float, master_x: float = -9.873):
    """Остановки -> [(метка, lat, lon)]: s_base_link = s_master − master_x
    (A: s кольца, B: s от начала пути B)."""
    s_arr = ring['s_m']
    pts = []
    for track in sorted(stops):
        for i, s in enumerate(stops[track] or []):
            s_ring = (float(s) if track == 'A' else b_start + float(s)) - master_x
            s_ring = float(np.mod(s_ring, s_arr[-1]))
            pts.append((f'{track}{i + 1}', float(np.interp(s_ring, s_arr, ring['lat_deg'])),
                        float(np.interp(s_ring, s_arr, ring['lon_deg']))))
    return pts


def frame_params(cfg: dict):
    zone = int(cfg.get('utm_zone', 37))
    origin = tuple(float(v) for v in cfg.get('mgrs_origin', DEFAULT_MGRS_ORIGIN))
    return zone, origin


def make_transformer(cfg: dict):
    """(имя системы, функция lat/lon/alt -> (x, y, z) в выходной системе ноды)."""
    frame = cfg.get('frame', 'mgrs')
    if frame == 'mgrs':
        zone, origin = frame_params(cfg)

        def to_frame(lat, lon, alt):
            p = geo.geodetic_to_mgrs(lat, lon, alt, zone, origin)
            return float(p[0]), float(p[1]), float(p[2])
        return 'mgrs', to_frame
    if frame == 'enu':
        ref = tuple(float(v) for v in cfg['enu_reference'])

        def to_frame(lat, lon, alt):
            p = geo.geodetic_to_enu(lat, lon, alt, ref)
            return float(p[0]), float(p[1]), float(p[2])
        return 'enu', to_frame
    raise ValueError(f'unsupported output.frame: {frame!r} (use mgrs or enu)')


def polyline_xy(frame: str, cfg: dict, rows) -> tuple:
    """Точки карты (кольцо или ветка) в выходной системе (x, y)."""
    if frame == 'enu':
        return np.asarray(rows['x_m'], float), np.asarray(rows['y_m'], float)
    zone, origin = frame_params(cfg)
    p = geo.geodetic_to_mgrs(rows['lat_deg'], rows['lon_deg'], 0.0, zone, origin)
    return p[..., 0], p[..., 1]


def route_xy(frame: str, cfg: dict, ring) -> tuple:
    return polyline_xy(frame, cfg, ring)


def map_polylines(frame: str, cfg: dict, data: dict) -> dict:
    """{'ring': (xs, ys), 'branch<id>': (xs, ys), ...} в выходной системе."""
    out = {'ring': route_xy(frame, cfg, data['ring'])}
    br = data['branches']
    for bid in sorted(set(int(b) for b in br['branch'])):
        out[f'branch{bid}'] = polyline_xy(frame, cfg, br[br['branch'] == bid])
    return out


def bounds(polylines: dict):
    xs = np.concatenate([p[0] for p in polylines.values()])
    ys = np.concatenate([p[1] for p in polylines.values()])
    return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())


def near_bounds(x: float, y: float, bbox, margin: float) -> bool:
    x0, y0, x1, y1 = bbox
    return (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin)


def top_down_view(bbox, view_px=(1260, 900), fill=0.9):
    """Камера rviz2 TopDownOrtho на весь маршрут: центр (x, y) и Scale (пикселей на метр)."""
    x0, y0, x1, y1 = bbox
    w, h = max(x1 - x0, 1.0), max(y1 - y0, 1.0)
    scale = fill * min(view_px[0] / w, view_px[1] / h)
    return 0.5 * (x0 + x1), 0.5 * (y0 + y1), scale


def enu_yaw_to_frame_yaw(frame: str, cfg: dict, x: float, y: float, ux: float, uy: float) -> float:
    """Курс в выходной системе; для MGRS — с учётом сближения меридианов."""
    if frame == 'enu':
        return math.atan2(uy, ux)
    ref = tuple(float(v) for v in cfg['enu_reference'])
    zone, origin = frame_params(cfg)
    lat0, lon0, _ = enu_to_geodetic(x, y, 0.0, ref)
    lat1, lon1, _ = enu_to_geodetic(x + ux, y + uy, 0.0, ref)
    p0 = geo.geodetic_to_mgrs(lat0, lon0, 0.0, zone, origin)
    p1 = geo.geodetic_to_mgrs(lat1, lon1, 0.0, zone, origin)
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


# ------------------------------------------------------------------ эталон

class BagClock:
    """«Текущее время bag»: максимум меток header полученных сообщений — для отбраковки
    устаревших GNSS-фиксов."""

    def __init__(self):
        self.now = -math.inf

    def update(self, t: float):
        if t > self.now:
            self.now = t

    def is_stale(self, t: float, tol: float) -> bool:
        return t < self.now - tol


class GtTracker:
    """Эталонная base_link в реальном времени из двух GNSS-антенн."""

    def __init__(self, cfg: dict, pair_tol: float = 0.05, min_status: int = 2,
                 baseline_tol: float = 0.5, stale_tol: float = 0.25):
        self.tf = VehicleTf(master_x=cfg.get('master_antenna_x', -9.873),
                            rover_x=cfg.get('rover_antenna_x', 2.563))
        self.ref = tuple(float(v) for v in cfg['enu_reference'])
        self.pair_tol = pair_tol
        self.min_status = min_status
        self.baseline_tol = baseline_tol
        self.stale_tol = stale_tol
        self.expected = self.tf.rover_x - self.tf.master_x
        self.buf = {'master': deque(maxlen=32), 'rover': deque(maxlen=32)}
        self.last_master_t = -math.inf   # метки монотонны: помним последнюю спаренную
        self.rejected_stale = 0

    def add(self, antenna: str, t: float, lat: float, lon: float, alt: float,
            status: int, clock: BagClock | None = None):
        """Новый фикс; возвращает {'t', 'lat', 'lon', 'alt', 'enu', 'dir'} или None.

        'enu' — base_link (x, y, z) в ENU карты, 'dir' — единичный вектор курса.
        """
        if status < self.min_status:
            return None
        if clock is not None and clock.is_stale(t, self.stale_tol):
            self.rejected_stale += 1
            return None
        self.buf[antenna].append((t, lat, lon, alt))
        other = 'rover' if antenna == 'master' else 'master'
        if not self.buf[other]:
            return None
        best = min(self.buf[other], key=lambda f: abs(f[0] - t))
        if abs(best[0] - t) > self.pair_tol:
            return None
        m, r = ((t, lat, lon, alt), best) if antenna == 'master' else (best, (t, lat, lon, alt))
        if m[0] <= self.last_master_t:
            return None
        pose = self._pose(m, r)
        if pose is not None:
            self.last_master_t = m[0]
        return pose

    def _pose(self, m, r):
        pm = geo.geodetic_to_enu(m[1], m[2], m[3], self.ref)
        pr = geo.geodetic_to_enu(r[1], r[2], r[3], self.ref)
        dx, dy = pr[0] - pm[0], pr[1] - pm[1]
        base = math.hypot(dx, dy)
        if abs(base - self.expected) > self.baseline_tol:
            return None
        ux, uy = dx / base, dy / base
        bx = pr[0] - self.tf.rover_x * ux
        by = pr[1] - self.tf.rover_x * uy
        w = -self.tf.master_x / self.expected
        z = pm[2] + (pr[2] - pm[2]) * w - self.tf.antenna_z
        lat, lon, alt = enu_to_geodetic(bx, by, z, self.ref)
        return {'t': 0.5 * (m[0] + r[0]), 'lat': float(lat), 'lon': float(lon),
                'alt': float(alt), 'enu': (float(bx), float(by), float(z)),
                'dir': (float(ux), float(uy))}


class History:
    """Последние позиции (t, x, y) — для GT в момент времени предсказания."""

    def __init__(self, maxlen: int = 200):
        self.buf = deque(maxlen=maxlen)

    def add(self, t: float, x: float, y: float):
        if self.buf and t <= self.buf[-1][0]:
            return
        self.buf.append((t, x, y))

    def at(self, t: float, max_gap: float = 0.5, tol: float = 1e-3):
        """Линейная интерполяция в момент t; None, если t вне данных или в дыре > max_gap."""
        if not self.buf:
            return None
        ts = [p[0] for p in self.buf]
        if t < ts[0] - tol or t > ts[-1] + tol:
            return None
        i = int(np.searchsorted(ts, t))
        if i >= len(ts):
            return self.buf[-1][1], self.buf[-1][2]
        if i == 0 or abs(ts[i] - t) <= tol:
            return self.buf[i][1], self.buf[i][2]
        (t0, x0, y0), (t1, x1, y1) = self.buf[i - 1], self.buf[i]
        if t1 - t0 > max_gap:
            return None
        a = (t - t0) / (t1 - t0)
        return x0 + a * (x1 - x0), y0 + a * (y1 - y0)


class Trail:
    """Хвост траектории (LINE_LIST): точки через min_step м, разрыв на скачке/паузе."""

    def __init__(self, maxlen: int = 3000, min_step: float = 1.0,
                 max_jump: float = 50.0, max_gap_s: float = 3.0):
        self.pts = deque(maxlen=maxlen)   # (t, x, y, break_before)
        self.min_step = min_step
        self.max_jump = max_jump
        self.max_gap_s = max_gap_s
        self.last_t = None   # последнее полученное сообщение (на стоянке точки не добавляются)

    def add(self, t: float, x: float, y: float) -> bool:
        """Добавить точку; True, если хвост изменился."""
        gap = self.last_t is not None and (t - self.last_t) > self.max_gap_s
        self.last_t = t
        if not self.pts:
            self.pts.append((t, x, y, True))
            return True
        _, x0, y0, _ = self.pts[-1]
        d = math.hypot(x - x0, y - y0)
        brk = d > self.max_jump or gap
        if not brk and d < self.min_step:
            return False
        self.pts.append((t, x, y, brk))
        return True

    def segments(self):
        """[(x0, y0, x1, y1), ...] без отрезков через разрывы."""
        pts = list(self.pts)
        return [(a[1], a[2], b[1], b[2]) for a, b in zip(pts, pts[1:]) if not b[3]]

    def clear(self):
        self.pts.clear()
        self.last_t = None


def tram_arrow(x: float, y: float, yaw: float, length: float = 30.0, nose: float = 4.5):
    """Стрелка «вагон»: от хвоста к носу; base_link ~4.5 м от носа."""
    c, s = math.cos(yaw), math.sin(yaw)
    back = length - nose
    return (x - back * c, y - back * s), (x + nose * c, y + nose * s)
