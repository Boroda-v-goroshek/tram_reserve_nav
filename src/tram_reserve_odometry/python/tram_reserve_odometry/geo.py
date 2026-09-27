"""Геодезия (та же модель, что core/geo.hpp): ENU карты и MGRS
(x = UTM_E − origin_x, y = UTM_N − origin_y, z = высота WGS84)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

A_WGS84 = 6378137.0
E2_WGS84 = 6.69437999014e-3
UTM_K0 = 0.9996
UTM_FALSE_EASTING = 500000.0


def _radii(lat0):
    s = np.sin(np.radians(lat0))
    den = 1.0 - E2_WGS84 * s * s
    return A_WGS84 / np.sqrt(den), A_WGS84 * (1.0 - E2_WGS84) / (den * np.sqrt(den))


def geodetic_to_enu(lat, lon, alt, ref):
    lat0, lon0, alt0 = ref
    rn, rm = _radii(lat0)
    x = np.radians(np.asarray(lon) - lon0) * (rn + alt0) * np.cos(np.radians(lat0))
    y = np.radians(np.asarray(lat) - lat0) * (rm + alt0)
    return np.stack([x, y, np.asarray(alt, float) - alt0], -1)


def enu_to_geodetic(x, y, z, ref):
    """Обратное преобразование ENU -> lat/lon/alt (плоское приближение, точное на длине базы)."""
    lat0, lon0, alt0 = ref
    rn, rm = _radii(lat0)
    lat = lat0 + np.degrees(y / (rm + alt0))
    lon = lon0 + np.degrees(x / ((rn + alt0) * np.cos(np.radians(lat0))))
    return lat, lon, z + alt0


def transverse_mercator(lat, lon, lon0, k0, a=A_WGS84, e2=E2_WGS84):
    """Снайдер (USGS PP 1395), формулы 8-9..8-10; (x восток, y север) без false easting."""
    phi = np.radians(lat)
    ep2 = e2 / (1 - e2)
    sp, cp, tp = np.sin(phi), np.cos(phi), np.tan(phi)
    n = a / np.sqrt(1 - e2 * sp * sp)
    t = tp * tp
    c = ep2 * cp * cp
    aa = np.radians(np.asarray(lon) - lon0) * cp
    e4, e6 = e2 * e2, e2 ** 3
    m = a * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
             - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * np.sin(2 * phi)
             + (15 * e4 / 256 + 45 * e6 / 1024) * np.sin(4 * phi)
             - (35 * e6 / 3072) * np.sin(6 * phi))
    x = k0 * n * (aa + (1 - t + c) * aa ** 3 / 6
                  + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * aa ** 5 / 120)
    y = k0 * (m + n * tp * (aa * aa / 2 + (5 - t + 9 * c + 4 * c * c) * aa ** 4 / 24
                            + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * aa ** 6 / 720))
    return np.stack([x, y], -1)


def geodetic_to_mgrs(lat, lon, alt, zone=37, origin=(300000.0, 6100000.0)):
    p = transverse_mercator(lat, lon, zone * 6.0 - 183.0, UTM_K0)
    alt_arr = np.broadcast_to(np.asarray(alt, float), p[..., 0].shape)
    return np.stack([p[..., 0] + UTM_FALSE_EASTING - origin[0],
                     p[..., 1] - origin[1], alt_arr], -1)


@dataclass
class VehicleTf:
    """TF антенн GNSS в base_link (configs/vehicle.yaml)."""
    master_x: float = -9.873
    rover_x: float = 2.563
    antenna_z: float = 3.0
