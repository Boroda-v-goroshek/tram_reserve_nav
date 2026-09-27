"""Конфиг ноды: набор мелких файлов без переопределений.

Конфиг = configs/*.yaml (share пакета), загружается как несколько --params-file.
Каждый параметр определяется ровно в одном файле набора.
"""
from __future__ import annotations

import glob
import os


def config_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('tram_reserve_odometry'), 'configs')


def config_set() -> list[str]:
    """Полный набор файлов конфига ноды, порядок детерминирован."""
    base = config_dir()
    files = sorted(glob.glob(os.path.join(base, '*.yaml')))
    if not files:
        raise FileNotFoundError(f'нет конфигов ноды: {base}/*.yaml')
    return files


def read_node_config(params_files: list[str] | None = None) -> dict:
    """Параметры ноды для отрисовки: система вывода (output.*), опорная точка карты
    (map.reference), TF антенн (vehicle.*)."""
    import yaml

    cfg = {'frame': 'mgrs', 'utm_zone': 37, 'mgrs_origin': [300000.0, 6100000.0],
           'enu_reference': [55.810367, 37.462267, 168.0],
           'master_antenna_x': -9.873, 'rover_antenna_x': 2.563, 'point': 'base_link'}
    for pf in (params_files or []):
        if not pf:
            continue
        with open(pf, encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        params = doc.get('reserve_odometry', {}).get('ros__parameters', {})
        out = params.get('output', {})
        for k_yaml, k_cfg in (('frame', 'frame'), ('utm_zone', 'utm_zone'),
                              ('mgrs_origin', 'mgrs_origin'), ('point', 'point')):
            if k_yaml in out:
                cfg[k_cfg] = out[k_yaml]
        if 'reference' in params.get('map', {}):
            cfg['enu_reference'] = [float(x) for x in params['map']['reference']]
        veh = params.get('vehicle', {})
        for k in ('master_antenna_x', 'rover_antenna_x'):
            if k in veh:
                cfg[k] = float(veh[k])
    return cfg
