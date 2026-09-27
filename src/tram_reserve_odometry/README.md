# tram_reserve_odometry

Пакет резервной одометрии трамвая. В реальном времени оценивает **скорость** и
**положение** по внутренней телеметрии: скоростям колёс передней и задней
тележки и положению ручки контроллера. GNSS используется для начальной
выставки в первые секунды прогона и для коррекции положения, когда фиксы
появляются. Положение ведётся вдоль **1D-карты маршрута** и публикуется в
системе координат судьи — **MGRS** — для точки **base_link** (ось поворота
передней тележки на уровне головки рельса).

Скорость оценивает фильтр Калмана по двум тележкам с моделью тягового привода
(`KalmanWheelEstimator` + `TableTractionModel`), положение — интегратор по
1D-карте со сбросом ошибки на остановках (`StopResetRouteIntegrator`).
Алгоритмы — плагины pluginlib, имена — в `configs/plugins.yaml`.

---

## Состав пакета

| Каталог | Назначение |
|---|---|
| `src/`, `include/` | C++: нода, ядро (типы, интерфейсы, реестр плагинов, геодезия `core/geo.hpp`), плагины, 1D-карта (`map/`), мосты ввода-вывода, офлайн-прогон `src/tools/replay_main.cpp` |
| `configs/` | конфиг ноды: `*.yaml`, каждый параметр — ровно в одном файле ([`docs/parameters.md`](docs/parameters.md)) |
| `maps/` | карта маршрута: `route_ring.csv`, `route_branches.csv`, `stops.yaml`; характеристика привода `drive_map.csv` |
| `launch/` | `live_map.launch.py` — нода + отрисовка карты в rviz2 + bag play |
| `python/tram_reserve_odometry/` | Python для отрисовки: `configs` (набор конфигов), `live_map_viewer`/`live_map_core`, `geo` |
| `scripts/` | точка входа `live_map_viewer` |
| `docs/` | математическая модель, параметры, точность, архитектура, системы координат, формат данных |

---

## Сборка

```bash
colcon build --base-paths src && source install/setup.bash
```

Пакет зависит от `tram_vehicle_msgs` (собирается в том же workspace). Внешних
библиотек для проекции MGRS не нужно: формулы встроены в `core/geo.hpp`.
Тип сборки по умолчанию — Release.

---

## Запуск

Нода берёт параметры из `configs/*.yaml` (каждый файл — `--params-file`); так
её запускают все три способа ниже.

### Проверка через чекер организаторов

`./jury_check.sh` в корне репозитория — см. [README репозитория](../../README.md).

### Офлайн-прогон по bag (replay)

Тот же код ноды, но сообщения читаются из bag в порядке записи, а таймер
публикации имитируется по времени записи bag. Детерминированно и быстрее
реального времени; задержку и ресурсы не меряет.

```bash
ros2 run tram_reserve_odometry reserve_odometry_replay \
  --bag <bag> --out out.csv [--no-driver 1]
```

Выход — CSV: `stamp_s, velocity_mps, x, y, z, position_known, s_m, route_id,
on_route, direction, heading_rad, quality, tick_s`. `--no-driver 1` — прогон без
ручки (отказ канала ручки).

### Отрисовка карты в реальном времени (rviz2)

Нода + рисовалка + rviz2 + bag play одной командой:

```bash
ros2 launch tram_reserve_odometry live_map.launch.py bag:=<bag>
# без rviz (только нода + рисовалка + bag):
ros2 launch tram_reserve_odometry live_map.launch.py bag:=<bag> rviz:=false
```

Аргументы: `bag` (пусто — проигрывать вручную), `rate` (1.0), `bag_delay`
(3.0 с: нода должна подписаться до первых GNSS-фиксов), `with_node` (true),
`frame` (mgrs/enu; пусто — `output.frame` из конфига), `rviz` (true).

Рисовалка публикует `visualization_msgs`: маршрут (`/viz/map`), предсказание
(`/viz/predicted`), эталон из двух GNSS-антенн bag (`/viz/gt`), траектории
(`/viz/trails`), ошибка (`/viz/error`), строка состояния (`/viz/status`).

---

## Топики

- **Входы:** `/vehicle/front_bogie_velocity`, `/vehicle/rear_bogie_velocity`
  (`tram_vehicle_msgs/VelocitySensor`, км/ч), `/vehicle/driver_position_cmd`
  (`tram_vehicle_msgs/DriverControllerCommand`); `/sensing/gnss/{master,rover}/fix`
  — только выставка и коррекция.
- **Выходы (контракт жюри):** `/result/velocity` (`tram_vehicle_msgs/VelocitySensor`,
  м/с), `/result/position` (`nav_msgs/Odometry`, base_link в MGRS), 20 Гц,
  `header.stamp` — время входа.
- **Отладка:** `/debug/state`, `/debug/slip`, `/diagnostics` (качество входов,
  проскальзывание, `latency_ms`, инициализация).

Полный контракт — в [`docs/data_format.md`](docs/data_format.md).

---

## Документация

- [`docs/math_model.md`](docs/math_model.md) — математическая модель: уравнения, входы/выходы, оценка ускорения, переход от силы/момента к скорости, положение;
- [`docs/physical_model.md`](docs/physical_model.md) — паспорт 71-911ЕМ ↔ параметры модели, сверка с данными;
- [`docs/parameters.md`](docs/parameters.md) — допущения, ограничения модели, конфигурируемые параметры;
- [`docs/evaluation.md`](docs/evaluation.md) — методика и результаты: точность, частота, задержка, ресурсы;
- [`docs/roadmap.md`](docs/roadmap.md) — ограничения решения и план развития;
- [`docs/architecture.md`](docs/architecture.md) — слои, один цикл, таймлайн и часы, выставка по GNSS;
- [`docs/coordinates.md`](docs/coordinates.md) — MGRS судьи, ENU карты, base_link, TF антенн;
- [`docs/data_format.md`](docs/data_format.md) — формат данных и контракт жюри;
- [`docs/plugins.md`](docs/plugins.md) — интерфейсы и плагины, как добавить свой.
