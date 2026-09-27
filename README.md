# Резервная одометрия трамвая

Решение кейса хакатона Московского транспорта «Резервная одометрия по модели» —
пакет ROS 2 Humble. Нода в реальном времени оценивает продольную скорость и положение
трамвая по скоростям колёс передней и задней тележки и положению ручки контроллера.
Скорость — фильтр Калмана с моделью тягового привода вагона 71-911ЕМ, положение —
интегрирование вдоль 1D-карты маршрута со сбросом ошибки на остановках. GNSS нужен
только для начальной выставки и коррекции положения, когда фиксы есть. Результат
публикуется в `/result/velocity` (м/с) и `/result/position` (base_link в MGRS).

| Каталог | Что внутри |
|---|---|
| [`src/tram_reserve_odometry`](src/tram_reserve_odometry/README.md) | пакет решения: нода, плагины алгоритмов, 1D-карта, конфиг, launch, офлайн-прогон, документация |
| `src/tram_vehicle_msgs` | сообщения: скорость тележки, команда ручки |
| `src/checker_ros` | чекер организаторов `hackathon_solution_checker` |

## Документация

| Тема | Документ |
|---|---|
| математическая модель: уравнения, структура, входы/выходы, оценка ускорения, переход от момента к скорости | [`docs/math_model.md`](src/tram_reserve_odometry/docs/math_model.md) |
| паспорт вагона 71-911ЕМ ↔ коэффициенты модели, сверка с данными | [`docs/physical_model.md`](src/tram_reserve_odometry/docs/physical_model.md) |
| допущения, ограничения модели, конфигурируемые параметры | [`docs/parameters.md`](src/tram_reserve_odometry/docs/parameters.md), [`configs/`](src/tram_reserve_odometry/configs/) |
| точность и быстродействие: методика, ошибки, задержка, частота, ресурсы | [`docs/evaluation.md`](src/tram_reserve_odometry/docs/evaluation.md) |
| ограничения решения и план развития | [`docs/roadmap.md`](src/tram_reserve_odometry/docs/roadmap.md) |
| архитектура ноды, системы координат, формат данных, плагины | [`docs/architecture.md`](src/tram_reserve_odometry/docs/architecture.md), [`docs/coordinates.md`](src/tram_reserve_odometry/docs/coordinates.md), [`docs/data_format.md`](src/tram_reserve_odometry/docs/data_format.md), [`docs/plugins.md`](src/tram_reserve_odometry/docs/plugins.md) |

Инструкция по проверке — ниже.

## Сборка

Нужен ROS 2 Humble. В Docker, из корня репозитория:

```bash
docker run -it --rm -v $(pwd):/workspace -w /workspace ros:humble-ros-base bash
# внутри контейнера:
colcon build --base-paths src
source install/setup.bash
```

Если решение кладётся в ваш workspace, где уже есть `tram_vehicle_msgs`, скопируйте в
`src/` только `src/tram_reserve_odometry` — два пакета с одним именем colcon не соберёт.

В src/ уже лежит checker_ros и tram_vehicle_msgs из тестового проверочного архива

## Запуск

Одна команда в одном терминале — нода, чекер организаторов (если пакет
`hackathon_solution_checker` собран в том же workspace) и проигрывание bag в 1x:

```bash
ros2 launch tram_reserve_odometry reserve_odometry.launch.py bag:=<путь к bag>
```

Что происходит:

1. стартует нода — в логе строка `reserve_odometry ready: rate=20.0Hz ...`;
2. стартует чекер (`hackathon_solution_checker metrics`), каждые 5 с печатает RMSE и max;
3. через 5 с (`bag_delay`) начинает играть bag; в логе ноды — `Map init (compass): ...`
   (привязка к карте по GNSS первых секунд);
4. bag доиграл — launch завершается сам, чекер печатает итоговые метрики.

Аргументы: `bag` (пусто — только нода, bag проигрывается вручную), `bag_delay` (5.0 с),
`checker` (true; false — без чекера).

## Что на выходе

| Топик | Тип | Содержание |
|---|---|---|
| `/result/velocity` | `tram_vehicle_msgs/VelocitySensor` | продольная скорость, м/с |
| `/result/position` | `nav_msgs/Odometry` | base_link в MGRS (x, y, z) |

Частота 20 Гц, `header.stamp` — время входа из bag. Отладка: `/debug/state`,
`/debug/slip`, `/diagnostics`.

## Логи, частота, задержка

Логи ноды, чекера и bag play — в том же терминале (строки с префиксами
`[tram_reserve_odometry_node-1]`, `[metrics-2]`, `[ros2-3]`) и в `~/.ros/log/`.
Во время прогона, из другого терминала того же контейнера
(`docker exec -it <контейнер> bash`, `source install/setup.bash`):

```bash
ros2 topic hz /result/velocity                       # ~20 Гц
ros2 topic echo /diagnostics | grep -A1 latency_ms   # задержка «приём входа -> публикация», мс
```
