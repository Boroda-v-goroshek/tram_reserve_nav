# Плагины (сменные алгоритмы)

Алгоритмы — это **плагины pluginlib**, выбираемые по имени в конфиге
(`configs/plugins.yaml`, секция `plugins`). Нода и пайплайн не зависят от
конкретного алгоритма: чтобы заменить модель, достаточно добавить плагин и
поменять имя в конфиге.

## Интерфейсы (в `include/tram_reserve_odometry/core/interfaces.hpp`)

| Интерфейс | Назначение | Состояние |
|-----------|-----------|-----------|
| `IPreprocessor` | сырые 3 канала → `PreprocessedSample` (опорная метка, единицы) | опц. состояние |
| `ITractionModel` | входы → ускорение модели (м/с²) | без состояния |
| `ISlipDetector` | детекция проскальзывания/потери сцепления | без состояния |
| `IStateEstimator` | оценка скорости/ускорения | **состояние** между вызовами |
| `IPositionIntegrator` | интеграция скорости в положение | **состояние** между вызовами |
| `IPositionInitializer` | начальное положение/скорость | без состояния |

Все интерфейсы принимают `ModelParams` и **не знают о ROS**. `IPreprocessor`
получает сырые `InputSample` трёх каналов (может быть `valid=false`) и
возвращает `PreprocessedSample`; остальные — уже нормализованный
`PreprocessedSample` (м/с, ручка в [−1, 1]).

## Встроенные плагины

| Имя в конфиге | Что делает |
|---------------|-----------|
| `tram::PassthroughPreprocessor` | не трогает данные — только опорный stamp, приведение единиц (км/ч → м/с, ручка → [−1, 1]) и presence-качество |
| `tram::TableTractionModel` | табличная характеристика тяги `f(u, v)` из `maps/drive_map.csv`; выше края таблицы — постоянная мощность 4 × 72 кВт |
| `tram::AlwaysGoodSlipDetector` | всегда «без проскальзывания», уверенность 1.0 (проскальзывание оценивает `KalmanWheelEstimator`) |
| `tram::KalmanWheelEstimator` | фильтр скорости `[v, a, θ_T, θ_B, b]`: режим «модель привода» с fallback на кинематику Зингера, робастные правила отбраковки, ZUPT, прогноз к метке публикации |
| `tram::StopResetRouteIntegrator` | ведёт маршрут и `s = s0 + direction·∫v dt` для base_link, читает `(x, y, z, heading)` с 1D-карты, с конца ветки переходит на кольцо (база `RouteIntegrator`); плюс сброс продольной ошибки `s` на стоянках у остановок из `maps/stops.yaml` |
| `tram::ZeroPositionInitializer` | старт из `initialization.initial_position/velocity` |

## Сброс ошибки пути на остановках (`StopResetRouteIntegrator`)

Остановки — **только** из `maps/stops.yaml`: вручную проверенные места, где
стоит base_link у реальных остановок маршрута (17 на кольце, соседние — не
ближе 200 м). Своих точек интегратор не добавляет; на ветках остановок нет.

1. **Неопределённость пути.** Между привязками ошибка `s` растёт с пройденным
   путём `d`: `σ² = σ_ref² + (drift_rel·d)² + drift_q²·d` — систематическая
   часть (масштаб колёс) и случайная (проскальзывания). `σ_ref` —
   неопределённость в момент последней привязки. После выставки
   `σ_ref² = stop_sigma_init² + ct²`, где `ct` — поперечное отклонение
   base_link от пути при выставке (нода передаёт его через
   `set_initial_uncertainty`): старт вне карты или фикс без RTK дают большую
   ошибку.
2. **Стоянка.** `|v| < stop_v_still` дольше `stop_t_still` на кольце — те же
   пороги, по которым строился `stops.yaml`. Одна попытка на стоянку.
3. **Окно.** Ближайшая остановка должна лежать в
   `|s_ост − s| ≤ clamp(stop_gate_k·√S, stop_gate_min, stop_gate_max)`,
   `S = σ² + stop_sigma²`. Стоянка вне окна (светофор, пробка) `s` не трогает.
4. **Сброс.** `s ← s + K·(s_ост − s)`, `K = σ²/S`: при `σ ≫ stop_sigma` это
   практически `s = s_ост`. Затем `σ_ref² = (1 − K)·σ²`, отсчёт `d` — заново.
5. **Подъезд к стоп-линии.** Остановка, по которой уже сбросились, повторно не
   используется, пока трамвай не отъедет от неё дальше `stop_rearm_m`: подъезд
   на несколько метров после посадки — движение, измеренное колёсами.

В `Odometry` уходит `position_cov = σ²`. Параметры — `stop_*` в секции
`integrator:` `configs/integrator.yaml`; `stop_enabled: false` отключает сброс.

## Как добавить свой плагин

1. **Реализовать интерфейс** (header + cpp) в `include/tram_reserve_odometry/plugins/`
   и `src/plugins/`. Пример — `zero_position_initializer.hpp`:

   ```cpp
   class MyIntegrator : public IPositionIntegrator
   {
   public:
     void integrate(Estimate & est, double dt, const ModelParams & params) override { /* ... */ }
     std::string name() const override {return "tram::MyIntegrator";}
     void reset() override {}
   };
   ```

2. **Добавить источник** в `CMakeLists.txt` (в `add_library(tram_reserve_odometry_plugins SHARED ...)`).

3. **Зарегистрировать** в `plugins.xml`:

   ```xml
   <class name="tram::MyIntegrator" type="tram::MyIntegrator" base_class="tram::IPositionIntegrator">
     <description>Описание</description>
   </class>
   ```

4. **Выбрать в конфиге**:

   ```yaml
   plugins:
     integrator: tram::MyIntegrator
   ```

5. Пересобрать: `colcon build --packages-select tram_reserve_odometry`.

> Имя класса в `type` и `name` в `plugins.xml` должно совпадать с именем,
> указанным в конфиге. Нода загружает плагин через `create_plugin<T>(package,
> base_class, name)`.

## Связанные документы

- Где плагины стоят в цикле ноды и как связаны с матмоделями —
  [`architecture.md`](architecture.md) (разделы «Один цикл», «Модули ↔ матмодели»).
- Имена плагинов и числовые параметры — в [`configs/`](../configs/) пакета.
