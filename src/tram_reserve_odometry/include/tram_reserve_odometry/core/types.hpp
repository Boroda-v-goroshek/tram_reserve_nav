// Типы данных пайплайна (без ROS): входные образцы, предобработанный образец, оценка,
// параметры плагинов.

#ifndef TRAM_RESERVE_ODOMETRY__CORE__TYPES_HPP_
#define TRAM_RESERVE_ODOMETRY__CORE__TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace tram
{

/// Плоская 2D-точка (метры).
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Пространственная 3D-точка (метры).
struct Point3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// Качество входного канала (или слитого образца).
enum class DataQuality : uint8_t
{
  OK = 0,        ///< Свежий, в диапазоне, согласованный.
  STALE,         ///< Старее максимального возраста входа.
  OUTLIER,       ///< Скачок выше порога выброса.
  MISSING,       ///< Нет образца для этого канала.
  INCONSISTENT   ///< Каналы не согласованы (напр. передняя vs задняя тележка).
};

inline const char * to_string(DataQuality q)
{
  switch (q) {
    case DataQuality::OK: return "OK";
    case DataQuality::STALE: return "STALE";
    case DataQuality::OUTLIER: return "OUTLIER";
    case DataQuality::MISSING: return "MISSING";
    case DataQuality::INCONSISTENT: return "INCONSISTENT";
    default: return "UNKNOWN";
  }
}

/// Статус проскальзывания / сцепления колёс.
enum class SlipStatus : uint8_t
{
  NONE = 0,
  LOW,
  MEDIUM,
  HIGH
};

inline const char * to_string(SlipStatus s)
{
  switch (s) {
    case SlipStatus::NONE: return "NONE";
    case SlipStatus::LOW: return "LOW";
    case SlipStatus::MEDIUM: return "MEDIUM";
    case SlipStatus::HIGH: return "HIGH";
    default: return "UNKNOWN";
  }
}

/// Один сырой образец входного канала (скорость тележки или команда машиниста).
struct InputSample
{
  int64_t stamp_ns = 0;      ///< Таймстамп сообщения в наносекундах.
  int64_t recv_ns = 0;       ///< Момент приёма (steady clock, нс) — для замера задержки.
  double value = 0.0;        ///< Значение канала (м/с для скоростей, cmd для ручки).
  double age_ms = 0.0;       ///< Возраст на момент предобработки (мс).
  DataQuality quality = DataQuality::MISSING;
  bool valid = false;        ///< true, если образец получен и декодирован.
};

/// Предобработанная тройка входов на общем опорном времени.
struct PreprocessedSample
{
  int64_t stamp_ns = 0;          ///< Опорный (синхронизированный) таймстамп, нс.
  double front_bogie_velocity = 0.0;  ///< м/с
  double rear_bogie_velocity = 0.0;   ///< м/с
  double driver_position_cmd = 0.0;   ///< нормированная команда контроллера
  int64_t front_stamp_ns = 0;     ///< метка образца передней тележки (0 — не было)
  int64_t rear_stamp_ns = 0;      ///< метка образца задней тележки (0 — не было)
  int64_t driver_stamp_ns = 0;    ///< метка образца ручки (0 — не было)
  bool front_valid = false;       ///< образец передней тележки получен
  bool rear_valid = false;        ///< образец задней тележки получен
  double age_ms = 0.0;            ///< Максимальный возраст по каналам (мс).
  DataQuality quality = DataQuality::OK;
  bool valid = false;             ///< true, если образец можно использовать (OK или STALE).
};

/// Оценка состояния трамвая.
struct Estimate
{
  int64_t stamp_ns = 0;
  double velocity = 0.0;          ///< Продольная скорость, м/с.
  double acceleration = 0.0;      ///< м/с^2
  Point3D position{0.0, 0.0, 0.0};
  double heading = 0.0;           ///< рад
  SlipStatus slip = SlipStatus::NONE;
  double slip_confidence = 1.0;   ///< 0..1
  DataQuality quality = DataQuality::OK;
  bool initialized = false;
  // Дисперсии (0 — неизвестно).
  double velocity_cov = 0.0;
  double position_cov = 0.0;
  // Привязка к 1D-карте маршрута (валидно, когда on_route == true).
  double s = 0.0;                 ///< дистанция вдоль маршрута (м), mod L для кольца
  std::size_t route_id = 0;       ///< 0 = кольцо, 1..N = ветка
  bool on_route = false;          ///< true, если позиция привязана к карте
  int direction = 1;              ///< направление движения вдоль маршрута: +1 / -1
  /// Режим прогноза оценщика: -1 — нет модели привода, 0 — кинематика, 1 — модель привода.
  int model_mode = -1;
};

/// Параметры плагинов: каждый плагин читает нужные ему поля.
struct ModelParams
{
  // Приведение единиц: тележки км/ч -> м/с, ручка −15…+15 -> [−1, 1].
  double bogie_scale = 1.0 / 3.6;
  double driver_scale = 1.0 / 15.0;
  // Начальные условия.
  Point3D initial_position{0.0, 0.0, 0.0};
  double initial_velocity = 0.0;
  std::string name;                ///< свободное имя плагина/конфига
  std::map<std::string, double> extra;  ///< дополнительные числовые параметры
  std::map<std::string, std::string> extra_str;  ///< строковые параметры (пути к файлам моделей)
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__CORE__TYPES_HPP_
