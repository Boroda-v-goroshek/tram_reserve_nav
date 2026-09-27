// Интерфейсы плагинов (без ROS). Реализации загружаются по имени через реестр плагинов.

#ifndef TRAM_RESERVE_ODOMETRY__CORE__INTERFACES_HPP_
#define TRAM_RESERVE_ODOMETRY__CORE__INTERFACES_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

class RouteMap;

/// Предобработка: последние сырые образцы трёх каналов (тележки, ручка) ->
/// нормализованный PreprocessedSample.
class IPreprocessor
{
public:
  virtual ~IPreprocessor() = default;

  /// \param front/rear/driver последние сырые образцы каналов (может быть valid=false).
  /// \return нормализованный образец (м/с, ручка в [−1, 1]).
  virtual PreprocessedSample process(
    const InputSample & front,
    const InputSample & rear,
    const InputSample & driver,
    const ModelParams & params) = 0;

  virtual void reset() = 0;
  virtual std::string name() const = 0;
};

/// Преобразует предобработанные входы в ускорение модели.
class ITractionModel
{
public:
  virtual ~ITractionModel() = default;
  /// \return ускорение модели в м/с^2.
  virtual double compute_acceleration(
    const PreprocessedSample & sample, const ModelParams & params) const = 0;
  virtual std::string name() const = 0;

  /// Есть ли у модели характеристика привода f(u, v).
  virtual bool has_drive_model(const ModelParams &) const {return false;}
  /// Удельная сила привода, м/с², на позиции ручки notch (−15…+15) при скорости v, м/с:
  /// тяга > 0, торможение < 0; без сопротивления движению и уклона.
  virtual double drive_force(double /*notch*/, double /*v*/, const ModelParams &) const
  {
    return 0.0;
  }
};

/// Обнаруживает проскальзывание колёс / потерю сцепления.
class ISlipDetector
{
public:
  virtual ~ISlipDetector() = default;

  struct Result
  {
    SlipStatus status = SlipStatus::NONE;
    double confidence = 1.0;  // 0..1
  };

  virtual Result detect(const PreprocessedSample & sample, const ModelParams & params) const = 0;
  virtual std::string name() const = 0;
};

/// Оценивает скорость и ускорение. Состояние сохраняется между вызовами.
class IStateEstimator
{
public:
  virtual ~IStateEstimator() = default;

  /// \param sample            валидированный вход
  /// \param model_acceleration из модели тяги
  /// \param slip              из детектора проскальзывания
  /// \return скорость и ускорение (положение заполняет интегратор)
  virtual Estimate estimate(
    const PreprocessedSample & sample,
    double model_acceleration,
    const ISlipDetector::Result & slip,
    const ModelParams & params) = 0;
  virtual std::string name() const = 0;
  virtual void reset() = 0;

  /// Модель тяги (характеристика привода) — для оценщиков с моделью динамики.
  virtual void set_traction_model(const std::shared_ptr<const ITractionModel> &) {}
  /// 1D-карта (уклон, кривизна) — для оценщиков, которым нужен путь.
  virtual void set_route_map(const std::shared_ptr<const RouteMap> &) {}
  /// Оценка предыдущего шага: привязка к карте (route_id, s, direction, on_route).
  virtual void set_route_context(const Estimate &) {}
};

/// Интегрирует скорость в положение. Состояние сохраняется между вызовами.
class IPositionIntegrator
{
public:
  virtual ~IPositionIntegrator() = default;

  /// Обновляет est.position (и при необходимости est.heading) на месте, используя est.velocity.
  /// \param dt  шаг времени в секундах (>= 0)
  virtual void integrate(Estimate & est, double dt, const ModelParams & params) = 0;
  virtual std::string name() const = 0;
  virtual void reset() = 0;

  /// 1D-карта маршрута.
  virtual void set_route_map(const std::shared_ptr<const RouteMap> &) {}
  /// Начальное положение на маршруте (s, м).
  virtual void set_initial_s(double) {}
  /// Направление движения вдоль маршрута (+1/-1).
  virtual void set_direction(int) {}
  /// Начальный маршрут (0 = кольцо, 1..N = ветка).
  virtual void set_route_id(std::size_t) {}
  /// Ошибка выставки начального s (м) — добавляется к неопределённости пути.
  virtual void set_initial_uncertainty(double /*sigma_m*/) {}
  /// Коррекция по внешней привязке (GNSS-фикс): s и его неопределённость sigma_m.
  virtual void correct_s(double /*s*/, double /*sigma_m*/) {}
};

/// Начальное положение и скорость.
class IPositionInitializer
{
public:
  virtual ~IPositionInitializer() = default;
  virtual Point3D initialize_position(const ModelParams & params) const = 0;
  virtual double initialize_velocity(const ModelParams & params) const = 0;
  virtual std::string name() const = 0;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__CORE__INTERFACES_HPP_
