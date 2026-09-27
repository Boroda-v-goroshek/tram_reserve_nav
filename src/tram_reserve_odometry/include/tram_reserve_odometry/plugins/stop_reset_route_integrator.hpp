// Интегратор по 1D-карте со сбросом продольной ошибки на остановках.
//
// Ведёт s так же, как RouteIntegrator (s += direction·v·dt), и дополнительно —
// неопределённость s (σ, м). Между привязками ошибка пути растёт с пройденным путём d:
//   σ²(d) = σ_ref² + (drift_rel·d)² + drift_q²·d,
// drift_rel — систематическая часть (масштаб колёс), drift_q — случайная
// (проскальзывания); σ_ref — неопределённость в момент последней привязки
// (выставки по GNSS или остановки).
//
// Остановки — из maps/stops.yaml (s base_link на кольце у остановок маршрута).
//
// Сброс: трамвай стоит (|v| < v_still) дольше t_still на кольце, а ближайшая остановка
// лежит в окне |s_ост − s| ≤ clamp(gate_k·√S, gate_min, gate_max), S = σ² + σ_ост².
// Тогда s ← s + K·(s_ост − s), K = σ²/S, σ_ref² = (1 − K)·σ², d = 0. С teleport — K = 1,
// σ_ref = σ_ост. Одна попытка на стоянку.
// correct_s (GNSS-фикс) — тоже телепорт: s ← s_fix, σ_ref = σ фикса, d = 0.
// Остановка последнего сброса снова доступна, когда трамвай отъехал от неё дальше rearm_m.
// На ветках остановок нет — там ведётся только σ.
//
// Выход: положение с карты (как RouteIntegrator), position_cov = σ².
// Параметры — секция integrator: конфига, ключи stop_* (см. Config).

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__STOP_RESET_ROUTE_INTEGRATOR_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__STOP_RESET_ROUTE_INTEGRATOR_HPP_

#include <cstddef>
#include <string>

#include "tram_reserve_odometry/plugins/route_integrator.hpp"

namespace tram
{

class StopResetRouteIntegrator : public RouteIntegrator
{
public:
  struct Config
  {
    bool enabled = true;         ///< сброс по остановкам (false — ровно RouteIntegrator)
    bool teleport = false;       ///< true — s ровно в точку остановки (K = 1), false — K = σ²/S
    double v_still = 0.1;        ///< «стоим», м/с
    double t_still = 3.0;        ///< стоим дольше — пробуем остановку, с
    double sigma_stop = 1.0;     ///< разброс места стоянки base_link у остановки, м
    double sigma_init = 0.5;     ///< неопределённость s после выставки по RTK, м
    double drift_rel = 0.005;    ///< систематическая ошибка пути, доля пройденного
    double drift_q = 0.05;       ///< случайная ошибка пути, м/√м
    double gate_k = 3.0;         ///< окно поиска остановки, сигм
    double gate_min = 3.0;       ///< окно не меньше, м
    double gate_max = 20.0;      ///< окно не больше, м
    double rearm_m = 50.0;       ///< остановка снова доступна, когда отъехали дальше, м
  };

  /// Счётчики для диагностики.
  struct Stats
  {
    std::size_t resets = 0;      ///< сбросов по остановке
    std::size_t no_stop = 0;     ///< стоянка без остановки в окне (светофор, пробка)
    std::size_t ambiguous = 0;   ///< в окне больше одной остановки
    std::size_t off_ring = 0;    ///< стоянка вне кольца или без карты
    std::size_t gnss = 0;        ///< коррекций по GNSS (correct_s)
  };

  StopResetRouteIntegrator() = default;

  void integrate(Estimate & est, double dt, const ModelParams & params) override;
  std::string name() const override {return "StopResetRouteIntegrator";}
  void reset() override;
  void set_initial_s(double s0) override;
  void set_initial_uncertainty(double sigma_m) override;
  void correct_s(double s, double sigma_m) override;

  static Config config_from(const ModelParams & params);
  /// Текущая неопределённость s, м².
  double sigma2() const;
  const Stats & stats() const {return stats_;}
  /// Поправка s при последнем сбросе, м (0 — сбросов не было).
  double last_correction() const {return last_correction_;}

private:
  /// Стоим -> ищем остановку в окне и сбрасываем s.
  void try_stop();
  /// Уехали от остановки последнего сброса дальше rearm_m -> она снова доступна.
  void rearm_used_stop();
  /// Разность a − b на кольце, в [−L/2, L/2).
  double ring_delta(double a, double b) const;

  Config cfg_;
  bool cfg_loaded_ = false;
  double sigma_ref2_ = -1.0;     ///< σ² в момент привязки; < 0 — задать на ближайшем шаге
  double init_extra_sigma_ = 0.0;  ///< ошибка выставки от ноды, м
  double dist_ = 0.0;            ///< путь с последней привязки, м
  double still_s_ = 0.0;         ///< сколько стоим, с
  bool tried_this_still_ = false;
  int used_stop_ = -1;           ///< индекс остановки последнего сброса (-1 — нет)
  double last_correction_ = 0.0;
  Stats stats_;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__STOP_RESET_ROUTE_INTEGRATOR_HPP_
