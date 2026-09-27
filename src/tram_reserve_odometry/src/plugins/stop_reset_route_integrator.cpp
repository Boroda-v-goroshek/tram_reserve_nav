#include "tram_reserve_odometry/plugins/stop_reset_route_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace tram
{

namespace
{
double get(const ModelParams & p, const char * key, double def)
{
  const auto it = p.extra.find(key);
  return (it == p.extra.end()) ? def : it->second;
}
bool get_flag(const ModelParams & p, const char * key, bool def)
{
  return get(p, key, def ? 1.0 : 0.0) > 0.5;
}
}  // namespace

StopResetRouteIntegrator::Config StopResetRouteIntegrator::config_from(const ModelParams & p)
{
  Config c;
  c.enabled = get_flag(p, "stop_enabled", c.enabled);
  c.teleport = get_flag(p, "stop_teleport", c.teleport);
  c.v_still = get(p, "stop_v_still", c.v_still);
  c.t_still = std::max(get(p, "stop_t_still", c.t_still), 0.0);
  c.sigma_stop = std::max(get(p, "stop_sigma", c.sigma_stop), 0.0);
  c.sigma_init = std::max(get(p, "stop_sigma_init", c.sigma_init), 0.0);
  c.drift_rel = std::max(get(p, "stop_drift_rel", c.drift_rel), 0.0);
  c.drift_q = std::max(get(p, "stop_drift_q", c.drift_q), 0.0);
  c.gate_k = std::max(get(p, "stop_gate_k", c.gate_k), 0.0);
  c.gate_min = std::max(get(p, "stop_gate_min", c.gate_min), 0.0);
  c.gate_max = std::max(get(p, "stop_gate_max", c.gate_max), c.gate_min);
  c.rearm_m = std::max(get(p, "stop_rearm_m", c.rearm_m), 0.0);
  return c;
}

void StopResetRouteIntegrator::reset()
{
  RouteIntegrator::reset();
  sigma_ref2_ = -1.0;
  init_extra_sigma_ = 0.0;
  dist_ = 0.0;
  still_s_ = 0.0;
  tried_this_still_ = false;
  used_stop_ = -1;
  last_correction_ = 0.0;
  stats_ = Stats{};
}

void StopResetRouteIntegrator::set_initial_s(double s0)
{
  RouteIntegrator::set_initial_s(s0);
  // Выставка — новая точка отсчёта ошибки: стоянку до неё не засчитываем.
  sigma_ref2_ = -1.0;
  dist_ = 0.0;
  still_s_ = 0.0;
  tried_this_still_ = false;
  used_stop_ = -1;
}

void StopResetRouteIntegrator::set_initial_uncertainty(double sigma_m)
{
  init_extra_sigma_ = (sigma_m > 0.0) ? sigma_m : 0.0;  // NaN -> 0
  sigma_ref2_ = -1.0;
  dist_ = 0.0;
}

void StopResetRouteIntegrator::correct_s(double s, double sigma_m)
{
  RouteIntegrator::correct_s(s, sigma_m);
  sigma_ref2_ = (sigma_m > 0.0) ? sigma_m * sigma_m : 0.0;
  dist_ = 0.0;
  ++stats_.gnss;
}

double StopResetRouteIntegrator::sigma2() const
{
  const double sys = cfg_.drift_rel * dist_;
  return std::max(sigma_ref2_, 0.0) + sys * sys + cfg_.drift_q * cfg_.drift_q * dist_;
}

void StopResetRouteIntegrator::integrate(Estimate & est, double dt, const ModelParams & params)
{
  if (!cfg_loaded_) {
    cfg_ = config_from(params);
    cfg_loaded_ = true;
  }
  if (sigma_ref2_ < 0.0) {
    sigma_ref2_ = cfg_.sigma_init * cfg_.sigma_init + init_extra_sigma_ * init_extra_sigma_;
  }
  if (dt > 0.0) {
    const double speed = std::abs(est.velocity);
    advance(direction_ * est.velocity * dt);
    dist_ += speed * dt;
    rearm_used_stop();

    // Стоянка -> одна попытка сброса.
    if (speed < cfg_.v_still) {
      still_s_ += dt;
    } else {
      still_s_ = 0.0;
      tried_this_still_ = false;
    }
    if (cfg_.enabled && !tried_this_still_ && still_s_ >= cfg_.t_still) {
      tried_this_still_ = true;
      try_stop();
    }
  }
  fill(est);
  est.position_cov = sigma2();
}

void StopResetRouteIntegrator::try_stop()
{
  if (!(map_ && map_->has_routes()) || route_ != 0) {
    ++stats_.off_ring;  // остановки есть только на кольце
    return;
  }
  const std::vector<double> & stops = map_->stops();
  const double s2 = sigma2();
  const double S = s2 + cfg_.sigma_stop * cfg_.sigma_stop;
  const double gate = std::min(std::max(cfg_.gate_k * std::sqrt(S), cfg_.gate_min), cfg_.gate_max);
  int best = -1;
  double best_nu = 0.0;
  int in_gate = 0;
  for (std::size_t i = 0; i < stops.size(); ++i) {
    if (static_cast<int>(i) == used_stop_) {
      continue;  // по ней уже сбросились на этой остановке
    }
    const double nu = ring_delta(stops[i], s_);
    if (std::abs(nu) > gate) {
      continue;
    }
    ++in_gate;
    if (best < 0 || std::abs(nu) < std::abs(best_nu)) {
      best = static_cast<int>(i);
      best_nu = nu;
    }
  }
  if (best < 0) {
    ++stats_.no_stop;
    return;
  }
  if (in_gate > 1) {
    ++stats_.ambiguous;
    return;
  }
  const double k = (cfg_.teleport || S <= 0.0) ? 1.0 : s2 / S;
  last_correction_ = k * best_nu;
  s_ = wrap_ring(s_ + last_correction_);
  sigma_ref2_ = cfg_.teleport ? cfg_.sigma_stop * cfg_.sigma_stop : (1.0 - k) * s2;
  dist_ = 0.0;
  used_stop_ = best;
  ++stats_.resets;
}

void StopResetRouteIntegrator::rearm_used_stop()
{
  if (used_stop_ < 0) {
    return;
  }
  const bool on_ring = map_ && map_->has_routes() && route_ == 0 &&
    static_cast<std::size_t>(used_stop_) < map_->stops().size();
  if (!on_ring ||
    std::abs(ring_delta(map_->stops()[static_cast<std::size_t>(used_stop_)], s_)) > cfg_.rearm_m)
  {
    used_stop_ = -1;
  }
}

double StopResetRouteIntegrator::ring_delta(double a, double b) const
{
  const double L = map_ ? map_->ring_length() : 0.0;
  if (L <= 0.0) {
    return a - b;
  }
  double d = std::fmod(a - b, L);
  if (d >= 0.5 * L) {
    d -= L;
  } else if (d < -0.5 * L) {
    d += L;
  }
  return d;
}

}  // namespace tram

PLUGINLIB_EXPORT_CLASS(tram::StopResetRouteIntegrator, tram::IPositionIntegrator)
