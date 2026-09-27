#include "tram_reserve_odometry/plugins/kalman_wheel_estimator.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "tram_reserve_odometry/map/route_map.hpp"

namespace tram
{

namespace
{
constexpr int kFront = 0;
constexpr int kRear = 1;
constexpr int64_t kRecentNs = 500000000;  ///< образец другой тележки «свежий», если моложе 0.5 с
constexpr double kG = 9.81;

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

KalmanWheelEstimator::Config KalmanWheelEstimator::config_from(const ModelParams & p)
{
  Config c;
  c.r = get(p, "kf_r", c.r);
  c.q_jerk = get(p, "kf_q_jerk", c.q_jerk);
  c.tau_a = std::max(get(p, "kf_tau_a", c.tau_a), 1e-3);
  c.sensor_delay = get(p, "kf_sensor_delay", c.sensor_delay);
  c.gate_sigma = get(p, "kf_gate_sigma", c.gate_sigma);
  c.gate_min = get(p, "kf_gate_min", c.gate_min);
  c.zero = get(p, "kf_zero", c.zero);
  c.dead_other = get(p, "kf_dead_other", c.dead_other);
  c.disagree_abs = get(p, "kf_disagree_abs", c.disagree_abs);
  c.disagree_rel = get(p, "kf_disagree_rel", c.disagree_rel);
  c.resync_s = get(p, "kf_resync_s", c.resync_s);
  c.max_horizon = get(p, "kf_max_horizon", c.max_horizon);
  c.a0 = get(p, "kf_a0", c.a0);
  c.robust = get_flag(p, "kf_robust", c.robust);
  c.extrapolate = get_flag(p, "kf_extrapolate", c.extrapolate);
  c.zupt = get_flag(p, "kf_zupt", c.zupt);
  c.v_max = get(p, "kf_v_max", c.v_max);
  c.slip_asym = get_flag(p, "kf_slip_asym", c.slip_asym);
  c.slip_low = get(p, "kf_slip_low", c.slip_low);
  c.slip_high = get(p, "kf_slip_high", c.slip_high);
  c.drive_model = get_flag(p, "kf_drive_model", c.drive_model);
  c.drv_tau = std::max(get(p, "drv_tau", c.drv_tau), 0.0);
  c.drv_T = std::max(get(p, "drv_T", c.drv_T), 1e-3);
  c.drv_w0 = get(p, "drv_w0", c.drv_w0);
  c.drv_c_aero = get(p, "drv_c_aero", c.drv_c_aero);
  c.drv_k_curve = get(p, "drv_k_curve", c.drv_k_curve);
  c.drv_r_min = std::max(get(p, "drv_r_min", c.drv_r_min), 1.0);
  c.drv_kg = get(p, "drv_kg", c.drv_kg);
  c.drv_q_jerk = get(p, "drv_q_jerk", c.drv_q_jerk);
  c.drv_q_theta = get(p, "drv_q_theta", c.drv_q_theta);
  c.drv_sigma_theta0 = get(p, "drv_sigma_theta0", c.drv_sigma_theta0);
  c.drv_q_b = get(p, "drv_q_b", c.drv_q_b);
  c.drv_sigma_b0 = get(p, "drv_sigma_b0", c.drv_sigma_b0);
  c.drv_theta_min = get(p, "drv_theta_min", c.drv_theta_min);
  c.drv_theta_max = get(p, "drv_theta_max", c.drv_theta_max);
  c.drv_v_hold = get(p, "drv_v_hold", c.drv_v_hold);
  c.drv_monitor = get_flag(p, "drv_monitor", c.drv_monitor);
  c.drv_mon_win = get(p, "drv_mon_win", c.drv_mon_win);
  c.drv_mon_on = get(p, "drv_mon_on", c.drv_mon_on);
  c.drv_mon_off = get(p, "drv_mon_off", c.drv_mon_off);
  c.drv_substep = std::max(get(p, "drv_substep", c.drv_substep), 1e-3);
  c.drv_outage_s = get(p, "drv_outage_s", c.drv_outage_s);
  return c;
}

void KalmanWheelEstimator::reset()
{
  init_ = false;
  x_ = {0.0, 0.0, 1.0, 1.0, 0.0};
  P_ = {};
  t_ns_ = last_accept_ns_ = last_meas_ns_ = 0;
  bogie_ = {};
  stats_ = Stats{};
  physics_ = drive_ok_;
  last_cmd_ns_ = 0;
  u_hist_.clear();
  wheel_hist_.clear();
  m_hist_.clear();
  mon_.clear();
  a_shadow_ = 0.0;
  shadow_ns_ = 0;
  grade_term_ = curve_term_ = 0.0;
  last_slip_reject_ns_ = 0;
  creep_ = 0.0;
}

// ------------------------------------------------------------------ модель процесса

double KalmanWheelEstimator::notch_at(int64_t t_ns) const
{
  if (u_hist_.empty()) {
    return 0.0;
  }
  const int64_t td = t_ns - static_cast<int64_t>(cfg_.drv_tau * 1e9);
  double u = u_hist_.front().second;
  for (const auto & e : u_hist_) {
    if (e.first > td) {
      break;
    }
    u = e.second;
  }
  return u;
}

double KalmanWheelEstimator::model_accel(
  const Vec & x, double notch, double & fT, double & fB, bool & hold) const
{
  const double v = std::max(x[kV], 0.0);
  const double f = traction_ ? traction_->drive_force(notch, v, params_) : 0.0;
  fT = std::max(f, 0.0);
  fB = std::min(f, 0.0);
  // Сопротивление движению: основное + аэродинамика + кривые; против движения, на стоянке — 0.
  const double w = cfg_.drv_w0 + cfg_.drv_c_aero * v * v + curve_term_;
  double m = x[kTT] * fT + x[kTB] * fB - w * std::tanh(v / 0.2) - grade_term_ + x[kB];
  hold = false;
  if (v < cfg_.drv_v_hold && m < 0.0) {
    // Стоянка: тормоз и трение покоя держат трамвай, назад он не едет.
    m = 0.0;
    fT = fB = 0.0;
    hold = true;
  }
  return m;
}

void KalmanWheelEstimator::step_kinematic(double dt, Vec & x, Mat & P) const
{
  if (dt <= 0.0) {
    return;
  }
  // Модель Зингера для (v, a); theta и b не меняются, связь с ними разорвана (decouple).
  const double al = std::exp(-dt / cfg_.tau_a);
  const double f01 = cfg_.tau_a * (1.0 - al);
  x[kV] += f01 * x[kA];
  x[kA] *= al;
  const double p00 = P[kV][kV], p01 = P[kV][kA], p11 = P[kA][kA];
  const double n00 = p00 + 2.0 * f01 * p01 + f01 * f01 * p11;
  const double n01 = al * (p01 + f01 * p11);
  const double n11 = al * al * p11;
  const double q = cfg_.q_jerk;
  P[kV][kV] = n00 + q * dt * dt * dt / 3.0;
  P[kV][kA] = P[kA][kV] = n01 + q * dt * dt / 2.0;
  P[kA][kA] = n11 + q * dt;
}

void KalmanWheelEstimator::step_physics(double dt, double notch, Vec & x, Mat & P) const
{
  if (dt <= 0.0) {
    return;
  }
  double fT = 0.0, fB = 0.0;
  bool hold = false;
  const double m = model_accel(x, notch, fT, fB, hold);
  const double T = cfg_.drv_T;
  const double al = std::exp(-dt / T);
  const double c0 = T * (1.0 - al);
  const double c1 = 1.0 - al;
  const double c2 = dt - c0;
  const double db = hold ? 0.0 : 1.0;
  // Точное решение da/dt = (m - a)/T, dv/dt = a при постоянном m.
  x[kV] += c0 * x[kA] + c2 * m;
  x[kA] = al * x[kA] + c1 * m;
  if (x[kV] < 0.0) {
    x[kV] = 0.0;
    x[kA] = std::max(x[kA], 0.0);
  }
  // Якобиан: m линейно по theta_T, theta_B, b (f — известный вход).
  Mat F{};
  for (int i = 0; i < N; ++i) {
    F[i][i] = 1.0;
  }
  F[kV][kA] = c0;
  F[kV][kTT] = c2 * fT;
  F[kV][kTB] = c2 * fB;
  F[kV][kB] = c2 * db;
  F[kA][kA] = al;
  F[kA][kTT] = c1 * fT;
  F[kA][kTB] = c1 * fB;
  F[kA][kB] = c1 * db;
  Mat FP{};
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double s = 0.0;
      for (int k = 0; k < N; ++k) {
        s += F[i][k] * P[k][j];
      }
      FP[i][j] = s;
    }
  }
  for (int i = 0; i < N; ++i) {
    for (int j = i; j < N; ++j) {
      double s = 0.0;
      for (int k = 0; k < N; ++k) {
        s += FP[i][k] * F[j][k];
      }
      P[i][j] = P[j][i] = s;
    }
  }
  const double q = cfg_.drv_q_jerk;
  P[kV][kV] += q * dt * dt * dt / 3.0;
  P[kV][kA] += q * dt * dt / 2.0;
  P[kA][kV] = P[kV][kA];
  P[kA][kA] += q * dt;
  P[kTT][kTT] += cfg_.drv_q_theta * cfg_.drv_q_theta * dt;
  P[kTB][kTB] += cfg_.drv_q_theta * cfg_.drv_q_theta * dt;
  P[kB][kB] += cfg_.drv_q_b * cfg_.drv_q_b * dt;
}

void KalmanWheelEstimator::predict(int64_t t0_ns, int64_t t1_ns, Vec & x, Mat & P) const
{
  if (t1_ns <= t0_ns) {
    return;
  }
  if (!(drive_ok_ && physics_)) {
    step_kinematic(static_cast<double>(t1_ns - t0_ns) * 1e-9, x, P);
    return;
  }
  const int64_t sub = static_cast<int64_t>(cfg_.drv_substep * 1e9);
  for (int64_t t = t0_ns; t < t1_ns; ) {
    const int64_t t_next = std::min(t + sub, t1_ns);
    step_physics(static_cast<double>(t_next - t) * 1e-9, notch_at(t), x, P);
    t = t_next;
  }
}

// ------------------------------------------------------------------ измерения

void KalmanWheelEstimator::update(double z)
{
  const double s = P_[kV][kV] + cfg_.r * cfg_.r;
  const double nu = z - x_[kV];
  std::array<double, N> K{};
  for (int i = 0; i < N; ++i) {
    K[i] = P_[i][kV] / s;
  }
  for (int i = 0; i < N; ++i) {
    x_[i] += K[i] * nu;
  }
  const std::array<double, N> row0 = P_[kV];
  const double p00 = P_[kV][kV], p01 = P_[kV][kA], p11 = P_[kA][kA];
  for (int i = 0; i < N; ++i) {
    for (int j = i; j < N; ++j) {
      P_[i][j] = P_[j][i] = P_[i][j] - K[i] * row0[j];
    }
  }
  P_[kV][kV] = (1.0 - K[kV]) * p00;
  P_[kV][kA] = P_[kA][kV] = (1.0 - K[kV]) * p01;
  P_[kA][kA] = p11 - K[kA] * p01;
  if (drive_ok_) {
    x_[kTT] = std::clamp(x_[kTT], cfg_.drv_theta_min, cfg_.drv_theta_max);
    x_[kTB] = std::clamp(x_[kTB], cfg_.drv_theta_min, cfg_.drv_theta_max);
  }
  ++stats_.accepted;
}

void KalmanWheelEstimator::decouple()
{
  for (int i : {kV, kA}) {
    for (int j : {kTT, kTB, kB}) {
      P_[i][j] = P_[j][i] = 0.0;
    }
  }
}

void KalmanWheelEstimator::reset_va(double v, double pvv, double paa)
{
  x_[kV] = v;
  x_[kA] = 0.0;
  P_[kV][kV] = pvv;
  P_[kV][kA] = P_[kA][kV] = 0.0;
  P_[kA][kA] = paa;
  decouple();
}

bool KalmanWheelEstimator::is_dead(int ch, double z, int64_t t_ns) const
{
  const Bogie & other = bogie_[1 - ch];
  const bool other_recent = other.has && std::llabs(other.stamp_ns - t_ns) < kRecentNs;
  return z < cfg_.zero && other_recent && other.value > cfg_.dead_other;
}

void KalmanWheelEstimator::monitor(int64_t t_ns, double z_mean)
{
  if (!drive_ok_) {
    return;
  }
  // a_мод, пропущенное через инерцию привода T, — для сравнения с ускорением по тележкам.
  double fT = 0.0, fB = 0.0;
  bool hold = false;
  const double m = model_accel(x_, notch_at(t_ns), fT, fB, hold);
  const double dt = (shadow_ns_ > 0) ?
    std::clamp(static_cast<double>(t_ns - shadow_ns_) * 1e-9, 0.0, 1.0) : 0.0;
  a_shadow_ += (m - a_shadow_) * (1.0 - std::exp(-dt / cfg_.drv_T));
  shadow_ns_ = t_ns;
  wheel_hist_.emplace_back(t_ns, z_mean);
  m_hist_.emplace_back(t_ns, a_shadow_);
  const int64_t win = 600000000;
  while (!wheel_hist_.empty() && wheel_hist_.front().first < t_ns - win) {
    wheel_hist_.pop_front();
  }
  while (!m_hist_.empty() && m_hist_.front().first < t_ns - win) {
    m_hist_.pop_front();
  }
  if (wheel_hist_.size() < 4 || x_[kV] < 0.5 || !cfg_.drv_monitor) {
    return;
  }
  const double span = static_cast<double>(t_ns - wheel_hist_.front().first) * 1e-9;
  if (span < 0.35) {
    return;
  }
  // Ускорение по тележкам — наклон МНК за окно; сравнивается со средним a_мод там же.
  double st = 0.0, sz = 0.0;
  for (const auto & e : wheel_hist_) {
    st += static_cast<double>(e.first - t_ns) * 1e-9;
    sz += e.second;
  }
  const double n = static_cast<double>(wheel_hist_.size());
  st /= n;
  sz /= n;
  double stt = 0.0, stz = 0.0;
  for (const auto & e : wheel_hist_) {
    const double d = static_cast<double>(e.first - t_ns) * 1e-9 - st;
    stt += d * d;
    stz += d * (e.second - sz);
  }
  const double a_wheel = (stt > 1e-9) ? stz / stt : 0.0;
  double m_avg = 0.0;
  for (const auto & e : m_hist_) {
    m_avg += e.second;
  }
  m_avg /= static_cast<double>(m_hist_.size());
  mon_.emplace_back(t_ns, a_wheel - m_avg);
  while (!mon_.empty() && mon_.front().first < t_ns - static_cast<int64_t>(cfg_.drv_mon_win * 1e9)) {
    mon_.pop_front();
  }
  if (mon_.size() < 15) {
    return;
  }
  double mean = 0.0, s2 = 0.0;
  for (const auto & q : mon_) {
    mean += q.second;
    s2 += q.second * q.second;
  }
  const double cnt = static_cast<double>(mon_.size());
  mean /= cnt;
  const double rms = std::sqrt(s2 / cnt);
  const double rms_centered = std::sqrt(std::max(s2 / cnt - mean * mean, 0.0));
  if (physics_ && rms > cfg_.drv_mon_on) {
    physics_ = false;      // ручка не объясняет движение (автоведение, ошибка модели)
    decouple();
    ++stats_.model_off;
  } else if (!physics_ && rms_centered < cfg_.drv_mon_off) {
    // Ручка снова объясняет изменения ускорения; постоянное смещение — в возмущение b.
    physics_ = true;
    x_[kB] = std::clamp(x_[kB] + mean, -0.5, 0.5);
    P_[kB][kB] = std::max(P_[kB][kB], cfg_.drv_sigma_b0 * cfg_.drv_sigma_b0);
    decouple();
    mon_.clear();
    ++stats_.model_on;
  }
}

void KalmanWheelEstimator::process_group(
  int64_t t_ns, const double * z, const int * ch, int n)
{
  if (!init_) {
    double sum = 0.0;
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
      if (!is_dead(ch[i], z[i], t_ns)) {
        sum += z[i];
        ++cnt;
      }
    }
    if (cnt == 0) {
      return;
    }
    x_ = {sum / cnt, 0.0, 1.0, 1.0, 0.0};
    P_ = {};
    P_[kV][kV] = cfg_.r * cfg_.r;
    P_[kA][kA] = cfg_.a0 * cfg_.a0;
    if (drive_ok_) {
      P_[kTT][kTT] = P_[kTB][kTB] = cfg_.drv_sigma_theta0 * cfg_.drv_sigma_theta0;
      P_[kB][kB] = cfg_.drv_sigma_b0 * cfg_.drv_sigma_b0;
    }
    t_ns_ = last_accept_ns_ = t_ns;
    init_ = true;
    return;
  }
  if (t_ns < t_ns_) {
    ++stats_.out_of_order;  // старше состояния — пропускаем
    return;
  }
  predict(t_ns_, t_ns, x_, P_);
  t_ns_ = t_ns;

  bool accept[2] = {true, true};
  bool dead[2] = {false, false};
  for (int i = 0; i < n; ++i) {
    dead[i] = is_dead(ch[i], z[i], t_ns);
  }

  // 5. ZUPT: все исправные тележки стоят.
  if (cfg_.zupt) {
    bool all_zero = true;
    int alive = 0;
    for (int c = 0; c < 2; ++c) {
      const Bogie & b = bogie_[c];
      if (!b.has || std::llabs(b.stamp_ns - t_ns) >= kRecentNs || is_dead(c, b.value, t_ns)) {
        continue;
      }
      ++alive;
      all_zero = all_zero && b.value <= cfg_.zero;
    }
    if (alive > 0 && all_zero) {
      reset_va(0.0, 1e-6, 1e-4);
      last_accept_ns_ = t_ns;
      ++stats_.zupts;
      return;
    }
  }

  if (cfg_.robust) {
    // 1. «мёртвая» тележка.
    for (int i = 0; i < n; ++i) {
      if (dead[i]) {
        accept[i] = false;
        ++stats_.rejected_dead;
      }
    }
    // 2. тележки расходятся: верим той, что ближе к прогнозу.
    if (n == 2 && accept[0] && accept[1]) {
      const double tol = cfg_.disagree_abs + cfg_.disagree_rel * std::max(x_[kV], 0.0);
      if (std::abs(z[0] - z[1]) > tol) {
        int bad = (std::abs(z[0] - x_[kV]) > std::abs(z[1] - x_[kV])) ? 0 : 1;
        const double u = (cfg_.slip_asym && drive_ok_) ? notch_at(t_ns) : 0.0;
        if (u > 0.5) {
          bad = (z[0] > z[1]) ? 0 : 1;  // тяга: боксующая тележка вращается быстрее кузова
          ++stats_.slip_traction;
        } else if (u < -0.5) {
          bad = (z[0] < z[1]) ? 0 : 1;  // торможение: тележка в юзе — медленнее кузова
          ++stats_.slip_braking;
        }
        accept[bad] = false;
        last_slip_reject_ns_ = t_ns;
        ++stats_.rejected_disagree;
      }
    }
    // 3. гейт по невязке.
    const double s = P_[kV][kV] + cfg_.r * cfg_.r;
    const double gate = std::max(cfg_.gate_sigma * std::sqrt(s), cfg_.gate_min);
    for (int i = 0; i < n; ++i) {
      if (accept[i] && std::abs(z[i] - x_[kV]) > gate) {
        accept[i] = false;
        last_slip_reject_ns_ = t_ns;
        ++stats_.rejected_gate;
      }
    }
  }
  // Относительное скольжение тележек относительно прогноза кузова (для статуса).
  creep_ = 0.0;
  for (int i = 0; i < n; ++i) {
    if (!dead[i]) {
      const double c = (z[i] - x_[kV]) / std::max(x_[kV], 1.0);
      creep_ = (std::abs(c) > std::abs(creep_)) ? c : creep_;
    }
  }

  bool any = false;
  double z_sum = 0.0;
  int z_cnt = 0;
  for (int i = 0; i < n; ++i) {
    if (accept[i]) {
      update(z[i]);
      any = true;
      z_sum += z[i];
      ++z_cnt;
    }
  }
  if (any) {
    last_accept_ns_ = t_ns;
    monitor(t_ns, z_sum / z_cnt);
    return;
  }
  // 4. измерения есть, но фильтр их долго не принимает — ресинхронизация.
  if (static_cast<double>(t_ns - last_accept_ns_) * 1e-9 > cfg_.resync_s) {
    double sum = 0.0;
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
      if (!dead[i]) {
        sum += z[i];
        ++cnt;
      }
    }
    if (cnt > 0) {
      reset_va(sum / cnt, cfg_.r * cfg_.r, cfg_.a0 * cfg_.a0);
      last_accept_ns_ = t_ns;
      ++stats_.resyncs;
    }
  }
}

Estimate KalmanWheelEstimator::estimate(
  const PreprocessedSample & sample, double, const ISlipDetector::Result &,
  const ModelParams & params)
{
  if (!cfg_loaded_) {
    cfg_ = config_from(params);
    params_ = params;
    drive_ok_ = cfg_.drive_model && traction_ && traction_->has_drive_model(params_);
    physics_ = drive_ok_;
    cfg_loaded_ = true;
  }
  const int64_t delay_ns = static_cast<int64_t>(cfg_.sensor_delay * 1e9);

  if (drive_ok_) {
    // История ручки (для запаздывания привода) и уклон в текущей точке карты.
    if (sample.driver_stamp_ns > last_cmd_ns_ && params_.driver_scale > 0.0) {
      last_cmd_ns_ = sample.driver_stamp_ns;
      u_hist_.emplace_back(
        sample.driver_stamp_ns, std::round(sample.driver_position_cmd / params_.driver_scale));
      const int64_t keep = sample.driver_stamp_ns - 3000000000LL;
      while (u_hist_.size() > 2 && u_hist_[1].first < keep) {
        u_hist_.pop_front();
      }
    }
    grade_term_ = 0.0;
    if (map_ && ctx_.on_route) {
      Point3D p;
      double hd = 0.0, grade = 0.0;
      double curv = 0.0;
      curve_term_ = 0.0;
      if (map_->lookup(ctx_.route_id, ctx_.s, p, hd, &grade, &curv)) {
        grade_term_ = cfg_.drv_kg * kG * grade * static_cast<double>(ctx_.direction);
        curve_term_ = cfg_.drv_k_curve * std::min(std::abs(curv), 1.0 / cfg_.drv_r_min);
      }
    }
  }

  // Новые образцы тележек; с общей меткой — одна группа измерений.
  const bool valid[2] = {sample.front_valid, sample.rear_valid};
  const int64_t stamp[2] = {sample.front_stamp_ns, sample.rear_stamp_ns};
  const double value[2] = {sample.front_bogie_velocity, sample.rear_bogie_velocity};
  bool fresh[2] = {false, false};
  for (int c = kFront; c <= kRear; ++c) {
    if (!valid[c] || stamp[c] <= bogie_[c].stamp_ns) {
      continue;
    }
    const double z = value[c];
    bogie_[c].stamp_ns = stamp[c];
    last_meas_ns_ = std::max(last_meas_ns_, stamp[c] - delay_ns);
    if (!std::isfinite(z) || z < -0.5 || z > cfg_.v_max) {
      continue;  // не скорость — образец отбрасываем
    }
    bogie_[c].value = std::max(z, 0.0);
    bogie_[c].has = true;
    fresh[c] = true;
  }
  if (fresh[kFront] && fresh[kRear] && stamp[kFront] == stamp[kRear]) {
    const double z[2] = {bogie_[kFront].value, bogie_[kRear].value};
    const int ch[2] = {kFront, kRear};
    process_group(stamp[kFront] - delay_ns, z, ch, 2);
  } else {
    // Разные метки — по одной, в порядке времени.
    int order[2] = {kFront, kRear};
    if (stamp[kRear] < stamp[kFront]) {
      std::swap(order[0], order[1]);
    }
    for (int c : order) {
      if (fresh[c]) {
        const double z = bogie_[c].value;
        process_group(stamp[c] - delay_ns, &z, &c, 1);
      }
    }
  }

  // Тележки молчат дольше drv_outage_s, модель валидна: счисление по ручке. Состояние —
  // на kKeepBackNs позади метки ручки, чтобы вернувшиеся образцы тележек были новее него.
  if (init_ && drive_ok_ && physics_) {
    constexpr int64_t kKeepBackNs = 150000000;
    const int64_t outage = static_cast<int64_t>(cfg_.drv_outage_s * 1e9);
    const int64_t target = sample.stamp_ns - kKeepBackNs;
    if (target > t_ns_ && sample.stamp_ns - last_meas_ns_ > outage) {
      predict(t_ns_, target, x_, P_);
      t_ns_ = target;
    }
  }

  Estimate e;
  e.stamp_ns = sample.stamp_ns;
  e.quality = sample.quality;
  e.initialized = true;
  if (!init_) {
    return e;
  }
  // Прогноз к метке публикации (копия состояния, само состояние не меняется).
  Vec x = x_;
  Mat P = P_;
  if (cfg_.extrapolate) {
    const double h = std::clamp(
      static_cast<double>(sample.stamp_ns - t_ns_) * 1e-9, 0.0, cfg_.max_horizon);
    predict(t_ns_, t_ns_ + static_cast<int64_t>(h * 1e9), x, P);
  }
  e.velocity = std::max(x[kV], 0.0);
  e.acceleration = x[kA];
  e.velocity_cov = P[kV][kV];
  e.model_mode = drive_ok_ ? (physics_ ? 1 : 0) : -1;
  // Статус проскальзывания: отбраковка тележки за последнюю секунду или скольжение.
  const double creep = std::abs(creep_);
  if (last_slip_reject_ns_ > 0 && sample.stamp_ns - last_slip_reject_ns_ < 1000000000LL) {
    e.slip = SlipStatus::HIGH;
  } else if (creep > cfg_.slip_high) {
    e.slip = SlipStatus::MEDIUM;
  } else if (creep > cfg_.slip_low && x[kV] > 1.0) {
    e.slip = SlipStatus::LOW;
  }
  e.slip_confidence = (e.slip == SlipStatus::HIGH) ? 1.0 :
    std::clamp(creep / std::max(cfg_.slip_high, 1e-6), 0.0, 1.0);
  return e;
}

}  // namespace tram

PLUGINLIB_EXPORT_CLASS(tram::KalmanWheelEstimator, tram::IStateEstimator)
