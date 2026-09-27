// Фильтр Калмана скорости трамвая: модель продольной динамики с приводом по ручке
// контроллера + робастная модель измерений двух тележек.
//
// Состояние x = [v, a, theta_T, theta_B, b]: скорость и ускорение кузова, множители тяги
// и торможения (масса, загрузка, износ — адаптация модели), медленное возмущение.
//
// Режим «модель» (kf_drive_model, характеристика привода есть, модель объясняет движение):
//   u_d(t)  = u(t - tau)                                       запаздывание привода
//   a_мод   = theta_T*f+(u_d, v) + theta_B*f-(u_d, v) - w(v, s)*tanh(v/0.2) - k_g*g*i(s)*dir + b
//   w(v, s) = w0 + c_aero*v^2 + k_curve*min(|kappa(s)|, 1/R_min)   основное + аэродинамика + кривые
//   da/dt   = (a_мод - a) / T + w_j,   dv/dt = a                 инерция привода T
// f(u, v) — характеристика привода (ITractionModel::drive_force, f+ — тяга, f- — торможение),
// i(s) и kappa(s) — уклон и кривизна карты в точке трамвая.
// Коэффициенты c_aero, k_curve и границы theta выведены из паспорта вагона 71-911ЕМ
// (docs/physical_model.md), theta = m_eq_ref/m_eq. На стоянке (v < v_hold) торможение
// не тянет назад, масса и возмущение там не обучаются. Точное решение на шаге dt при
// постоянном a_мод; прогноз — подшагами drv_substep (ручка меняется).
// Режим «кинематика» (модель выключена, её нет или она не объясняет движение):
//   da/dt = -a / tau_a + w_j (модель Зингера), theta и b заморожены.
// Монитор валидности: ускорение по тележкам (МНК за 0.5 с) против a_мод, пропущенного через
// ту же инерцию привода T. RMS невязки за окно > drv_mon_on -> кинематика (автоведение:
// ручка не отражает тягу). Обратно — когда RMS невязки без её среднего < drv_mon_off:
// ручка снова объясняет изменения ускорения, а постоянное смещение уходит в возмущение b.
//
// Модель измерений: z_i = v(t_i) + n_i, i — передняя / задняя тележка, t_i — метка образца
// тележки минус задержка датчика, n_i ~ N(0, r^2).
//
// Робастность — из того, что обе тележки стоят под одним жёстким кузовом:
//   1. «мёртвая» тележка: показывает ~0, когда другая едет быстрее dead_other;
//   2. тележки расходятся сильнее disagree_abs + disagree_rel * v — отбрасывается та,
//      что дальше от прогноза; с slip_asym — по физике сцепления: в тяге (ручка > 0)
//      боксует опережающая тележка, в торможении (ручка < 0) идёт юзом отстающая;
//   3. гейт по невязке |nu| > max(gate_sigma * sqrt(S), gate_min): за шаг кузов так
//      разогнаться не может;
//   4. нет принятых измерений дольше resync_s, а тележки есть — ресинхронизация
//      по ним (защита от «запирания» фильтра);
//   5. ZUPT: все исправные тележки показывают 0 — v = 0, a = 0.
// Статус проскальзывания (Estimate::slip) — по относительному скольжению тележек
// (z_i − v)/max(v, 1 м/с) и по отбраковке: HIGH — тележка отброшена правилами 2–3 за
// последнюю секунду, MEDIUM — скольжение > slip_high, LOW — > slip_low.
// Выход — прогноз модели на метку публикации (она свежее образца тележек): это
// убирает запаздывание выхода. Без тележек в режиме модели состояние продолжает
// двигаться по модели (счисление по ручке).
//
// Параметры (секция estimator: конфига, ключи без префикса) — см. Config.

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__KALMAN_WHEEL_ESTIMATOR_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__KALMAN_WHEEL_ESTIMATOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include "tram_reserve_odometry/core/interfaces.hpp"

namespace tram
{

class KalmanWheelEstimator : public IStateEstimator
{
public:
  struct Config
  {
    double r = 0.03;              ///< шум тележки, м/с
    double q_jerk = 1.0;          ///< плотность шума рывка (кинематика), м²/с⁵
    double tau_a = 1.5;           ///< время корреляции ускорения (кинематика), с
    double sensor_delay = 0.0;    ///< задержка датчика тележки, с
    double gate_sigma = 5.0;      ///< гейт невязки в сигмах
    double gate_min = 0.5;        ///< гейт невязки не меньше, м/с
    double zero = 0.05;           ///< «ноль» тележки, м/с
    double dead_other = 1.0;      ///< другая тележка быстрее -> ноль = отказ, м/с
    double disagree_abs = 0.5;    ///< допуск расхождения тележек, м/с
    double disagree_rel = 0.05;   ///< ... плюс доля скорости
    double resync_s = 1.0;        ///< без принятых измерений -> ресинхронизация, с
    double max_horizon = 0.3;     ///< предел прогноза к метке публикации, с
    double a0 = 0.5;              ///< начальная неопределённость ускорения, м/с²
    bool robust = true;           ///< правила 1–4
    bool extrapolate = true;      ///< прогноз к метке публикации
    bool zupt = true;             ///< правило 5
    double v_max = 30.0;          ///< выше — не скорость трамвая (образец отбрасывается), м/с
    bool slip_asym = false;       ///< правило 2 по режиму: тяга — боксование, торможение — юз
    double slip_low = 0.02;       ///< относительное скольжение для статуса LOW
    double slip_high = 0.05;      ///< ... для статуса MEDIUM

    // Модель привода и продольной динамики.
    bool drive_model = false;     ///< режим «модель» (нужна характеристика привода)
    double drv_tau = 0.225;       ///< запаздывание привода, с
    double drv_T = 0.1375;        ///< инерция привода (постоянная времени), с
    double drv_w0 = 0.046;        ///< основное сопротивление движению, м/с²
    double drv_c_aero = 0.0;      ///< аэродинамика c = rho*c_x*A/(2*m_eq), 1/м
    double drv_k_curve = 0.0;     ///< сопротивление в кривых w = k/R, м²/с²
    double drv_r_min = 18.0;      ///< радиус кривой не меньше (шум кривизны карты), м
    double drv_kg = 0.779;        ///< коэффициент уклона (вращающиеся массы, шум карты)
    double drv_q_jerk = 1.0;      ///< шум рывка в режиме модели (ошибка модели), м²/с⁵
    double drv_q_theta = 1e-3;    ///< шум множителей тяги/торможения, 1/√с
    double drv_sigma_theta0 = 0.15;  ///< начальная неопределённость множителей
    double drv_q_b = 0.01;        ///< шум возмущения, м/с²/√с
    double drv_sigma_b0 = 0.1;    ///< начальная неопределённость возмущения, м/с²
    double drv_theta_min = 0.6;   ///< границы множителей
    double drv_theta_max = 1.6;
    double drv_v_hold = 0.1;      ///< ниже — стоянка: торможение не тянет назад, м/с
    bool drv_monitor = true;      ///< монитор валидности модели
    double drv_mon_win = 3.0;     ///< окно монитора валидности модели, с
    double drv_mon_on = 0.35;     ///< RMS невязки ускорения выше — модель не объясняет движение, м/с²
    double drv_mon_off = 0.2;     ///< ниже — снова объясняет, м/с²
    double drv_substep = 0.02;    ///< шаг интегрирования прогноза, с
    double drv_outage_s = 0.5;    ///< без тележек дольше — счисление по модели, с
  };

  /// Счётчики для диагностики.
  struct Stats
  {
    std::size_t accepted = 0;
    std::size_t rejected_dead = 0;
    std::size_t rejected_disagree = 0;
    std::size_t rejected_gate = 0;
    std::size_t resyncs = 0;
    std::size_t zupts = 0;
    std::size_t out_of_order = 0;
    std::size_t model_off = 0;    ///< переключений «модель -> кинематика»
    std::size_t model_on = 0;     ///< переключений «кинематика -> модель»
    std::size_t slip_traction = 0;  ///< правило 2 по режиму: отброшена опережающая (боксование)
    std::size_t slip_braking = 0;   ///< ... отстающая (юз)
  };

  KalmanWheelEstimator() = default;

  Estimate estimate(
    const PreprocessedSample & sample,
    double model_acceleration,
    const ISlipDetector::Result & slip,
    const ModelParams & params) override;

  std::string name() const override {return "KalmanWheelEstimator";}
  void reset() override;

  void set_traction_model(const std::shared_ptr<const ITractionModel> & m) override
  {
    traction_ = m;
  }
  void set_route_map(const std::shared_ptr<const RouteMap> & m) override {map_ = m;}
  void set_route_context(const Estimate & e) override {ctx_ = e;}

  static Config config_from(const ModelParams & params);
  const Stats & stats() const {return stats_;}
  /// Режим «модель» доступен (включён и есть характеристика привода).
  bool drive_available() const {return drive_ok_;}
  /// Сейчас прогноз по модели (а не по кинематике).
  bool model_active() const {return drive_ok_ && physics_;}
  double theta_traction() const {return x_[kTT];}
  double theta_braking() const {return x_[kTB];}
  double disturbance() const {return x_[kB];}

private:
  static constexpr int N = 5;
  using Vec = std::array<double, N>;
  using Mat = std::array<std::array<double, N>, N>;
  enum : int {kV = 0, kA = 1, kTT = 2, kTB = 3, kB = 4};

  struct Bogie
  {
    int64_t stamp_ns = 0;   ///< метка последнего образца
    double value = 0.0;     ///< последнее значение, м/с
    bool has = false;
  };

  /// Прогноз (x, P) от t0 до t1 в текущем режиме.
  void predict(int64_t t0_ns, int64_t t1_ns, Vec & x, Mat & P) const;
  void step_kinematic(double dt, Vec & x, Mat & P) const;
  void step_physics(double dt, double notch, Vec & x, Mat & P) const;
  /// a_мод; fT/fB — эффективные доли тяги/торможения для якобиана, hold — стоянка.
  double model_accel(const Vec & x, double notch, double & fT, double & fB, bool & hold) const;
  void update(double z);
  /// Позиция ручки на момент t с учётом запаздывания привода.
  double notch_at(int64_t t_ns) const;
  /// Разорвать связь (v, a) с (theta, b): переключение режима, стоянка, ресинхронизация.
  void decouple();
  void reset_va(double v, double pvv, double paa);
  void process_group(int64_t t_ns, const double * z, const int * ch, int n);
  bool is_dead(int ch, double z, int64_t t_ns) const;
  void monitor(int64_t t_ns, double z_mean);

  Config cfg_;
  ModelParams params_;          ///< копия параметров (нужна характеристике привода)
  bool cfg_loaded_ = false;
  bool init_ = false;
  Vec x_{0.0, 0.0, 1.0, 1.0, 0.0};
  Mat P_{};
  int64_t t_ns_ = 0;            ///< время состояния
  int64_t last_accept_ns_ = 0;  ///< последнее принятое измерение
  int64_t last_meas_ns_ = 0;    ///< последнее пришедшее измерение тележек (любое)
  std::array<Bogie, 2> bogie_{};
  Stats stats_;

  // Модель привода.
  std::shared_ptr<const ITractionModel> traction_;
  std::shared_ptr<const RouteMap> map_;
  Estimate ctx_;
  bool drive_ok_ = false;
  bool physics_ = false;
  double grade_term_ = 0.0;     ///< k_g * g * i(s) * dir на текущем шаге, м/с²
  double curve_term_ = 0.0;     ///< k_curve * |kappa(s)| на текущем шаге, м/с²
  int64_t last_slip_reject_ns_ = 0;  ///< последняя отбраковка тележки правилами 2–3
  double creep_ = 0.0;          ///< относительное скольжение тележек на последнем шаге
  int64_t last_cmd_ns_ = 0;
  std::deque<std::pair<int64_t, double>> u_hist_;      ///< (метка, позиция ручки)
  std::deque<std::pair<int64_t, double>> wheel_hist_;  ///< принятые измерения для a по тележкам
  std::deque<std::pair<int64_t, double>> m_hist_;      ///< a_мод в моменты измерений
  std::deque<std::pair<int64_t, double>> mon_;         ///< невязки ускорения
  double a_shadow_ = 0.0;       ///< a_мод, пропущенное через инерцию привода (для монитора)
  int64_t shadow_ns_ = 0;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__KALMAN_WHEEL_ESTIMATOR_HPP_
