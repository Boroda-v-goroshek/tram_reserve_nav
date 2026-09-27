// ROS-нода резервной одометрии: входной мост, плагины пайплайна (по имени из конфига),
// выставка и коррекция положения по GNSS на 1D-карте, перевод в выходную систему,
// выходной мост; цикл — таймер output_rate_hz.

#ifndef TRAM_RESERVE_ODOMETRY__NODE__RESERVE_ODOMETRY_NODE_HPP_
#define TRAM_RESERVE_ODOMETRY__NODE__RESERVE_ODOMETRY_NODE_HPP_

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "tram_reserve_odometry/core/geo.hpp"
#include "tram_reserve_odometry/core/interfaces.hpp"
#include "tram_reserve_odometry/core/types.hpp"
#include "tram_reserve_odometry/estimation/estimation_pipeline.hpp"
#include "tram_reserve_odometry/io/input_bridge.hpp"
#include "tram_reserve_odometry/io/output_bridge.hpp"
#include "tram_reserve_odometry/map/route_map.hpp"

namespace tram
{

class ReserveOdometryNode : public rclcpp::Node
{
public:
  explicit ReserveOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ReserveOdometryNode() override;

  /// Один цикл: предобработка -> оценка -> публикация.
  void on_timer();

  // --- Офлайн-прогон (reserve_odometry_replay).

  /// Сообщение входного топика в обход подписки. Топик сверяется с input.*_topic;
  /// \return false, если топик не входной.
  bool ingest_input(
    const std::string & topic, const tram_vehicle_msgs::msg::VelocitySensor & msg,
    int64_t recv_ns);
  bool ingest_input(
    const std::string & topic, const DriverCommand & msg, int64_t recv_ns);
  /// GNSS-фикс в обход подписки; учитывается, только пока идёт выставка.
  void ingest_gnss_fix(const std::string & topic, const sensor_msgs::msg::NavSatFix & msg);
  /// Вызывается в on_timer() с тем, что публикуется в /result/* (после перевода в output.frame).
  using OutputHook = std::function<void(const Estimate & published, bool position_known)>;
  void set_output_hook(OutputHook hook) {output_hook_ = std::move(hook);}
  /// Источник «сейчас» для метки публикации и задержки (по умолчанию — steady clock).
  void set_now_source(std::function<int64_t()> now) {now_fn_ = std::move(now);}
  const std::string & front_topic() const {return front_topic_;}
  const std::string & rear_topic() const {return rear_topic_;}
  const std::string & driver_topic() const {return driver_topic_;}
  double output_rate_hz() const {return output_rate_hz_;}
  /// false, если плагины не загрузились (нода не работает).
  bool ready() const {return static_cast<bool>(output_bridge_);}

private:
  struct GnssFix
  {
    int64_t stamp_ns = 0;
    double lat = 0.0, lon = 0.0, alt = 0.0;
    int status = 0;   ///< NavSatStatus: 0 — автономный фикс, 2 — RTK/GBAS
  };

  bool load_plugins();
  void declare_and_read_params();
  /// Загрузить 1D-карту (если map.enabled) и подписаться на 2 GNSS-антенны.
  void setup_map();
  /// Выставка по GNSS: компас (s0 по позиции + курс по базе антенн) или по смещению.
  /// \param input_stamp_ns текущее время по входам (время bag) — для таймаута.
  void try_initialize_map(int64_t input_stamp_ns);
  /// Проекция base_link на карту для выставки: с учётом курса (map.init_forward_only),
  /// иначе — ближайшая точка любых путей.
  bool project_for_init(
    double x, double y, double heading, std::size_t & rid, double & s0, double & ct) const;
  /// Привязать интегратор к карте.
  /// \param cross_track поперечное отклонение base_link от пути при выставке, м.
  void bind_map(std::size_t route_id, double s0, int dir, double cross_track);
  /// Завершить выставку без карты (относительный режим).
  void finish_map_init_unbound();
  /// Коррекция s по фиксу антенны ("master"/"rover") после выставки:
  /// антенна -> проекция на путь -> base_link -> s. С gnss_corr_require_pair — по паре фиксов.
  void correct_by_gnss(const std::string & antenna, const sensor_msgs::msg::NavSatFix & msg);
  /// Поправка s base_link по фиксу антенны относительно текущей оценки, м.
  /// \return false — фикс не ложится на путь.
  bool gnss_delta(
    const std::string & antenna, const GnssFix & fix, double & delta, double & ct,
    bool * off_route = nullptr) const;
  /// Перепривязка к другому пути по серии фиксов антенны idx: курс по смещению фиксов ->
  /// путь, согласный с курсом, -> route_id, s, направление.
  void try_relocate(int idx);
  /// Фикс антенны позиции (master).
  void on_gnss_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  /// Фикс антенны ориентации (rover).
  void on_ori_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  static GnssFix make_fix(const sensor_msgs::msg::NavSatFix & msg);
  /// Продольное смещение антенны ("master"/"rover") в base_link, м.
  double antenna_x(const std::string & name) const;
  /// Скорость для /result/velocity с задержкой output.velocity_delay_s (история оценок).
  double delayed_velocity(int64_t stamp_ns, double velocity);
  /// Внутренняя оценка (ENU, base_link) -> публикуемая (output.point, output.frame).
  Estimate to_output(const Estimate & est);

  // Параметры.
  double output_rate_hz_ = 20.0;
  /// Насколько метка публикации может уйти вперёд от самого свежего входа, с.
  double max_lead_s_ = 0.3;
  double velocity_delay_s_ = 0.0;      ///< задержка публикуемой скорости, с
  std::deque<std::pair<int64_t, double>> vel_hist_;  ///< (метка, скорость) для задержки
  int64_t last_fed_stamp_ns_ = 0;      ///< последняя метка, поданная в конвейер
  std::function<int64_t()> now_fn_;
  std::string front_topic_ = "/vehicle/front_bogie_velocity";
  std::string rear_topic_ = "/vehicle/rear_bogie_velocity";
  std::string driver_topic_ = "/vehicle/driver_position_cmd";
  std::string bogie_unit_ = "kmh";   ///< kmh | ms
  double driver_cmd_max_ = 15.0;     ///< крайняя позиция ручки
  OutputBridge::Config output_cfg_;
  std::string output_frame_ = "mgrs";     ///< mgrs | enu | relative
  std::string output_point_ = "base_link";  ///< base_link | master
  bool output_master_point_ = false;
  int utm_zone_ = 37;
  std::vector<double> mgrs_origin_{300000.0, 6100000.0};
  geo::OutputProjector projector_;
  bool warned_unprojected_ = false;
  double master_antenna_x_ = -9.873;      ///< TF антенны master в base_link, м
  double rover_antenna_x_ = 2.563;        ///< TF антенны rover в base_link, м
  ModelParams params_;

  struct PluginNames
  {
    std::string preprocessor;
    std::string traction;
    std::string slip;
    std::string estimator;
    std::string integrator;
    std::string initializer;
  };

  // Компоненты.
  std::unique_ptr<InputBridge> input_bridge_;
  std::unique_ptr<EstimationPipeline> pipeline_;
  std::unique_ptr<OutputBridge> output_bridge_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Плагины (загружаются по имени из конфига).
  std::shared_ptr<IPreprocessor> plugin_preprocessor_;
  std::shared_ptr<ITractionModel> plugin_traction_;
  std::shared_ptr<ISlipDetector> plugin_slip_;
  std::shared_ptr<IStateEstimator> plugin_estimator_;
  std::shared_ptr<IPositionIntegrator> plugin_integrator_;
  std::shared_ptr<IPositionInitializer> plugin_initializer_;
  PluginNames plugin_names_;

  // 1D-карта маршрута и выставка по GNSS.
  std::shared_ptr<RouteMap> map_;
  bool map_enabled_ = false;        ///< map.enabled из конфига
  bool map_bound_ = false;         ///< true, если привязаны к карте (иначе относительный режим)
  bool map_init_done_ = false;     ///< выставка завершена (привязка или относительный режим)
  std::string position_antenna_ = "master";     ///< антенна позиции (ведущая) -> s0
  std::string orientation_antenna_ = "rover";   ///< антенна ориентации (передняя) -> heading
  double init_window_s_ = 2.0;      ///< окно для fallback-направления (смещение)
  double init_timeout_s_ = 10.0;   ///< таймаут выставки, с
  double min_init_displacement_m_ = 0.5;  ///< порог движения для fallback
  double gnss_match_tolerance_s_ = 0.1;    ///< допуск совпадения меток master/rover
  bool init_forward_only_ = true;          ///< выставка только на пути, идущие по курсу (+s)
  double init_forward_max_cross_m_ = 50.0;  ///< предел отклонения для выставки по курсу, м
  double init_baseline_tol_m_ = 1.0;       ///< пара фиксов с базой дальше от номинала — шум

  // Коррекция положения по GNSS после выставки (map.gnss_*).
  bool gnss_corr_enabled_ = false;       ///< map.gnss_correction
  int gnss_corr_min_status_ = 2;         ///< фикс не хуже (2 — RTK/GBAS)
  double gnss_corr_max_cross_m_ = 2.0;   ///< антенна дальше от пути — фикс не принимается, м
  double gnss_corr_gate_m_ = 50.0;       ///< окно поправки s не больше, м
  double gnss_corr_gate_min_m_ = 3.0;    ///< окно поправки s не меньше, м
  double gnss_corr_gate_k_ = 3.0;        ///< окно поправки s, сигм неопределённости пути
  bool gnss_corr_require_pair_ = true;   ///< только по согласной паре master + rover
  double gnss_corr_pair_tol_m_ = 1.0;    ///< base_link по двум антеннам расходится больше — шум, м
  double gnss_corr_window_s_ = 1.0;      ///< окно проверки согласованности поправок, с
  int gnss_corr_min_fixes_ = 5;          ///< поправок в окне не меньше
  double gnss_corr_max_rate_ = 0.5;      ///< поправка плывёт быстрее — фикс застыл, м/с
  std::deque<std::pair<int64_t, double>> corr_hist_;  ///< (метка фикса, поправка s) в окне
  bool gnss_reloc_enabled_ = true;       ///< map.gnss_reloc: перепривязка к другому пути
  double gnss_reloc_max_cross_m_ = 5.0;  ///< антенна дальше от нового пути — не перепривязываемся, м
  double gnss_reloc_min_disp_m_ = 1.0;   ///< смещение серии фиксов для курса не меньше, м
  double gnss_reloc_margin_m_ = 2.0;     ///< новый путь ближе текущего хотя бы на, м
  std::deque<GnssFix> recent_fix_[2];    ///< фиксы master / rover за окно (для курса)
  std::size_t n_gnss_reloc_ = 0;         ///< перепривязок
  GnssFix corr_fix_[2];                  ///< последний фикс master / rover (для пары)
  bool corr_has_[2] = {false, false};
  double gnss_corr_max_age_s_ = 0.3;     ///< метка фикса дальше от оценки — фикс не принимается, с
  double gnss_corr_min_interval_s_ = 1.0;  ///< не чаще, с
  double gnss_corr_sigma_m_ = 0.3;       ///< неопределённость s после коррекции, м
  Estimate last_est_;                    ///< оценка последнего такта (s, v, маршрут)
  bool has_last_est_ = false;
  int64_t last_gnss_corr_ns_ = 0;
  std::size_t n_gnss_corr_ = 0;          ///< принятых коррекций
  std::size_t n_gnss_rej_ = 0;           ///< отброшенных фиксов

  std::deque<GnssFix> gnss_buffer_;   ///< фиксы антенны позиции (master): s0 + fallback
  GnssFix latest_ori_fix_;           ///< последний фикс антенны ориентации (rover)
  bool has_ori_fix_ = false;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pos_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_ori_sub_;
  int64_t gnss_first_ns_ = 0;
  int64_t first_input_ns_ = 0;        ///< первая метка входа (время bag) — таймаут без GNSS
  int64_t last_published_stamp_ns_ = 0;  ///< не публикуем повторно ту же метку
  OutputHook output_hook_;             ///< копия публикуемого (офлайн-прогон)

  // Пути файлов карты (пусто = дефолт в share пакета) и ENU-reference.
  std::string map_ring_file_;
  std::string map_branches_file_;
  std::string map_stops_file_;
  double map_ref_lat_ = 55.810367, map_ref_lon_ = 37.462267, map_ref_alt_ = 168.0;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__NODE__RESERVE_ODOMETRY_NODE_HPP_
