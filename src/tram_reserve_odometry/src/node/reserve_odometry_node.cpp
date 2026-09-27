#include "tram_reserve_odometry/node/reserve_odometry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "tram_reserve_odometry/core/interfaces.hpp"
#include "tram_reserve_odometry/core/plugin_registry.hpp"

namespace tram
{

namespace
{
constexpr const char * kPackage = "tram_reserve_odometry";
}

ReserveOdometryNode::ReserveOdometryNode(const rclcpp::NodeOptions & options)
: Node("reserve_odometry", options)
{
  declare_and_read_params();

  // Входной мост (подписчики) заполняет буфер сырых образцов.
  input_bridge_ = std::make_unique<InputBridge>(
    this, front_topic_, rear_topic_, driver_topic_);

  if (!load_plugins()) {
    RCLCPP_ERROR(get_logger(), "Failed to load one or more plugins; node will not run.");
    return;
  }

  pipeline_ = std::make_unique<EstimationPipeline>(
    plugin_traction_, plugin_slip_, plugin_estimator_,
    plugin_integrator_, plugin_initializer_, params_);

  setup_map();

  output_bridge_ = std::make_unique<OutputBridge>(this, output_cfg_);

  // Таймер — по стенным часам; время в сообщениях — из входов (bag).
  const auto period = std::chrono::duration<double>(1.0 / output_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this] {on_timer();});

  RCLCPP_INFO(
    get_logger(),
    "reserve_odometry ready: rate=%.1fHz plugins=[preproc=%s traction=%s slip=%s estimator=%s integrator=%s init=%s]",
    output_rate_hz_, plugin_preprocessor_->name().c_str(), plugin_traction_->name().c_str(),
    plugin_slip_->name().c_str(), plugin_estimator_->name().c_str(),
    plugin_integrator_->name().c_str(), plugin_initializer_->name().c_str());
}

ReserveOdometryNode::~ReserveOdometryNode() = default;

void ReserveOdometryNode::declare_and_read_params()
{
  output_rate_hz_ = declare_parameter<double>("output_rate_hz", 20.0);
  max_lead_s_ = declare_parameter<double>("output.max_lead_s", 0.3);
  velocity_delay_s_ = std::max(declare_parameter<double>("output.velocity_delay_s", 0.0), 0.0);

  front_topic_ = declare_parameter<std::string>(
    "input.front_bogie_velocity_topic", "/vehicle/front_bogie_velocity");
  rear_topic_ = declare_parameter<std::string>(
    "input.rear_bogie_velocity_topic", "/vehicle/rear_bogie_velocity");
  driver_topic_ = declare_parameter<std::string>(
    "input.driver_position_topic", "/vehicle/driver_position_cmd");

  output_cfg_.velocity_topic = declare_parameter<std::string>(
    "output.velocity_topic", "/result/velocity");
  output_cfg_.position_topic = declare_parameter<std::string>(
    "output.position_topic", "/result/position");
  output_cfg_.frame_id = declare_parameter<std::string>("output.frame_id", "map");
  output_cfg_.child_frame_id = declare_parameter<std::string>(
    "output.child_frame_id", "tram_base_link");
  output_cfg_.publish_debug = declare_parameter<bool>("output.publish_debug", true);
  output_frame_ = declare_parameter<std::string>("output.frame", "mgrs");
  output_point_ = declare_parameter<std::string>("output.point", "base_link");
  utm_zone_ = static_cast<int>(declare_parameter<int64_t>("output.utm_zone", 37));
  mgrs_origin_ = declare_parameter<std::vector<double>>(
    "output.mgrs_origin", std::vector<double>{300000.0, 6100000.0});

  // TF антенн в base_link (x вперёд, REP 103). base_link — ось поворота передней
  // тележки на уровне головки рельса.
  master_antenna_x_ = declare_parameter<double>("vehicle.master_antenna_x", -9.873);
  rover_antenna_x_ = declare_parameter<double>("vehicle.rover_antenna_x", 2.563);

  // Приведение единиц входов -> масштабы в ModelParams (читает препроцессор-плагин).
  bogie_unit_ = declare_parameter<std::string>("input.bogie_velocity_unit", "kmh");  // "kmh" или "ms"
  driver_cmd_max_ = declare_parameter<double>("input.driver_cmd_max", 15.0);
  params_.bogie_scale = (bogie_unit_ == "ms") ? 1.0 : (1.0 / 3.6);  // км/ч -> м/с
  params_.driver_scale = (driver_cmd_max_ > 0.0) ? (1.0 / driver_cmd_max_) : 1.0;

  // Имена плагинов.
  const std::string preprocessor = declare_parameter<std::string>(
    "plugins.preprocessor", "tram::PassthroughPreprocessor");
  const std::string traction = declare_parameter<std::string>(
    "plugins.traction_model", "tram::TableTractionModel");
  const std::string slip = declare_parameter<std::string>(
    "plugins.slip_detector", "tram::AlwaysGoodSlipDetector");
  const std::string estimator = declare_parameter<std::string>(
    "plugins.estimator", "tram::KalmanWheelEstimator");
  const std::string integrator = declare_parameter<std::string>(
    "plugins.integrator", "tram::StopResetRouteIntegrator");
  const std::string initializer = declare_parameter<std::string>(
    "plugins.position_initializer", "tram::ZeroPositionInitializer");

  // Параметры алгоритмов (секции estimator:/integrator:/drive:) -> ModelParams::extra;
  // плагины берут нужные ключи сами.
  static const std::vector<std::string> kExtraPrefixes = {"estimator.", "integrator.", "drive."};
  for (const auto & kv : get_node_parameters_interface()->get_parameter_overrides()) {
    std::string key;
    for (const auto & prefix : kExtraPrefixes) {
      if (kv.first.rfind(prefix, 0) == 0) {
        key = kv.first.substr(prefix.size());
        break;
      }
    }
    if (key.empty()) {
      continue;
    }
    const rclcpp::ParameterValue & v = kv.second;
    switch (v.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE: params_.extra[key] = v.get<double>(); break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params_.extra[key] = static_cast<double>(v.get<int64_t>()); break;
      case rclcpp::ParameterType::PARAMETER_BOOL: params_.extra[key] = v.get<bool>() ? 1.0 : 0.0; break;
      case rclcpp::ParameterType::PARAMETER_STRING: params_.extra_str[key] = v.get<std::string>(); break;
      default:
        RCLCPP_WARN(get_logger(), "Parameter %s is not numeric/string -> ignored.", kv.first.c_str());
    }
  }
  // Характеристика привода f(u, v) по умолчанию — из share пакета.
  if (params_.extra_str["drv_table_file"].empty()) {
    params_.extra_str["drv_table_file"] =
      ament_index_cpp::get_package_share_directory(kPackage) + "/maps/drive_map.csv";
  }

  // Инициализация.
  std::vector<double> init_pos = declare_parameter<std::vector<double>>(
    "initialization.initial_position", std::vector<double>{0.0, 0.0, 0.0});
  params_.initial_velocity = declare_parameter<double>("initialization.initial_velocity", 0.0);
  if (init_pos.size() >= 3) {
    params_.initial_position = {init_pos[0], init_pos[1], init_pos[2]};
  }

  // 1D-карта маршрута и выставка по GNSS: антенна позиции -> s0, антенна ориентации -> курс.
  map_enabled_ = declare_parameter<bool>("map.enabled", false);
  position_antenna_ = declare_parameter<std::string>("map.position_antenna", "master");
  orientation_antenna_ = declare_parameter<std::string>("map.orientation_antenna", "rover");
  init_window_s_ = declare_parameter<double>("map.init_window_s", 2.0);
  init_timeout_s_ = declare_parameter<double>("map.init_timeout_s", 10.0);
  min_init_displacement_m_ = declare_parameter<double>("map.min_init_displacement_m", 0.5);
  gnss_match_tolerance_s_ = declare_parameter<double>("map.gnss_match_tolerance_s", 0.1);
  init_forward_only_ = declare_parameter<bool>("map.init_forward_only", true);
  init_forward_max_cross_m_ = declare_parameter<double>("map.init_forward_max_cross_m", 50.0);
  init_baseline_tol_m_ = declare_parameter<double>("map.init_baseline_tol_m", 1.0);
  gnss_corr_enabled_ = declare_parameter<bool>("map.gnss_correction", false);
  gnss_corr_min_status_ = static_cast<int>(declare_parameter<int64_t>("map.gnss_corr_min_status", 2));
  gnss_corr_max_cross_m_ = declare_parameter<double>("map.gnss_corr_max_cross_m", 2.0);
  gnss_corr_gate_m_ = declare_parameter<double>("map.gnss_corr_gate_m", 50.0);
  gnss_corr_gate_min_m_ = declare_parameter<double>("map.gnss_corr_gate_min_m", 3.0);
  gnss_corr_gate_k_ = declare_parameter<double>("map.gnss_corr_gate_k", 3.0);
  gnss_corr_require_pair_ = declare_parameter<bool>("map.gnss_corr_require_pair", true);
  gnss_corr_pair_tol_m_ = declare_parameter<double>("map.gnss_corr_pair_tol_m", 1.0);
  gnss_corr_window_s_ = declare_parameter<double>("map.gnss_corr_window_s", 1.0);
  gnss_corr_min_fixes_ = static_cast<int>(declare_parameter<int64_t>("map.gnss_corr_min_fixes", 5));
  gnss_corr_max_rate_ = declare_parameter<double>("map.gnss_corr_max_rate", 0.5);
  gnss_reloc_enabled_ = declare_parameter<bool>("map.gnss_reloc", true);
  gnss_reloc_max_cross_m_ = declare_parameter<double>("map.gnss_reloc_max_cross_m", 5.0);
  gnss_reloc_min_disp_m_ = declare_parameter<double>("map.gnss_reloc_min_disp_m", 1.0);
  gnss_reloc_margin_m_ = declare_parameter<double>("map.gnss_reloc_margin_m", 2.0);
  gnss_corr_max_age_s_ = declare_parameter<double>("map.gnss_corr_max_age_s", 0.3);
  gnss_corr_min_interval_s_ = declare_parameter<double>("map.gnss_corr_min_interval_s", 1.0);
  gnss_corr_sigma_m_ = declare_parameter<double>("map.gnss_corr_sigma_m", 0.3);
  const std::string ring_file = declare_parameter<std::string>("map.ring_file", "");
  const std::string branches_file = declare_parameter<std::string>("map.branches_file", "");
  const std::string stops_file = declare_parameter<std::string>("map.stops_file", "");
  std::vector<double> ref = declare_parameter<std::vector<double>>(
    "map.reference", std::vector<double>{55.810367, 37.462267, 168.0});
  if (ref.size() >= 3) {
    map_ref_lat_ = ref[0];
    map_ref_lon_ = ref[1];
    map_ref_alt_ = ref[2];
  }
  map_ring_file_ = ring_file;

  // Выходная система координат (опорная ENU = map.reference).
  geo::OutputProjector::Config pcfg;
  if (!geo::OutputProjector::parse_frame(output_frame_, pcfg.frame)) {
    RCLCPP_WARN(
      get_logger(), "Unknown output.frame '%s' -> using 'mgrs'.", output_frame_.c_str());
    pcfg.frame = geo::OutputProjector::Frame::MGRS;
  }
  pcfg.enu_ref = geo::EnuReference{map_ref_lat_, map_ref_lon_, map_ref_alt_};
  pcfg.utm_zone = utm_zone_;
  if (mgrs_origin_.size() >= 2) {
    pcfg.mgrs_origin_x = mgrs_origin_[0];
    pcfg.mgrs_origin_y = mgrs_origin_[1];
  }
  projector_ = geo::OutputProjector(pcfg);
  if (output_point_ != "base_link" && output_point_ != "master") {
    RCLCPP_WARN(
      get_logger(), "Unknown output.point '%s' -> using 'base_link'.", output_point_.c_str());
    output_point_ = "base_link";
  }
  output_master_point_ = (output_point_ == "master");
  map_branches_file_ = branches_file;
  map_stops_file_ = stops_file;

  // Сохранить имена плагинов для load_plugins().
  plugin_names_.preprocessor = preprocessor;
  plugin_names_.traction = traction;
  plugin_names_.slip = slip;
  plugin_names_.estimator = estimator;
  plugin_names_.integrator = integrator;
  plugin_names_.initializer = initializer;
}

bool ReserveOdometryNode::load_plugins()
{
  plugin_preprocessor_ = create_plugin<IPreprocessor>(
    kPackage, "tram::IPreprocessor", plugin_names_.preprocessor);
  plugin_traction_ = create_plugin<ITractionModel>(
    kPackage, "tram::ITractionModel", plugin_names_.traction);
  plugin_slip_ = create_plugin<ISlipDetector>(
    kPackage, "tram::ISlipDetector", plugin_names_.slip);
  plugin_estimator_ = create_plugin<IStateEstimator>(
    kPackage, "tram::IStateEstimator", plugin_names_.estimator);
  plugin_integrator_ = create_plugin<IPositionIntegrator>(
    kPackage, "tram::IPositionIntegrator", plugin_names_.integrator);
  plugin_initializer_ = create_plugin<IPositionInitializer>(
    kPackage, "tram::IPositionInitializer", plugin_names_.initializer);

  if (!plugin_preprocessor_ || !plugin_traction_ || !plugin_slip_ ||
    !plugin_estimator_ || !plugin_integrator_ || !plugin_initializer_)
  {
    if (!plugin_preprocessor_) {
      RCLCPP_ERROR(get_logger(), "Preprocessor plugin not found: %s", plugin_names_.preprocessor.c_str());
    }
    if (!plugin_traction_) {
      RCLCPP_ERROR(get_logger(), "Traction model plugin not found: %s", plugin_names_.traction.c_str());
    }
    if (!plugin_slip_) {
      RCLCPP_ERROR(get_logger(), "Slip detector plugin not found: %s", plugin_names_.slip.c_str());
    }
    if (!plugin_estimator_) {
      RCLCPP_ERROR(get_logger(), "State estimator plugin not found: %s", plugin_names_.estimator.c_str());
    }
    if (!plugin_integrator_) {
      RCLCPP_ERROR(get_logger(), "Position integrator plugin not found: %s", plugin_names_.integrator.c_str());
    }
    if (!plugin_initializer_) {
      RCLCPP_ERROR(get_logger(), "Position initializer plugin not found: %s", plugin_names_.initializer.c_str());
    }
    return false;
  }
  return true;
}

void ReserveOdometryNode::on_timer()
{
  // Прочитать последние сырые образцы и прогнать через препроцессор-плагин.
  const InputSample front = input_bridge_->buffer().get(Channel::FRONT);
  const InputSample rear = input_bridge_->buffer().get(Channel::REAR);
  const InputSample driver = input_bridge_->buffer().get(Channel::DRIVER);

  const int64_t now_ns = now_fn_ ? now_fn_() :
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const int64_t max_lead_ns = static_cast<int64_t>(std::max(0.0, max_lead_s_) * 1e9);

  // Метка публикации — метка входа плюс время с его приёма (не больше max_lead_s):
  // «сейчас» в шкале времени bag; фильтр прогнозирует к этой метке.
  int64_t input_stamp_ns = 0;
  int64_t newest_recv_ns = 0;
  int64_t publish_stamp_ns = 0;
  for (const InputSample * s : {&front, &rear, &driver}) {
    if (s->valid) {
      input_stamp_ns = std::max(input_stamp_ns, s->stamp_ns);
      newest_recv_ns = std::max(newest_recv_ns, s->recv_ns);
      const int64_t lead = std::clamp<int64_t>(now_ns - s->recv_ns, 0, max_lead_ns);
      publish_stamp_ns = std::max(publish_stamp_ns, s->stamp_ns + lead);
    }
  }
  if (first_input_ns_ == 0 && input_stamp_ns > 0) {
    first_input_ns_ = input_stamp_ns;
  }

  // Выставка по GNSS (однократно): привязка к карте или относительный режим.
  try_initialize_map(input_stamp_ns);

  PreprocessedSample sample = plugin_preprocessor_->process(front, rear, driver, params_);
  if (sample.stamp_ns > 0) {
    // Метка не убывает.
    sample.stamp_ns = std::max({sample.stamp_ns, publish_stamp_ns, last_fed_stamp_ns_});
    last_fed_stamp_ns_ = sample.stamp_ns;
  }
  const Estimate est = pipeline_->process(sample);
  if (map_bound_ && est.on_route) {
    last_est_ = est;  // для коррекции по GNSS: s, v, маршрут на метке такта
    has_last_est_ = true;
  }

  // Публикуем только новую метку времени.
  if (sample.stamp_ns <= 0 || sample.stamp_ns == last_published_stamp_ns_) {
    return;
  }
  last_published_stamp_ns_ = sample.stamp_ns;

  RCLCPP_DEBUG(
    get_logger(), "state: t=%.3f vel=%.3f s=%.2f pos=(%.2f,%.2f) route=%zu on_route=%d dir=%d",
    static_cast<double>(sample.stamp_ns) * 1e-9, est.velocity, est.s,
    est.position.x, est.position.y, est.route_id, static_cast<int>(est.on_route),
    est.direction);

  // Задержка «вход -> публикация»: от приёма самого свежего входа, steady clock.
  double latency_ms = 0.0;
  if (newest_recv_ns > 0) {
    latency_ms = std::max(0.0, static_cast<double>(now_ns - newest_recv_ns) * 1e-6);
  }

  // Пока идёт выставка, позиция неизвестна: публикуем только скорость.
  const bool position_known = !map_enabled_ || map_init_done_;
  Estimate published = to_output(est);
  if (velocity_delay_s_ > 0.0) {
    published.velocity = delayed_velocity(est.stamp_ns, est.velocity);
  }
  if (output_hook_) {
    output_hook_(published, position_known);
  }
  output_bridge_->publish(published, latency_ms, position_known);
}

bool ReserveOdometryNode::ingest_input(
  const std::string & topic, const tram_vehicle_msgs::msg::VelocitySensor & msg,
  int64_t recv_ns)
{
  if (topic == front_topic_) {
    input_bridge_->inject_velocity(msg, Channel::FRONT, recv_ns);
  } else if (topic == rear_topic_) {
    input_bridge_->inject_velocity(msg, Channel::REAR, recv_ns);
  } else {
    return false;
  }
  return true;
}

bool ReserveOdometryNode::ingest_input(
  const std::string & topic, const DriverCommand & msg, int64_t recv_ns)
{
  if (topic != driver_topic_) {
    return false;
  }
  input_bridge_->inject_driver(msg, recv_ns);
  return true;
}

double ReserveOdometryNode::delayed_velocity(int64_t stamp_ns, double velocity)
{
  // Скорость на момент stamp − velocity_delay_s из истории оценок; положение — по текущей.
  vel_hist_.emplace_back(stamp_ns, velocity);
  const int64_t delay_ns = static_cast<int64_t>(velocity_delay_s_ * 1e9);
  const int64_t t = stamp_ns - delay_ns;
  while (vel_hist_.size() > 2 && vel_hist_[1].first <= t) {
    vel_hist_.pop_front();
  }
  if (vel_hist_.front().first >= t) {
    return vel_hist_.front().second;  // история короче задержки (старт)
  }
  const auto & a = vel_hist_[0];
  const auto & b = vel_hist_[1];
  const double w = static_cast<double>(t - a.first) / static_cast<double>(b.first - a.first);
  return a.second + std::clamp(w, 0.0, 1.0) * (b.second - a.second);
}

void ReserveOdometryNode::ingest_gnss_fix(
  const std::string & topic, const sensor_msgs::msg::NavSatFix & msg)
{
  if (!map_enabled_) {
    return;
  }
  if (map_init_done_) {
    if (gnss_corr_enabled_) {
      for (const std::string ant : {"master", "rover"}) {
        if (topic == "/sensing/gnss/" + ant + "/fix") {
          correct_by_gnss(ant, msg);
        }
      }
    }
    return;
  }
  auto copy = std::make_shared<sensor_msgs::msg::NavSatFix>(msg);
  if (topic == "/sensing/gnss/" + position_antenna_ + "/fix") {
    on_gnss_fix(copy);
  } else if (topic == "/sensing/gnss/" + orientation_antenna_ + "/fix") {
    on_ori_fix(copy);
  }
}

double ReserveOdometryNode::antenna_x(const std::string & name) const
{
  return (name == "rover") ? rover_antenna_x_ : master_antenna_x_;
}

Estimate ReserveOdometryNode::to_output(const Estimate & est)
{
  Estimate out = est;
  // Точка публикации: base_link или точка пути под антенной master (s + dir·x_master).
  if (output_master_point_) {
    if (est.on_route && map_) {
      Point3D p;
      double hd = est.heading;
      if (map_->lookup(est.route_id, est.s + est.direction * master_antenna_x_, p, hd)) {
        out.position = p;
        out.heading = hd;
      }
    } else {
      out.position.x += master_antenna_x_ * std::cos(est.heading);
      out.position.y += master_antenna_x_ * std::sin(est.heading);
    }
  }
  // Система координат: MGRS/ENU имеют смысл только при привязке к карте
  // (иначе позиция — относительная одометрия от старта).
  if (!est.on_route && projector_.config().frame == geo::OutputProjector::Frame::MGRS) {
    // До конца выставки позиция не публикуется — предупреждать не о чем.
    if (!warned_unprojected_ && (!map_enabled_ || map_init_done_)) {
      RCLCPP_WARN(
        get_logger(),
        "Position is not bound to the map (no GNSS init) -> publishing relative odometry "
        "from start instead of MGRS.");
      warned_unprojected_ = true;
    }
    return out;
  }
  const double hd = projector_.project_heading(out.position, out.heading);
  out.position = projector_.project(out.position);
  out.heading = hd;
  return out;
}

void ReserveOdometryNode::setup_map()
{
  if (!map_enabled_) {
    return;
  }
  const std::string share = ament_index_cpp::get_package_share_directory(kPackage);
  const std::string ring =
    map_ring_file_.empty() ? (share + "/maps/route_ring.csv") : map_ring_file_;
  const std::string branches =
    map_branches_file_.empty() ? (share + "/maps/route_branches.csv") : map_branches_file_;
  const std::string stops =
    map_stops_file_.empty() ? (share + "/maps/stops.yaml") : map_stops_file_;

  auto m = std::make_shared<RouteMap>();
  m->set_reference(map_ref_lat_, map_ref_lon_, map_ref_alt_);
  if (!m->load_ring(ring) || !m->has_routes()) {
    RCLCPP_WARN(get_logger(), "Map not loaded (%s) -> relative mode.", ring.c_str());
    map_enabled_ = false;
    return;
  }
  (void)m->load_branches(branches);  // ветки (не критично, если файла нет)
  (void)m->load_stops(stops);        // остановки (не критично)
  map_ = m;
  plugin_estimator_->set_route_map(map_);  // уклон и кривизна для модели динамики
  RCLCPP_INFO(
    get_logger(), "Map loaded: L=%.1f m, %zu routes, %zu stops.",
    map_->ring_length(), map_->num_routes(), map_->stops().size());

  const std::string pos_topic = "/sensing/gnss/" + position_antenna_ + "/fix";
  const std::string ori_topic = "/sensing/gnss/" + orientation_antenna_ + "/fix";
  gnss_pos_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    pos_topic, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {on_gnss_fix(msg);});
  gnss_ori_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    ori_topic, rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {on_ori_fix(msg);});
  RCLCPP_INFO(
    get_logger(),
    "GNSS init (compass): pos=%s ori=%s (match tol %.2fs, fallback window %.1fs, timeout %.1fs).",
    pos_topic.c_str(), ori_topic.c_str(), gnss_match_tolerance_s_, init_window_s_, init_timeout_s_);
}

ReserveOdometryNode::GnssFix ReserveOdometryNode::make_fix(const sensor_msgs::msg::NavSatFix & msg)
{
  GnssFix fix;
  fix.stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
    msg.header.stamp.nanosec;
  fix.lat = msg.latitude;
  fix.lon = msg.longitude;
  fix.alt = msg.altitude;
  fix.status = msg.status.status;
  return fix;
}

void ReserveOdometryNode::on_gnss_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (map_init_done_) {
    correct_by_gnss(position_antenna_, *msg);
    return;
  }
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    return;
  }
  const GnssFix fix = make_fix(*msg);
  if (gnss_buffer_.empty()) {
    gnss_first_ns_ = fix.stamp_ns;
  }
  gnss_buffer_.push_back(fix);
  while (gnss_buffer_.size() > 2000) {
    gnss_buffer_.pop_front();
  }
}

void ReserveOdometryNode::on_ori_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (map_init_done_) {
    correct_by_gnss(orientation_antenna_, *msg);
    return;
  }
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    return;
  }
  latest_ori_fix_ = make_fix(*msg);
  has_ori_fix_ = true;
}

void ReserveOdometryNode::bind_map(std::size_t route_id, double s0, int dir, double cross_track)
{
  plugin_integrator_->set_route_map(map_);
  plugin_integrator_->set_route_id(route_id);
  plugin_integrator_->set_initial_s(s0);
  plugin_integrator_->set_direction(dir);
  plugin_integrator_->set_initial_uncertainty(cross_track);
  map_bound_ = true;
  map_init_done_ = true;
  if (!gnss_corr_enabled_) {
    // Без коррекции по GNSS подписки больше не нужны.
    gnss_pos_sub_.reset();
    gnss_ori_sub_.reset();
  }
  gnss_buffer_.clear();
}

bool ReserveOdometryNode::gnss_delta(
  const std::string & antenna, const GnssFix & fix, double & delta, double & ct,
  bool * off_route) const
{
  if (off_route) {
    *off_route = false;
  }
  // Фикс с меткой, отстающей от оценки больше gnss_corr_max_age_s, отбрасываем.
  const double age = static_cast<double>(last_est_.stamp_ns - fix.stamp_ns) * 1e-9;
  if (std::abs(age) > gnss_corr_max_age_s_) {
    return false;
  }
  // s base_link на момент фикса по текущей оценке; антенна — на x_ant вдоль пути.
  const int dir = (last_est_.direction < 0) ? -1 : 1;
  const double x_ant = antenna_x(antenna);
  const double s_pred = last_est_.s - dir * last_est_.velocity * age;
  const Point3D p = map_->to_enu(fix.lat, fix.lon, fix.alt);
  std::size_t rid = 0;
  double s_ant = 0.0;
  if (!map_->project(p.x, p.y, last_est_.route_id, s_pred + dir * x_ant, rid, s_ant, ct) ||
    rid != last_est_.route_id || ct > gnss_corr_max_cross_m_)
  {
    if (off_route) {
      *off_route = true;  // антенна не на текущем пути: другой путь или ошибка > 60 м
    }
    return false;
  }
  delta = (s_ant - dir * x_ant) - s_pred;
  const double L = map_->ring_length();
  if (rid == 0 && L > 0.0) {
    delta = std::remainder(delta, L);  // кольцо: поправка по кратчайшей дуге
  }
  return true;
}

void ReserveOdometryNode::try_relocate(int idx)
{
  const auto & q = recent_fix_[idx];
  if (!gnss_reloc_enabled_ || static_cast<int>(q.size()) < gnss_corr_min_fixes_) {
    return;
  }
  const GnssFix & f0 = q.front();
  const GnssFix & f1 = q.back();
  const std::string antenna = (idx == 1) ? "rover" : "master";
  const double age = static_cast<double>(last_est_.stamp_ns - f1.stamp_ns) * 1e-9;
  if (std::abs(age) > gnss_corr_max_age_s_) {
    return;
  }
  // Курс — по смещению антенны за серию: без движения путь и направление не определить.
  const Point3D p0 = map_->to_enu(f0.lat, f0.lon, f0.alt);
  const Point3D p1 = map_->to_enu(f1.lat, f1.lon, f1.alt);
  const double disp = std::hypot(p1.x - p0.x, p1.y - p0.y);
  if (disp < gnss_reloc_min_disp_m_) {
    return;
  }
  const double heading = std::atan2(p1.y - p0.y, p1.x - p0.x);
  // Кандидаты: путь, чьё +s смотрит по курсу (dir = +1), и ветка, чьё +s смотрит против
  // курса (dir = -1: едем к тупику). На кольце — только +s.
  std::size_t rid = 0;
  double s_ant = 0.0, ct = 1e30;
  int dir = 1;
  {
    std::size_t r = 0;
    double s = 0.0, c = 0.0;
    if (map_->project_global_heading(p1.x, p1.y, heading, M_PI / 4.0, r, s, c) && c < ct) {
      rid = r, s_ant = s, ct = c, dir = 1;
    }
    if (map_->project_global_heading(p1.x, p1.y, heading + M_PI, M_PI / 4.0, r, s, c) &&
      r != 0 && c < ct)
    {
      rid = r, s_ant = s, ct = c, dir = -1;
    }
  }
  if (ct > gnss_reloc_max_cross_m_) {
    return;
  }
  // Текущий путь рядом с оценкой (окно проекции ±60 м): новый должен быть заметно ближе,
  // а прыжок по тому же пути — только при фиксе на пути (обычный допуск поправки).
  double ct_cur = 1e30;
  {
    std::size_t r = 0;
    double s = 0.0, c = 0.0;
    const int dcur = (last_est_.direction < 0) ? -1 : 1;
    const double hint = last_est_.s - dcur * last_est_.velocity * age + dcur * antenna_x(antenna);
    if (map_->project(p1.x, p1.y, last_est_.route_id, hint, r, s, c)) {
      ct_cur = c;
    }
  }
  if (ct + gnss_reloc_margin_m_ > ct_cur ||
    (rid == last_est_.route_id && ct > gnss_corr_max_cross_m_))
  {
    RCLCPP_DEBUG(get_logger(), "GNSS relocation rejected: route %zu cross %.2f vs current %.2f.",
      rid, ct, ct_cur);
    return;
  }
  // Серия должна лечь на тот же путь: каждый фикс рядом с ним, s антенны растёт по
  // направлению со скоростью, близкой к оценке.
  for (const GnssFix & f : q) {
    const Point3D p = map_->to_enu(f.lat, f.lon, f.alt);
    std::size_t r = 0;
    double s = 0.0, c = 0.0;
    const double dt = static_cast<double>(f1.stamp_ns - f.stamp_ns) * 1e-9;
    const double hint = s_ant - dir * last_est_.velocity * dt;
    if (!map_->project(p.x, p.y, rid, hint, r, s, c) || c > gnss_reloc_max_cross_m_ ||
      std::abs(s - hint) > 1.0 + 0.5 * dt)
    {
      return;
    }
  }
  const double x_ant = antenna_x(antenna);
  const double s_new = s_ant - dir * x_ant + dir * last_est_.velocity * age;
  if (rid == last_est_.route_id) {
    const double L = map_->ring_length();
    const double jump = (rid == 0 && L > 0.0) ?
      std::remainder(s_new - last_est_.s, L) : s_new - last_est_.s;
    if (std::abs(jump) <= 60.0) {
      return;  // тот же путь рядом — это обычная поправка, её проверяет correct_by_gnss
    }
  }
  plugin_integrator_->set_route_id(rid);
  plugin_integrator_->set_direction(dir);
  plugin_integrator_->correct_s(s_new, gnss_corr_sigma_m_);
  RCLCPP_INFO(
    get_logger(),
    "GNSS relocation (%s): route %zu s=%.1f -> route %zu s=%.1f dir=%+d (cross %.2f m, "
    "current route %.2f m).",
    antenna.c_str(), last_est_.route_id, last_est_.s, rid, s_new, dir, ct, ct_cur);
  last_est_.route_id = rid;
  last_est_.s = s_new;
  last_est_.direction = dir;
  last_est_.position_cov = gnss_corr_sigma_m_ * gnss_corr_sigma_m_;
  last_gnss_corr_ns_ = f1.stamp_ns;
  corr_hist_.clear();
  recent_fix_[0].clear();
  recent_fix_[1].clear();
  ++n_gnss_reloc_;
}

void ReserveOdometryNode::correct_by_gnss(
  const std::string & antenna, const sensor_msgs::msg::NavSatFix & msg)
{
  if (!gnss_corr_enabled_ || !map_bound_ || !has_last_est_ || !map_) {
    return;
  }
  if (msg.status.status < gnss_corr_min_status_) {
    ++n_gnss_rej_;  // без RTK ошибка фикса — метры
    return;
  }
  const GnssFix fix = make_fix(msg);
  const int idx = (antenna == "rover") ? 1 : 0;
  corr_fix_[idx] = fix;
  corr_has_[idx] = true;
  auto & recent = recent_fix_[idx];
  if (!recent.empty() && fix.stamp_ns - recent.back().stamp_ns > 500000000LL) {
    recent.clear();  // разрыв — новая серия
  }
  recent.push_back(fix);
  while (!recent.empty() &&
    recent.front().stamp_ns < fix.stamp_ns - static_cast<int64_t>(gnss_corr_window_s_ * 1e9))
  {
    recent.pop_front();
  }
  double delta = 0.0, ct = 0.0;
  bool off_route = false;
  if (!gnss_delta(antenna, fix, delta, ct, &off_route)) {
    ++n_gnss_rej_;
    if (off_route) {
      try_relocate(idx);
    }
    return;
  }
  if (gnss_corr_require_pair_) {
    // Пара master + rover: база сходится, base_link по обеим антеннам совпадает.
    const int other = 1 - idx;
    if (!corr_has_[other] ||
      std::abs(static_cast<double>(corr_fix_[other].stamp_ns - fix.stamp_ns) * 1e-9) >
      gnss_match_tolerance_s_)
    {
      return;  // ждём фикс второй антенны
    }
    const std::string other_ant = (other == 1) ? "rover" : "master";
    double delta_o = 0.0, ct_o = 0.0;
    const Point3D p = map_->to_enu(fix.lat, fix.lon, fix.alt);
    const Point3D q = map_->to_enu(corr_fix_[other].lat, corr_fix_[other].lon, corr_fix_[other].alt);
    const double base = std::hypot(p.x - q.x, p.y - q.y);
    const double base_nom = std::abs(antenna_x("rover") - antenna_x("master"));
    if (!gnss_delta(other_ant, corr_fix_[other], delta_o, ct_o) ||
      std::abs(base - base_nom) > init_baseline_tol_m_ ||
      std::abs(delta - delta_o) > gnss_corr_pair_tol_m_)
    {
      ++n_gnss_rej_;
      return;
    }
    delta = 0.5 * (delta + delta_o);
    ct = std::max(ct, ct_o);
  }
  // Согласованность с одометрией: поправки за окно должны совпадать (застывший фикс
  // даёт поправку, плывущую на v·dt за шаг).
  corr_hist_.emplace_back(fix.stamp_ns, delta);
  const int64_t win_ns = static_cast<int64_t>(gnss_corr_window_s_ * 1e9);
  while (!corr_hist_.empty() && corr_hist_.front().first < fix.stamp_ns - win_ns) {
    corr_hist_.pop_front();
  }
  if (static_cast<int>(corr_hist_.size()) < gnss_corr_min_fixes_) {
    return;  // мало фиксов подряд
  }
  double d_min = corr_hist_.front().second, d_max = d_min, d_sum = 0.0, t_sum = 0.0;
  for (const auto & e : corr_hist_) {
    d_min = std::min(d_min, e.second);
    d_max = std::max(d_max, e.second);
    d_sum += e.second;
    t_sum += static_cast<double>(e.first - fix.stamp_ns) * 1e-9;
  }
  // Скорость изменения поправки (МНК): у настоящего фикса ≈ 0 (ошибка масштаба колёс —
  // проценты скорости), у застывшего — минус скорость трамвая.
  const double cnt = static_cast<double>(corr_hist_.size());
  const double t_mean = t_sum / cnt, d_mean = d_sum / cnt;
  double stt = 0.0, std_ = 0.0;
  for (const auto & e : corr_hist_) {
    const double dt = static_cast<double>(e.first - fix.stamp_ns) * 1e-9 - t_mean;
    stt += dt * dt;
    std_ += dt * (e.second - d_mean);
  }
  const double rate = (stt > 1e-6) ? std_ / stt : 0.0;
  if (d_max - d_min > gnss_corr_pair_tol_m_ || std::abs(rate) > gnss_corr_max_rate_) {
    ++n_gnss_rej_;
    return;
  }
  if (last_gnss_corr_ns_ > 0 &&
    static_cast<double>(fix.stamp_ns - last_gnss_corr_ns_) * 1e-9 < gnss_corr_min_interval_s_)
  {
    return;
  }
  delta = d_sum / static_cast<double>(corr_hist_.size());
  // Окно поправки — по накопленной неопределённости пути (position_cov интегратора);
  // без модели ошибки пути (cov = 0) — максимальное.
  const double cov = last_est_.position_cov;
  const double gate = (cov > 0.0) ?
    std::clamp(
    gnss_corr_gate_k_ * std::sqrt(cov + gnss_corr_sigma_m_ * gnss_corr_sigma_m_),
    gnss_corr_gate_min_m_, gnss_corr_gate_m_) :
    gnss_corr_gate_m_;
  if (std::abs(delta) > gate) {
    ++n_gnss_rej_;
    return;
  }
  plugin_integrator_->correct_s(last_est_.s + delta, gnss_corr_sigma_m_);
  last_est_.s += delta;
  last_est_.position_cov = gnss_corr_sigma_m_ * gnss_corr_sigma_m_;
  last_gnss_corr_ns_ = fix.stamp_ns;
  corr_hist_.clear();  // поправки в окне считались от старого s
  ++n_gnss_corr_;
  if (n_gnss_corr_ == 1) {
    RCLCPP_INFO(
      get_logger(), "GNSS correction: first fix accepted (%s, status %d, ds %+.2f m, cross %.2f m).",
      antenna.c_str(), fix.status, delta, ct);
  }
  RCLCPP_DEBUG(
    get_logger(), "GNSS correction #%zu (%s): ds %+.2f m (gate %.1f), cross %.2f m, rejected %zu.",
    n_gnss_corr_, antenna.c_str(), delta, gate, ct, n_gnss_rej_);
}

void ReserveOdometryNode::finish_map_init_unbound()
{
  map_bound_ = false;
  map_init_done_ = true;
  gnss_pos_sub_.reset();
  gnss_ori_sub_.reset();
  gnss_buffer_.clear();
}

bool ReserveOdometryNode::project_for_init(
  double x, double y, double heading, std::size_t & rid, double & s0, double & ct) const
{
  // По кольцу и по веткам трамвай едет в +s: проекция только на пути по курсу.
  if (init_forward_only_ &&
    map_->project_global_heading(x, y, heading, M_PI / 2.0, rid, s0, ct) &&
    ct <= init_forward_max_cross_m_)
  {
    return true;
  }
  return map_->project_global(x, y, rid, s0, ct);
}

void ReserveOdometryNode::try_initialize_map(int64_t input_stamp_ns)
{
  if (!map_enabled_ || map_init_done_) {
    return;
  }
  // Таймаут — по времени bag (метки входов и GNSS).
  const int64_t bag_now_ns = std::max(
    input_stamp_ns, gnss_buffer_.empty() ? int64_t{0} : gnss_buffer_.back().stamp_ns);
  const int64_t start_ns = (gnss_first_ns_ > 0) ? gnss_first_ns_ : first_input_ns_;
  const bool timed_out = start_ns > 0 &&
    (bag_now_ns - start_ns) > static_cast<int64_t>(init_timeout_s_ * 1e9);

  if (gnss_buffer_.empty()) {
    if (timed_out) {
      RCLCPP_WARN(
        get_logger(), "GNSS init: no fix within %.1fs -> relative mode.", init_timeout_s_);
      finish_map_init_unbound();
    }
    return;  // ждём первый фикс позиции
  }
  const GnssFix & pos_last = gnss_buffer_.back();

  // Смещения антенн вдоль продольной оси base_link (x вперёд, REP 103).
  const double x_pos = antenna_x(position_antenna_);
  const double x_ori = antenna_x(orientation_antenna_);

  // 1) GNSS-компас: курс из базовой линии pos->ori (мгновенно, без ожидания
  //    движения), затем base_link = pos - x_pos·u и проекция base_link на карту.
  if (has_ori_fix_ && std::abs(x_ori - x_pos) > 1.0) {
    const GnssFix & ori = latest_ori_fix_;
    const double dt_s = std::abs(static_cast<double>(ori.stamp_ns - pos_last.stamp_ns) * 1e-9);
    if (dt_s <= gnss_match_tolerance_s_) {
      const Point3D pp = map_->to_enu(pos_last.lat, pos_last.lon, pos_last.alt);
      const Point3D po = map_->to_enu(ori.lat, ori.lon, ori.alt);
      RCLCPP_DEBUG(get_logger(), "compass: pp=(%.2f,%.2f) po=(%.2f,%.2f) dt=%.2fms",
        pp.x, pp.y, po.x, po.y, dt_s * 1000.0);
      const double sign = (x_ori > x_pos) ? 1.0 : -1.0;  // u — вперёд по трамваю
      const double dx = sign * (po.x - pp.x);
      const double dy = sign * (po.y - pp.y);
      const double base = std::hypot(dx, dy);
      // База жёсткая (~12.4 м): пара фиксов с другой длиной — шум, ждём следующую.
      if (base > 1.0 && std::abs(base - std::abs(x_ori - x_pos)) <= init_baseline_tol_m_) {
        const double ux = dx / base, uy = dy / base;
        // base_link — от антенны позиции; если у антенны ориентации фикс лучше
        // (RTK против автономного, разница — метры), то от неё.
        const bool use_ori = ori.status > pos_last.status;
        const double blx = use_ori ? (po.x - x_ori * ux) : (pp.x - x_pos * ux);
        const double bly = use_ori ? (po.y - x_ori * uy) : (pp.y - x_pos * uy);
        std::size_t rid = 0;
        double s0 = 0.0, ct = 0.0;
        if (!project_for_init(blx, bly, std::atan2(uy, ux), rid, s0, ct)) {
          RCLCPP_WARN(get_logger(), "GNSS init: project failed -> relative mode.");
          finish_map_init_unbound();
          return;
        }
        double hd = 0.0;
        Point3D dummy;
        map_->lookup(rid, s0, dummy, hd);
        const int dir = (std::cos(std::atan2(uy, ux) - hd) >= 0.0) ? 1 : -1;
        bind_map(rid, s0, dir, ct);
        RCLCPP_INFO(
          get_logger(),
          "Map init (compass): base_link s0=%.1f m (route %zu, cross %.2f m), dir=%+d, "
          "baseline=%.2f m (expected %.2f), status pos/ori=%d/%d.",
          s0, rid, ct, dir, base, std::abs(x_ori - x_pos), pos_last.status, ori.status);
        return;
      }
    }
  }

  // 2) Без компаса: направление из смещения антенны позиции за окно.
  const GnssFix & pos_first = gnss_buffer_.front();
  const double span_s = static_cast<double>(pos_last.stamp_ns - pos_first.stamp_ns) * 1e-9;
  if (span_s >= init_window_s_) {
    const Point3D p0 = map_->to_enu(pos_first.lat, pos_first.lon, pos_first.alt);
    const Point3D p1 = map_->to_enu(pos_last.lat, pos_last.lon, pos_last.alt);
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double disp = std::hypot(dx, dy);
    if (disp < min_init_displacement_m_) {
      return;  // слишком мало движения: ждём
    }
    // Трамвай едет кабиной вперёд: смещение антенны задаёт ось base_link.
    const double blx = p1.x - x_pos * dx / disp;
    const double bly = p1.y - x_pos * dy / disp;
    std::size_t rid = 0;
    double s0 = 0.0, ct = 0.0;
    if (!project_for_init(blx, bly, std::atan2(dy, dx), rid, s0, ct)) {
      RCLCPP_WARN(get_logger(), "GNSS init: project failed -> relative mode.");
      finish_map_init_unbound();
      return;
    }
    double hd = 0.0;
    Point3D dummy;
    map_->lookup(rid, s0, dummy, hd);
    const int dir = (dx * std::cos(hd) + dy * std::sin(hd) >= 0.0) ? 1 : -1;
    bind_map(rid, s0, dir, ct);
    RCLCPP_INFO(
      get_logger(),
      "Map init (fallback): s0=%.1f m (route %zu, cross %.2f m), dir=%+d, disp=%.2f m.",
      s0, rid, ct, dir, disp);
    return;
  }

  // 3) Таймаут -> относительный режим.
  if (timed_out) {
    RCLCPP_WARN(
      get_logger(), "GNSS init: timeout (%.1fs) -> relative mode.", init_timeout_s_);
    finish_map_init_unbound();
  }
}

}  // namespace tram
