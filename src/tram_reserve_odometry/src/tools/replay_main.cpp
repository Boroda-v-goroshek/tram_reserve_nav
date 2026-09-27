// Офлайн-прогон ноды по bag: та же нода и конфиг (share/configs/*.yaml), сообщения
// подаются в порядке записи, таймер публикации — по времени записи bag.
// Выход — CSV с тем, что нода опубликовала бы в /result/*:
//   stamp_s, velocity_mps, x, y, z, position_known, s_m, route_id, on_route,
//   direction, heading_rad, quality, tick_s
//
//   ros2 run tram_reserve_odometry reserve_odometry_replay --bag <bag> --out out.csv
//     [--no-driver 1]  без ручки (отказ канала ручки)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tram_vehicle_msgs/msg/velocity_sensor.hpp>

#include "tram_reserve_odometry/node/reserve_odometry_node.hpp"

namespace
{

struct Args
{
  std::string bag;
  std::string out;
  bool verbose = false;   ///< лог ноды уровня info (по умолчанию — только warn)
  bool no_driver = false; ///< без ручки: проверка отказа канала ручки
  // Инъекция отказа: обе тележки молчат drop_duration с каждые drop_period с,
  // начиная с drop_offset с от начала bag (0 — без инъекции).
  double drop_period = 0.0, drop_duration = 0.0, drop_offset = 0.0;
  // Редкий GNSS: фиксы проходят первые gnss_keep с (выставка),
  // дальше — только gnss_duration с каждые gnss_period с (0 — все фиксы проходят).
  double gnss_period = 0.0, gnss_duration = 0.0, gnss_keep = 0.0;
  bool gnss_alternate = false;  ///< в окне — только одна антенна, master и rover по очереди
};

/// Конфиг ноды: share/configs/*.yaml в алфавитном порядке.
std::vector<std::string> config_set()
{
  const std::filesystem::path base =
    ament_index_cpp::get_package_share_directory("tram_reserve_odometry") + "/configs";
  std::vector<std::string> files;
  for (const std::filesystem::directory_entry & e : std::filesystem::directory_iterator(base)) {
    if (e.path().extension() == ".yaml") {
      files.push_back(e.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) {
      out.push_back(item);
    }
  }
  return out;
}

bool parse_args(int argc, char ** argv, Args & a)
{
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--ros-args") {
      break;
    }
    if (i + 1 >= argc) {
      return false;
    }
    const std::string v = argv[++i];
    if (k == "--bag") {
      a.bag = v;
    } else if (k == "--out") {
      a.out = v;
    } else if (k == "--verbose") {
      a.verbose = (v == "1" || v == "true");
    } else if (k == "--no-driver") {
      a.no_driver = (v == "1" || v == "true");
    } else if (k == "--drop-bogies") {
      const auto f = split_csv(v);
      if (f.size() != 3) {
        return false;
      }
      a.drop_period = std::stod(f[0]);
      a.drop_duration = std::stod(f[1]);
      a.drop_offset = std::stod(f[2]);
    } else if (k == "--gnss-windows") {
      const auto f = split_csv(v);
      if (f.size() != 3 && f.size() != 4) {
        return false;
      }
      a.gnss_period = std::stod(f[0]);
      a.gnss_duration = std::stod(f[1]);
      a.gnss_keep = std::stod(f[2]);
      a.gnss_alternate = f.size() == 4 && std::stod(f[3]) > 0.5;
    } else {
      return false;
    }
  }
  return !a.bag.empty() && !a.out.empty();
}

template<typename T>
T deserialize(const rosbag2_storage::SerializedBagMessage & bm)
{
  static rclcpp::Serialization<T> ser;
  rclcpp::SerializedMessage raw(*bm.serialized_data);
  T msg;
  ser.deserialize_message(&raw, &msg);
  return msg;
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  if (!parse_args(argc, argv, args)) {
    std::fprintf(
      stderr,
      "usage: reserve_odometry_replay --bag <bag_dir> --out <file.csv> "
      "[--verbose 1] [--no-driver 1] [--drop-bogies PERIOD,DURATION,OFFSET]\n"
      "  [--gnss-windows PERIOD,DURATION,KEEP]  GNSS only first KEEP s, then DURATION s every PERIOD s\n"
      "      (4th field ALT=1: one antenna per window, master/rover alternating)\n"
      "config: share/configs/*.yaml\n");
    return 2;
  }
  std::vector<std::string> params_files;
  try {
    params_files = config_set();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 2;
  }

  // Свой DDS-домен и только localhost: публикации replay не попадают в живую сеть ROS.
  setenv("ROS_DOMAIN_ID", "101", 1);
  setenv("ROS_LOCALHOST_ONLY", "1", 1);
  rclcpp::init(argc, argv);
  std::vector<std::string> ros_args{"--ros-args"};
  if (!args.verbose) {
    ros_args.insert(ros_args.end(), {"--log-level", "reserve_odometry:=warn"});
  }
  for (const auto & f : params_files) {
    ros_args.push_back("--params-file");
    ros_args.push_back(f);
  }
  auto node = std::make_shared<tram::ReserveOdometryNode>(
    rclcpp::NodeOptions().arguments(ros_args));
  if (!node->ready()) {
    std::fprintf(stderr, "node is not ready (plugins failed to load)\n");
    rclcpp::shutdown();
    return 1;
  }

  std::FILE * out = std::fopen(args.out.c_str(), "w");
  if (!out) {
    std::fprintf(stderr, "cannot open %s\n", args.out.c_str());
    rclcpp::shutdown();
    return 1;
  }
  std::fprintf(
    out, "stamp_s,velocity_mps,x,y,z,position_known,s_m,route_id,on_route,direction,"
    "heading_rad,quality,tick_s,model_mode\n");

  int64_t tick_ns = 0;  // время записи текущего такта (для CSV и как «сейчас» ноды)
  node->set_now_source([&tick_ns] {return tick_ns;});
  node->set_output_hook(
    [out, &tick_ns](const tram::Estimate & e, bool position_known) {
      std::fprintf(
        out, "%.9f,%.6f,%.4f,%.4f,%.4f,%d,%.4f,%zu,%d,%d,%.6f,%d,%.9f,%d\n",
        static_cast<double>(e.stamp_ns) * 1e-9, e.velocity,
        e.position.x, e.position.y, e.position.z, position_known ? 1 : 0,
        e.s, e.route_id, e.on_route ? 1 : 0, e.direction, e.heading,
        static_cast<int>(e.quality), static_cast<double>(tick_ns) * 1e-9, e.model_mode);
    });

  const std::string gnss_master = "/sensing/gnss/master/fix";
  const std::string gnss_rover = "/sensing/gnss/rover/fix";

  rosbag2_cpp::Reader reader;
  reader.open(args.bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {node->front_topic(), node->rear_topic(), gnss_master, gnss_rover};
  if (!args.no_driver) {
    filter.topics.push_back(node->driver_topic());
  }
  reader.set_filter(filter);

  const int64_t period_ns = static_cast<int64_t>(1e9 / node->output_rate_hz());
  int64_t next_tick_ns = 0;
  int64_t first_rec_ns = 0;
  std::size_t n_msgs = 0, n_dropped = 0, n_gnss_dropped = 0;
  while (reader.has_next()) {
    const auto bm = reader.read_next();
    const int64_t t_rec = bm->time_stamp;
    if (next_tick_ns == 0) {
      next_tick_ns = t_rec + period_ns;
      first_rec_ns = t_rec;
    }
    // Такты стенного таймера, наступившие до прихода этого сообщения.
    while (next_tick_ns <= t_rec) {
      tick_ns = next_tick_ns;
      node->on_timer();
      next_tick_ns += period_ns;
    }
    const std::string & topic = bm->topic_name;
    if (topic == node->driver_topic()) {
      tram::DriverCommand cmd;
      if (tram::decode_driver_command(
          bm->serialized_data->buffer, bm->serialized_data->buffer_length, cmd))
      {
        node->ingest_input(topic, cmd, t_rec);
      }
    } else if (topic == node->front_topic() || topic == node->rear_topic()) {
      const double el = static_cast<double>(t_rec - first_rec_ns) * 1e-9 - args.drop_offset;
      if (args.drop_period > 0.0 && el >= 0.0 && std::fmod(el, args.drop_period) < args.drop_duration) {
        ++n_dropped;  // инъекция: тележки молчат
        continue;
      }
      node->ingest_input(topic, deserialize<tram_vehicle_msgs::msg::VelocitySensor>(*bm), t_rec);
    } else {
      const double el = static_cast<double>(t_rec - first_rec_ns) * 1e-9 - args.gnss_keep;
      if (args.gnss_period > 0.0 && el >= 0.0) {
        const bool in_window = std::fmod(el, args.gnss_period) < args.gnss_duration;
        const bool rover_window = static_cast<long>(el / args.gnss_period) % 2 == 1;
        const bool wrong_antenna = args.gnss_alternate &&
          ((topic == gnss_rover) != rover_window);
        if (!in_window || wrong_antenna) {
          ++n_gnss_dropped;  // редкий GNSS: вне окна (или не та антенна) фикса нет
          continue;
        }
      }
      node->ingest_gnss_fix(topic, deserialize<sensor_msgs::msg::NavSatFix>(*bm));
    }
    ++n_msgs;
  }
  // Последний такт после конца bag: публикует хвост входов.
  tick_ns = next_tick_ns;
  node->on_timer();

  std::fclose(out);
  std::fprintf(stderr, "replayed %zu messages (bogie messages dropped: %zu, GNSS dropped: %zu) from %s -> %s\n",
    n_msgs, n_dropped, n_gnss_dropped, args.bag.c_str(), args.out.c_str());
  node.reset();
  rclcpp::shutdown();
  return 0;
}
