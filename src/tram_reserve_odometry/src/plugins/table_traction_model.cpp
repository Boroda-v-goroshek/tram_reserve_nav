#include "tram_reserve_odometry/plugins/table_traction_model.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

namespace tram
{

namespace
{
std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}
}  // namespace

bool TableTractionModel::load(const std::string & path)
{
  tried_ = true;
  notch_.clear();
  speed_.clear();
  f_.clear();
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  std::string line;
  bool header = false;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto cols = split(line);
    if (!header) {
      // u,v0,v1,v2,v4,...  -> узлы скорости из имён столбцов
      for (std::size_t j = 1; j < cols.size(); ++j) {
        speed_.push_back(std::stod(cols[j].substr(1)));
      }
      header = true;
      continue;
    }
    if (cols.size() != speed_.size() + 1) {
      continue;
    }
    notch_.push_back(std::stod(cols[0]));
    std::vector<double> row;
    for (std::size_t j = 1; j < cols.size(); ++j) {
      row.push_back(std::stod(cols[j]));
    }
    f_.push_back(row);
  }
  if (speed_.size() < 2 || notch_.empty()) {
    notch_.clear();
    return false;
  }
  return true;
}

void TableTractionModel::ensure_loaded(const ModelParams & params) const
{
  if (tried_) {
    return;
  }
  const auto it = params.extra_str.find("drv_table_file");
  if (it != params.extra_str.end()) {
    const_cast<TableTractionModel *>(this)->load(it->second);
  }
  tried_ = true;
}

bool TableTractionModel::has_drive_model(const ModelParams & params) const
{
  ensure_loaded(params);
  return loaded();
}

double TableTractionModel::drive_force(double notch, double v, const ModelParams & params) const
{
  ensure_loaded(params);
  if (!loaded()) {
    return 0.0;
  }
  // Ближайшая позиция ручки (позиции целые).
  std::size_t i = 0;
  for (std::size_t k = 1; k < notch_.size(); ++k) {
    if (std::abs(notch_[k] - notch) < std::abs(notch_[i] - notch)) {
      i = k;
    }
  }
  const std::vector<double> & row = f_[i];
  if (v <= speed_.front()) {
    return row.front();
  }
  if (v >= speed_.back()) {
    const auto it = params.extra.find("drv_power_extrap");
    const bool power = it != params.extra.end() && it->second > 0.5;
    if (power && row.back() > 0.0 && speed_.back() > 0.0) {
      return row.back() * speed_.back() / v;  // постоянная мощность: a·v = const
    }
    return row.back();
  }
  const std::size_t j = static_cast<std::size_t>(
    std::upper_bound(speed_.begin(), speed_.end(), v) - speed_.begin()) - 1;
  const double w = (v - speed_[j]) / (speed_[j + 1] - speed_[j]);
  return row[j] * (1.0 - w) + row[j + 1] * w;
}

double TableTractionModel::compute_acceleration(
  const PreprocessedSample & sample, const ModelParams & params) const
{
  const double notch = (params.driver_scale > 0.0) ?
    std::round(sample.driver_position_cmd / params.driver_scale) : 0.0;
  const double v = std::max(0.5 * (sample.front_bogie_velocity + sample.rear_bogie_velocity), 0.0);
  const auto it = params.extra.find("drv_w0");
  const double w0 = (it == params.extra.end()) ? 0.046 : it->second;
  return drive_force(notch, v, params) - w0 * std::tanh(v / 0.2);
}

}  // namespace tram

PLUGINLIB_EXPORT_CLASS(tram::TableTractionModel, tram::ITractionModel)
