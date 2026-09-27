
#include "tram_reserve_odometry/map/route_map.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>

#include "tram_reserve_odometry/core/geo.hpp"

namespace tram
{

namespace
{

/// Интерполяция угла с учётом wrap-around (±π).
double interp_angle(double a0, double a1, double f)
{
  double d = a1 - a0;
  while (d > M_PI) {d -= 2.0 * M_PI;}
  while (d < -M_PI) {d += 2.0 * M_PI;}
  return a0 + d * f;
}

/// Разбить строку по запятой.
std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

bool to_double(const std::string & s, double & out)
{
  try {
    size_t pos = 0;
    out = std::stod(s, &pos);
    return pos > 0;
  } catch (const std::exception &) {
    return false;
  }
}

/// Извлечь первое число после подстроки `marker` (для парсинга комментария кольца).
bool number_after(const std::string & text, const std::string & marker, double & out)
{
  size_t p = text.find(marker);
  if (p == std::string::npos) {
    return false;
  }
  p += marker.size();
  while (p < text.size() && (text[p] == ' ' || text[p] == '[' || text[p] == '=')) {
    ++p;
  }
  return to_double(text.substr(p), out);
}

}  // namespace

void RouteMap::set_reference(double lat0, double lon0, double alt0)
{
  lat0_ = lat0;
  lon0_ = lon0;
  alt0_ = alt0;
}

Point3D RouteMap::to_enu(double lat, double lon, double alt) const
{
  return geo::geodetic_to_enu(lat, lon, alt, geo::EnuReference{lat0_, lon0_, alt0_});
}

bool RouteMap::load_ring(const std::string & csv_path)
{
  std::ifstream f(csv_path);
  if (!f) {
    return false;
  }
  Route r;
  r.name = "ring";
  r.closed = true;
  double s_b0 = 0.0;
  std::vector<double> s_col;
  std::string line;
  bool header_seen = false;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      // комментарий: пробуем вытащить начало пути B (для привязки остановок)
      if (line[0] == '#') {
        (void)number_after(line, "track B: s in [", s_b0);
      }
      continue;
    }
    if (!header_seen) {
      header_seen = true;  // строка заголовка
      continue;
    }
    auto cols = split_csv(line);
    if (cols.size() < 9) {
      continue;
    }
    double s, lat, lon, alt, x, y, hd, grade, kappa;
    if (!to_double(cols[0], s) || !to_double(cols[1], lat) || !to_double(cols[2], lon) ||
      !to_double(cols[3], alt) || !to_double(cols[4], x) || !to_double(cols[5], y) ||
      !to_double(cols[6], hd) || !to_double(cols[7], grade) || !to_double(cols[8], kappa))
    {
      continue;
    }
    s_col.push_back(s);
    r.pts.push_back(Point3D{x, y, alt});
    r.heading.push_back(hd);
    r.grade.push_back(grade);
    r.curvature.push_back(kappa);
  }
  if (r.pts.size() < 2) {
    return false;
  }
  // Шаг ds и длина — из колонки s_m (равномерная сетка по дуге).
  const std::size_t n = r.pts.size();
  r.ds = (s_col[n - 1] - s_col[0]) / static_cast<double>(n - 1);
  if (!(r.ds > 0.0)) {
    r.ds = 2.0;
  }
  // Длина кольца: s последней точки + замыкающий сегмент (последняя -> первая).
  r.length = (s_col[n - 1] - s_col[0]) +
    std::hypot(r.pts[0].x - r.pts[n - 1].x, r.pts[0].y - r.pts[n - 1].y);
  r.join_s_ring = -1.0;
  routes_.push_back(std::move(r));
  // храним s_b0 для привязки остановок пути B
  track_b_start_ = s_b0;
  return true;
}

bool RouteMap::load_branches(const std::string & csv_path)
{
  std::ifstream f(csv_path);
  if (!f) {
    return false;
  }
  // группировка по id ветки: id -> [(x, y)], id -> join_s_ring
  std::map<int, std::vector<std::pair<double, double>>> pts;
  std::map<int, double> join;
  std::string line;
  bool header_seen = false;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    if (!header_seen) {
      header_seen = true;
      continue;
    }
    auto cols = split_csv(line);
    if (cols.size() < 7) {
      continue;
    }
    double id_d = 0.0;
    double s, lat, lon, x, y, js;
    if (!to_double(cols[0], id_d) || !to_double(cols[1], s) ||
      !to_double(cols[2], lat) || !to_double(cols[3], lon) || !to_double(cols[4], x) ||
      !to_double(cols[5], y) || !to_double(cols[6], js))
    {
      continue;
    }
    const int id = static_cast<int>(id_d);
    pts[id].push_back({x, y});
    join[id] = js;
  }
  bool any = false;
  for (const auto & kv : pts) {
    if (kv.second.size() < 2) {
      continue;
    }
    any = true;
    Route r;
    r.name = "branch_" + std::to_string(kv.first);
    r.closed = false;
    r.join_s_ring = join[kv.first];
    // В CSV веток нет высоты: берём высоту рельса кольца в точке слияния.
    double z_join = 0.0;
    if (!routes_.empty() && routes_[0].closed) {
      Point3D pj;
      double hj = 0.0;
      if (lookup(0, r.join_s_ring, pj, hj)) {
        z_join = pj.z;
      }
    }
    const auto & v = kv.second;
    for (const auto & p : v) {
      r.pts.push_back(Point3D{p.first, p.second, z_join});
    }
    const std::size_t n = v.size();
    r.ds = 2.0;
    r.length = (n > 1) ? (n - 1) * r.ds : 0.0;
    r.heading.resize(n);
    r.grade.assign(n, 0.0);
    r.curvature.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t i1 = (i + 1 < n) ? i + 1 : i;
      const double dx = r.pts[i1].x - r.pts[i].x;
      const double dy = r.pts[i1].y - r.pts[i].y;
      r.heading[i] = (std::hypot(dx, dy) > 1e-9) ? std::atan2(dy, dx) :
        (i > 0 ? r.heading[i - 1] : 0.0);
    }
    routes_.push_back(std::move(r));
  }
  return any;
}

bool RouteMap::load_stops(const std::string & yaml_path)
{
  if (routes_.empty()) {
    return false;
  }
  std::ifstream f(yaml_path);
  if (!f) {
    return false;
  }
  std::vector<double> stops_a, stops_b;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    auto pos = line.find(':');
    if (pos == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, pos);
    const std::string body = line.substr(pos + 1);
    std::vector<double> vals;
    {
      std::stringstream ss(body);
      std::string item;
      while (std::getline(ss, item, ',')) {
        // убираем скобки/пробелы
        std::string clean;
        for (char c : item) {
          if (c != '[' && c != ']' && c != ' ' && c != '\r') {
            clean += c;
          }
        }
        double v;
        if (to_double(clean, v)) {
          vals.push_back(v);
        }
      }
    }
    if (key == "A") {
      stops_a = vals;
    } else if (key == "B") {
      stops_b = vals;
    }
  }
  // Привязка к кольцу: A -> как есть, B -> со сдвигом на track_b_start_.
  // Остановки вне [0, L) отбрасываем.
  const double L = routes_[0].length;
  std::vector<double> ring_stops;
  for (double s : stops_a) {
    if (s >= 0.0 && s < L) {
      ring_stops.push_back(s);
    }
  }
  for (double s : stops_b) {
    const double sr = s + track_b_start_;
    if (sr >= 0.0 && sr < L) {
      ring_stops.push_back(sr);
    }
  }
  std::sort(ring_stops.begin(), ring_stops.end());
  routes_[0].stops = ring_stops;
  return true;
}

void RouteMap::project_segment(
  const Point3D & p0, const Point3D & p1, double x, double y,
  double & t, double & dist, double & s_proj)
{
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) {
    t = 0.0;
    dist = std::hypot(x - p0.x, y - p0.y);
    s_proj = 0.0;
    return;
  }
  t = ((x - p0.x) * dx + (y - p0.y) * dy) / len2;
  if (t < 0.0) {t = 0.0;}
  if (t > 1.0) {t = 1.0;}
  const double px = p0.x + t * dx;
  const double py = p0.y + t * dy;
  dist = std::hypot(x - px, y - py);
  s_proj = t * std::sqrt(len2);
}

bool RouteMap::project_route(
  const Route & r, double x, double y,
  double s_hint, double window_m,
  double & s, double & cross_track) const
{
  const std::size_t n = r.pts.size();
  if (n < 2) {
    return false;
  }
  // диапазон индексов сегментов для поиска
  // Кольцо: n сегментов (включая замыкающий); ветка: n-1, без замыкания.
  const int n_seg = r.closed ? static_cast<int>(n) : static_cast<int>(n) - 1;
  int i_start = 0;
  int i_end = n_seg;
  if (window_m > 0.0 && s_hint >= 0.0) {
    const int i0 = static_cast<int>(std::floor(s_hint / r.ds));
    const int w = static_cast<int>(std::ceil(window_m / r.ds)) + 1;
    i_start = i0 - w;
    i_end = i0 + w + 1;
    if (!r.closed) {
      i_start = std::max(i_start, 0);
      i_end = std::min(i_end, n_seg);
    }
  }
  double best_dist = 1e30;
  double best_s = 0.0;
  for (int i = i_start; i < i_end; ++i) {
    const int ii = r.closed ?
      ((i % static_cast<int>(n)) + static_cast<int>(n)) % static_cast<int>(n) : i;
    const std::size_t i1 = (static_cast<std::size_t>(ii) + 1) % n;
    double t, dist, s_proj;
    project_segment(r.pts[ii], r.pts[i1], x, y, t, dist, s_proj);
    double s_i = static_cast<double>(ii) * r.ds;
    double s_candidate = s_i + s_proj;
    if (r.closed && s_candidate > r.length) {
      s_candidate = r.length;
    }
    if (dist < best_dist) {
      best_dist = dist;
      best_s = s_candidate;
    }
  }
  s = best_s;
  cross_track = best_dist;
  return true;
}

bool RouteMap::lookup(
  std::size_t route_id, double s,
  Point3D & pos, double & heading,
  double * grade, double * curvature) const
{
  if (route_id >= routes_.size()) {
    return false;
  }
  const Route & r = routes_[route_id];
  const std::size_t n = r.pts.size();
  if (n < 2) {
    return false;
  }
  double sn = s;
  if (r.closed) {
    sn = std::fmod(s, r.length);
    if (sn < 0.0) {
      sn += r.length;
    }
  } else {
    if (sn < 0.0) {sn = 0.0;}
    if (sn > r.length) {sn = r.length;}
  }
  double idx = sn / r.ds;
  std::size_t i0 = static_cast<std::size_t>(std::floor(idx));
  if (i0 >= n) {
    i0 = n - 1;
  }
  std::size_t i1;
  double f;
  if (r.closed && i0 == n - 1) {
    i1 = 0;
    const double seg = r.length - (n - 1) * r.ds;
    f = (seg > 1e-9) ? (sn - (n - 1) * r.ds) / seg : 0.0;
  } else {
    i1 = (i0 + 1) % n;
    f = idx - static_cast<double>(i0);
  }
  if (f < 0.0) {f = 0.0;}
  if (f > 1.0) {f = 1.0;}
  pos.x = r.pts[i0].x * (1.0 - f) + r.pts[i1].x * f;
  pos.y = r.pts[i0].y * (1.0 - f) + r.pts[i1].y * f;
  pos.z = r.pts[i0].z * (1.0 - f) + r.pts[i1].z * f;  // высота рельса (ENU)
  heading = interp_angle(r.heading[i0], r.heading[i1], f);
  if (grade != nullptr) {
    *grade = r.grade[i0] * (1.0 - f) + r.grade[i1] * f;
  }
  if (curvature != nullptr) {
    *curvature = r.curvature[i0] * (1.0 - f) + r.curvature[i1] * f;
  }
  return true;
}

bool RouteMap::project(
  double x, double y,
  std::size_t hint_route, double hint_s,
  std::size_t & route_id, double & s, double & cross_track) const
{
  if (hint_route >= routes_.size()) {
    return false;
  }
  const double window_m = 60.0;  // локальное окно ±60 м вокруг подсказки
  if (!project_route(routes_[hint_route], x, y, hint_s, window_m, s, cross_track)) {
    return false;
  }
  route_id = hint_route;
  return true;
}

bool RouteMap::project_global(
  double x, double y,
  std::size_t & route_id, double & s, double & cross_track) const
{
  double best_dist = 1e30;
  bool found = false;
  for (std::size_t i = 0; i < routes_.size(); ++i) {
    double s_i, ct;
    if (project_route(routes_[i], x, y, -1.0, 0.0, s_i, ct)) {
      if (ct < best_dist) {
        best_dist = ct;
        s = s_i;
        route_id = i;
        found = true;
      }
    }
  }
  if (found) {
    cross_track = best_dist;
  }
  return found;
}

bool RouteMap::project_global_heading(
  double x, double y, double heading, double max_dheading,
  std::size_t & route_id, double & s, double & cross_track) const
{
  const double cos_max = std::cos(max_dheading);
  const double hx = std::cos(heading), hy = std::sin(heading);
  double best_dist = 1e30;
  bool found = false;
  for (std::size_t k = 0; k < routes_.size(); ++k) {
    const Route & r = routes_[k];
    const std::size_t n = r.pts.size();
    if (n < 2) {
      continue;
    }
    const std::size_t n_seg = r.closed ? n : n - 1;
    for (std::size_t i = 0; i < n_seg; ++i) {
      const std::size_t i1 = (i + 1) % n;
      const double dx = r.pts[i1].x - r.pts[i].x;
      const double dy = r.pts[i1].y - r.pts[i].y;
      const double len = std::hypot(dx, dy);
      if (len < 1e-9 || (dx * hx + dy * hy) / len < cos_max) {
        continue;  // сегмент смотрит не туда, куда едет трамвай
      }
      double t, dist, s_proj;
      project_segment(r.pts[i], r.pts[i1], x, y, t, dist, s_proj);
      if (dist < best_dist) {
        best_dist = dist;
        s = std::min(static_cast<double>(i) * r.ds + s_proj, r.length);
        route_id = k;
        found = true;
      }
    }
  }
  if (found) {
    cross_track = best_dist;
  }
  return found;
}

}  // namespace tram
