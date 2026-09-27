// 1D-карта маршрута: замкнутое кольцо (route 0) + тупиковые ветки (routes 1..N).
// Положение — скаляр s вдоль полилинии; s -> (x, y, z, курс, уклон, кривизна) —
// интерполяция; обратная операция — проекция точки на путь.

#ifndef TRAM_RESERVE_ODOMETRY__MAP__ROUTE_MAP_HPP_
#define TRAM_RESERVE_ODOMETRY__MAP__ROUTE_MAP_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

/// Один маршрут (полилиния) в 1D-карте.
struct Route
{
  std::string name;
  std::vector<Point3D> pts;   ///< точки, отсортированы по s, равномерный шаг ds
  std::vector<double> heading;  ///< рад, на каждой точке
  std::vector<double> grade;    ///< уклон (м/м), на каждой точке
  std::vector<double> curvature;///< 1/м, на каждой точке
  double ds = 2.0;              ///< шаг по s (м)
  double length = 0.0;          ///< длина (м)
  bool closed = false;          ///< кольцо: s замыкается (mod length)
  double join_s_ring = -1.0;    ///< для веток: точка слияния с кольцом (s кольца)
  std::vector<double> stops;    ///< опорные остановки (s вдоль этого маршрута)
};

/// 1D-карта маршрута.
class RouteMap
{
public:
  RouteMap() = default;

  /// Загрузить кольцо из CSV (s_m,lat,lon,alt,x,y,heading,grade,curvature).
  /// Первая строка может быть комментарием (#). Кольцо становится route 0.
  bool load_ring(const std::string & csv_path);

  /// Загрузить ветки из CSV (branch,s_m,lat,lon,x,y,join_s_ring,runs).
  /// Каждая уникальная ветка становится отдельным route (1..N).
  bool load_branches(const std::string & csv_path);

  /// Загрузить опорные остановки из YAML (A: [...], B: [...]) и привязать к кольцу.
  bool load_stops(const std::string & yaml_path);

  bool has_routes() const {return !routes_.empty();}
  std::size_t num_routes() const {return routes_.size();}
  double ring_length() const {return routes_.empty() ? 0.0 : routes_[0].length;}

  /// Интерполяция по (route_id, s) -> (позиция, курс[, уклон, кривизна]).
  /// Для кольца s приводится к [0, L); для веток сжимается к [0, length].
  bool lookup(
    std::size_t route_id, double s,
    Point3D & pos, double & heading,
    double * grade = nullptr, double * curvature = nullptr) const;

  /// Ближайшая точка на подсказанном маршруте (локальный поиск вокруг hint_s).
  /// Возвращает (route_id, s, cross_track). cross_track >= 0.
  bool project(
    double x, double y,
    std::size_t hint_route, double hint_s,
    std::size_t & route_id, double & s, double & cross_track) const;

  /// Глобальная проекция: ближайшая точка по всем маршрутам.
  bool project_global(
    double x, double y,
    std::size_t & route_id, double & s, double & cross_track) const;

  /// Глобальная проекция с учётом курса: только сегменты, направление которых (+s)
  /// отличается от heading не больше чем на max_dheading (рад). false — таких нет.
  bool project_global_heading(
    double x, double y, double heading, double max_dheading,
    std::size_t & route_id, double & s, double & cross_track) const;

  /// Опорные остановки кольца (route 0).
  const std::vector<double> & stops() const {
    return routes_.empty() ? empty_ : routes_[0].stops;
  }

  /// lat/lon/alt -> ENU карты.
  Point3D to_enu(double lat, double lon, double alt) const;
  void set_reference(double lat0, double lon0, double alt0);
  double ref_lat() const {return lat0_;}
  double ref_lon() const {return lon0_;}
  double ref_alt() const {return alt0_;}

  const Route & route(std::size_t id) const {return routes_.at(id);}

private:
  /// Проекция (x, y) на сегмент (p0, p1): t в [0,1], dist, s проекции.
  static void project_segment(
    const Point3D & p0, const Point3D & p1, double x, double y,
    double & t, double & dist, double & s_proj);

  /// Лучшая проекция на весь маршрут (по сегментам).
  bool project_route(
    const Route & r, double x, double y,
    double s_hint, double window_m,
    double & s, double & cross_track) const;

  std::vector<Route> routes_;
  double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;
  double track_b_start_ = 0.0;  ///< s начала пути B на кольце (для привязки остановок)
  const std::vector<double> empty_;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__MAP__ROUTE_MAP_HPP_
