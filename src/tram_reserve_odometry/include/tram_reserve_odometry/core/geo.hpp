// Геодезия без внешних зависимостей:
//   - локальная ENU-плоскость <-> WGS84 lat/lon/alt;
//   - поперечный Меркатор / UTM (ряды Снайдера, USGS PP 1395);
//   - OutputProjector: ENU -> выходная система (MGRS: x = E − origin_x, y = N − origin_y;
//     ENU; относительно старта).

#ifndef TRAM_RESERVE_ODOMETRY__CORE__GEO_HPP_
#define TRAM_RESERVE_ODOMETRY__CORE__GEO_HPP_

#include <cmath>
#include <string>

#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{
namespace geo
{

constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 6.69437999014e-3;
constexpr double kUtmK0 = 0.9996;
constexpr double kUtmFalseEasting = 500000.0;

inline double deg2rad(double d) {return d * M_PI / 180.0;}
inline double rad2deg(double r) {return r * 180.0 / M_PI;}

/// Опорная точка локальной ENU-плоскости (lat, lon в градусах, alt в метрах).
struct EnuReference
{
  double lat0 = 0.0;
  double lon0 = 0.0;
  double alt0 = 0.0;
};

/// Радиусы кривизны WGS84 в опорной широте: RN (первый вертикал), RM (меридиан).
inline void curvature_radii(double lat0_deg, double & rn, double & rm)
{
  const double s = std::sin(deg2rad(lat0_deg));
  const double denom = 1.0 - kWgs84E2 * s * s;
  rn = kWgs84A / std::sqrt(denom);
  rm = kWgs84A * (1.0 - kWgs84E2) / (denom * std::sqrt(denom));
}

/// lat/lon/alt -> ENU (касательная плоскость с радиусами WGS84 в опорной точке).
inline Point3D geodetic_to_enu(double lat, double lon, double alt, const EnuReference & ref)
{
  double rn = 0.0, rm = 0.0;
  curvature_radii(ref.lat0, rn, rm);
  Point3D p;
  p.x = deg2rad(lon - ref.lon0) * (rn + ref.alt0) * std::cos(deg2rad(ref.lat0));
  p.y = deg2rad(lat - ref.lat0) * (rm + ref.alt0);
  p.z = alt - ref.alt0;
  return p;
}

/// Обратное к geodetic_to_enu (точно, та же модель).
inline void enu_to_geodetic(
  const Point3D & p, const EnuReference & ref, double & lat, double & lon, double & alt)
{
  double rn = 0.0, rm = 0.0;
  curvature_radii(ref.lat0, rn, rm);
  lat = ref.lat0 + rad2deg(p.y / (rm + ref.alt0));
  lon = ref.lon0 + rad2deg(p.x / ((rn + ref.alt0) * std::cos(deg2rad(ref.lat0))));
  alt = p.z + ref.alt0;
}

/// Поперечный Меркатор (Снайдер, формулы 8-9..8-10), без false easting/northing.
/// \return (x = восток, y = север) в метрах.
inline Point2D transverse_mercator(
  double lat, double lon, double lon0, double k0,
  double a = kWgs84A, double e2 = kWgs84E2)
{
  const double phi = deg2rad(lat);
  const double ep2 = e2 / (1.0 - e2);
  const double sp = std::sin(phi), cp = std::cos(phi), tp = std::tan(phi);
  const double N = a / std::sqrt(1.0 - e2 * sp * sp);
  const double T = tp * tp;
  const double C = ep2 * cp * cp;
  const double A = deg2rad(lon - lon0) * cp;
  const double e4 = e2 * e2, e6 = e4 * e2;
  const double M = a * (
    (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi -
    (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * std::sin(2.0 * phi) +
    (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * phi) -
    (35.0 * e6 / 3072.0) * std::sin(6.0 * phi));
  const double A2 = A * A, A3 = A2 * A, A4 = A3 * A, A5 = A4 * A, A6 = A5 * A;
  Point2D out;
  out.x = k0 * N * (A + (1.0 - T + C) * A3 / 6.0 +
    (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0);
  out.y = k0 * (M + N * tp * (A2 / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0 +
    (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0));
  return out;
}

/// Центральный меридиан зоны UTM (1..60).
inline double utm_central_meridian(int zone) {return zone * 6.0 - 183.0;}

/// lat/lon -> UTM (E с false easting 500 км, N для северного полушария).
inline Point2D geodetic_to_utm(double lat, double lon, int zone)
{
  Point2D p = transverse_mercator(lat, lon, utm_central_meridian(zone), kUtmK0);
  p.x += kUtmFalseEasting;
  return p;
}

/// Выходная система координат позиции.
enum class OutputFrame { ENU, MGRS, RELATIVE };

/// Настройки OutputProjector.
struct OutputProjectorConfig
{
  OutputFrame frame = OutputFrame::MGRS;
  EnuReference enu_ref;          ///< опорная точка внутренней ENU (= map.reference)
  int utm_zone = 37;
  double mgrs_origin_x = 300000.0;   ///< угол 100-км квадрата по востоку (UTM E)
  double mgrs_origin_y = 6100000.0;  ///< угол 100-км квадрата по северу (UTM N)
};

/// Перевод внутренней позиции (ENU карты) в выходную систему координат.
class OutputProjector
{
public:
  using Frame = OutputFrame;
  using Config = OutputProjectorConfig;

  /// "mgrs" | "enu" | "relative" -> Frame; false, если строка не распознана.
  static bool parse_frame(const std::string & s, Frame & out)
  {
    if (s == "mgrs") {out = Frame::MGRS; return true;}
    if (s == "enu") {out = Frame::ENU; return true;}
    if (s == "relative") {out = Frame::RELATIVE; return true;}
    return false;
  }

  OutputProjector() = default;
  explicit OutputProjector(const Config & cfg) : cfg_(cfg) {}

  const Config & config() const {return cfg_;}

  /// ENU -> выходная система. z: MGRS — абсолютная высота (alt WGS84),
  /// ENU — относительно опорной, RELATIVE — относительно первой точки.
  Point3D project(const Point3D & enu)
  {
    switch (cfg_.frame) {
      case Frame::ENU:
        return enu;
      case Frame::RELATIVE:
        if (!has_origin_) {
          origin_ = enu;
          has_origin_ = true;
        }
        return Point3D{enu.x - origin_.x, enu.y - origin_.y, enu.z - origin_.z};
      case Frame::MGRS:
      default:
        return to_mgrs(enu);
    }
  }

  /// Курс (рад, от оси x против часовой) в выходной системе; для MGRS — с учётом
  /// сближения меридианов.
  double project_heading(const Point3D & enu, double heading) const
  {
    if (cfg_.frame != Frame::MGRS) {
      return heading;
    }
    const Point3D a = to_mgrs(enu);
    const Point3D b = to_mgrs(
      Point3D{enu.x + std::cos(heading), enu.y + std::sin(heading), enu.z});
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  void reset() {has_origin_ = false;}

private:
  Point3D to_mgrs(const Point3D & enu) const
  {
    double lat = 0.0, lon = 0.0, alt = 0.0;
    enu_to_geodetic(enu, cfg_.enu_ref, lat, lon, alt);
    const Point2D u = geodetic_to_utm(lat, lon, cfg_.utm_zone);
    return Point3D{u.x - cfg_.mgrs_origin_x, u.y - cfg_.mgrs_origin_y, alt};
  }

  Config cfg_;
  Point3D origin_{};
  bool has_origin_ = false;
};

}  // namespace geo
}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__CORE__GEO_HPP_
