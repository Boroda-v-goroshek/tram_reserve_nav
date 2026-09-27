// Интегратор положения по 1D-карте маршрута: маршрут route_id, s и направление;
// позиция и курс — с карты.
//   - с картой: s = s0 + direction·∫v dt, position = lookup(route_id, s), on_route = true;
//   - без карты: s = ∫v dt от 0, position = (s, 0), on_route = false.
// Кольцо (route 0) — s mod L; за концом ветки — переход на кольцо в join_s_ring,
// направление по кольцу — по совпадению курсов. Съезд с кольца на ветку не отслеживается.

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__ROUTE_INTEGRATOR_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__ROUTE_INTEGRATOR_HPP_

#include <cmath>
#include <cstddef>
#include <memory>

#include "tram_reserve_odometry/core/interfaces.hpp"
#include "tram_reserve_odometry/map/route_map.hpp"

namespace tram
{

class RouteIntegrator : public IPositionIntegrator
{
public:
  RouteIntegrator() = default;

  void set_route_map(const std::shared_ptr<const RouteMap> & m) override {map_ = m;}
  void set_initial_s(double s0) override {s_ = s0;}
  void set_direction(int d) override {direction_ = (d < 0) ? -1 : 1;}
  void set_route_id(std::size_t id) override {route_ = id;}
  void correct_s(double s, double) override {s_ = on_ring() ? wrap_ring(s) : s;}

  void integrate(Estimate & est, double dt, const ModelParams &) override
  {
    // При dt <= 0 s не продвигается, но позиция пишется.
    advance((dt > 0.0) ? direction_ * est.velocity * dt : 0.0);
    fill(est);
  }

  std::string name() const override {return "RouteIntegrator";}

  void reset() override
  {
    s_ = 0.0;
    direction_ = 1;
    route_ = 0;
  }

protected:
  /// Сдвинуть s на ds вдоль маршрута: кольцо по модулю L, с конца ветки — на кольцо,
  /// тупиковый конец ветки — упор. Без карты — относительный s.
  void advance(double ds)
  {
    s_ += ds;
    if (!(map_ && map_->has_routes())) {
      return;
    }
    if (route_ >= map_->num_routes()) {
      route_ = 0;
    }
    const Route & r = map_->route(route_);
    if (r.closed) {
      s_ = wrap_ring(s_);
    } else if (s_ > r.length && r.join_s_ring >= 0.0) {
      leave_branch(r);
    } else if (s_ < 0.0) {
      s_ = 0.0;  // тупиковый конец ветки
    } else if (s_ > r.length) {
      s_ = r.length;
    }
  }

  /// Записать в est положение и курс для текущих (route, s).
  void fill(Estimate & est) const
  {
    if (map_ && map_->has_routes()) {
      Point3D p;
      double hd = 0.0;
      if (map_->lookup(route_, s_, p, hd)) {
        est.position = p;
        est.heading = hd;
        est.s = s_;
        est.route_id = route_;
        est.on_route = true;
        est.direction = direction_;
      }
    } else {
      // Относительный режим (карта не задана): s от 0, позиция вдоль X.
      est.position.x = s_;
      est.position.y = 0.0;
      est.position.z = 0.0;
      est.heading = 0.0;
      est.s = s_;
      est.route_id = 0;
      est.on_route = false;
    }
  }

  bool on_ring() const {return map_ && map_->has_routes() && route_ == 0;}

  double wrap_ring(double s) const
  {
    const double L = map_->ring_length();
    if (L <= 0.0) {
      return s;
    }
    s = std::fmod(s, L);
    return (s < 0.0) ? s + L : s;
  }

  /// Проехали за конец ветки (s > length) -> переход на кольцо в join_s_ring.
  void leave_branch(const Route & branch)
  {
    const double over = s_ - branch.length;
    Point3D p;
    double hd_branch = 0.0, hd_ring = 0.0;
    map_->lookup(route_, branch.length, p, hd_branch);
    map_->lookup(0, branch.join_s_ring, p, hd_ring);
    // Направление по кольцу = знак cos(разности курсов ветки и кольца).
    const int c = (std::cos(hd_branch - hd_ring) >= 0.0) ? 1 : -1;
    direction_ *= c;
    route_ = 0;
    s_ = wrap_ring(branch.join_s_ring + c * over);
  }

  std::shared_ptr<const RouteMap> map_;
  double s_ = 0.0;
  int direction_ = 1;
  std::size_t route_ = 0;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__ROUTE_INTEGRATOR_HPP_
