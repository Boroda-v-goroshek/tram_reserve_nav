// Начальное положение и скорость из конфига (initialization.*).

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__ZERO_POSITION_INITIALIZER_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__ZERO_POSITION_INITIALIZER_HPP_

#include "tram_reserve_odometry/core/interfaces.hpp"

namespace tram
{

class ZeroPositionInitializer : public IPositionInitializer
{
public:
  ZeroPositionInitializer() = default;

  Point3D initialize_position(const ModelParams & params) const override
  {
    return params.initial_position;
  }

  double initialize_velocity(const ModelParams & params) const override
  {
    return params.initial_velocity;
  }

  std::string name() const override {return "ZeroPositionInitializer";}
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__ZERO_POSITION_INITIALIZER_HPP_
