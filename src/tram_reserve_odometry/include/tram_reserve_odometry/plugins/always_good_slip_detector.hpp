// Детектор проскальзывания без собственной логики: всегда «без проскальзывания»
// (статус проскальзывания даёт KalmanWheelEstimator).

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__ALWAYS_GOOD_SLIP_DETECTOR_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__ALWAYS_GOOD_SLIP_DETECTOR_HPP_

#include "tram_reserve_odometry/core/interfaces.hpp"

namespace tram
{

class AlwaysGoodSlipDetector : public ISlipDetector
{
public:
  AlwaysGoodSlipDetector() = default;

  Result detect(const PreprocessedSample &, const ModelParams &) const override
  {
    Result r;
    r.status = SlipStatus::NONE;
    r.confidence = 1.0;
    return r;
  }

  std::string name() const override {return "AlwaysGoodSlipDetector";}
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__ALWAYS_GOOD_SLIP_DETECTOR_HPP_
