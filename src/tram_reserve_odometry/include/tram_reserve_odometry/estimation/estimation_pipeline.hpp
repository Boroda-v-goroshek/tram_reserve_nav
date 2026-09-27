// Пайплайн оценки: PreprocessedSample -> модель тяги -> детектор проскальзывания ->
// оценщик скорости -> интегратор положения -> Estimate.

#ifndef TRAM_RESERVE_ODOMETRY__ESTIMATION__ESTIMATION_PIPELINE_HPP_
#define TRAM_RESERVE_ODOMETRY__ESTIMATION__ESTIMATION_PIPELINE_HPP_

#include <memory>

#include "tram_reserve_odometry/core/interfaces.hpp"
#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

class EstimationPipeline
{
public:
  EstimationPipeline(
    std::shared_ptr<ITractionModel> traction,
    std::shared_ptr<ISlipDetector> slip,
    std::shared_ptr<IStateEstimator> estimator,
    std::shared_ptr<IPositionIntegrator> integrator,
    std::shared_ptr<IPositionInitializer> initializer,
    ModelParams params = ModelParams{})
  : traction_(std::move(traction)),
    slip_(std::move(slip)),
    estimator_(std::move(estimator)),
    integrator_(std::move(integrator)),
    initializer_(std::move(initializer)),
    params_(params)
  {
    if (estimator_ && traction_) {
      estimator_->set_traction_model(traction_);
    }
  }

  const ModelParams & params() const {return params_;}
  void set_params(const ModelParams & p) {params_ = p;}

  Estimate process(const PreprocessedSample & sample)
  {
    // Первый вызов: инициализация положения / скорости.
    if (!initialized_ && initializer_) {
      est_.position = initializer_->initialize_position(params_);
      est_.velocity = initializer_->initialize_velocity(params_);
      est_.acceleration = 0.0;
      est_.initialized = true;
      initialized_ = true;
    }

    // Невалидный вход: держим последнюю оценку.
    if (!sample.valid) {
      est_.quality = sample.quality;
      est_.stamp_ns = sample.stamp_ns;
      est_.slip = SlipStatus::NONE;
      est_.slip_confidence = 0.0;
      return est_;
    }

    // Шаг времени из синхронизированных таймстампов.
    double dt = 0.0;
    if (has_last_) {
      dt = static_cast<double>(sample.stamp_ns - last_stamp_ns_) * 1e-9;
      if (dt < 0.0) {
        dt = 0.0;
      }
    }
    last_stamp_ns_ = sample.stamp_ns;
    has_last_ = true;

    const double model_accel =
      traction_ ? traction_->compute_acceleration(sample, params_) : 0.0;
    const ISlipDetector::Result slip_res =
      slip_ ? slip_->detect(sample, params_) : ISlipDetector::Result{};

    if (estimator_) {
      // Позиция и курс — состояние интегратора: оценщик их не меняет.
      const Point3D prev_position = est_.position;
      const double prev_heading = est_.heading;
      estimator_->set_route_context(est_);  // привязка прошлого шага (уклон, кривизна)
      est_ = estimator_->estimate(sample, model_accel, slip_res, params_);
      est_.position = prev_position;
      est_.heading = prev_heading;
    } else {
      est_.velocity = sample.front_bogie_velocity;
      est_.acceleration = model_accel;
    }

    // Статус проскальзывания — от детектора, иначе — от оценщика.
    if (slip_res.status != SlipStatus::NONE || est_.slip == SlipStatus::NONE) {
      est_.slip = slip_res.status;
      est_.slip_confidence = slip_res.confidence;
    }
    est_.quality = sample.quality;
    est_.stamp_ns = sample.stamp_ns;
    est_.initialized = true;

    if (integrator_) {
      integrator_->integrate(est_, dt, params_);
    }
    return est_;
  }

  void reset()
  {
    initialized_ = false;
    has_last_ = false;
    last_stamp_ns_ = 0;
    est_ = Estimate{};
    if (estimator_) {
      estimator_->reset();
    }
    if (integrator_) {
      integrator_->reset();
    }
  }

  const Estimate & last() const {return est_;}

private:
  std::shared_ptr<ITractionModel> traction_;
  std::shared_ptr<ISlipDetector> slip_;
  std::shared_ptr<IStateEstimator> estimator_;
  std::shared_ptr<IPositionIntegrator> integrator_;
  std::shared_ptr<IPositionInitializer> initializer_;
  ModelParams params_;
  Estimate est_{};
  int64_t last_stamp_ns_ = 0;
  bool has_last_ = false;
  bool initialized_ = false;
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__ESTIMATION__ESTIMATION_PIPELINE_HPP_
