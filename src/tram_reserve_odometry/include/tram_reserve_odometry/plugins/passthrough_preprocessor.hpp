// Препроцессор без фильтрации: опорное время = самый свежий stamp, приведение единиц
// (тележки -> м/с, ручка -> [−1, 1]), качество OK/MISSING по наличию каналов.
// Отбраковка выбросов и отказов — в оценщике.

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__PASSTHROUGH_PREPROCESSOR_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__PASSTHROUGH_PREPROCESSOR_HPP_

#include "tram_reserve_odometry/core/interfaces.hpp"
#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

class PassthroughPreprocessor : public IPreprocessor
{
public:
  PassthroughPreprocessor() = default;

  PreprocessedSample process(
    const InputSample & front,
    const InputSample & rear,
    const InputSample & driver,
    const ModelParams & params) override
  {
    PreprocessedSample out{};

    // Опорное время = самый свежий доступный таймстамп.
    int64_t ref = 0;
    bool any = false;
    for (const InputSample * s : {&front, &rear, &driver}) {
      if (s->valid && s->stamp_ns > ref) {
        ref = s->stamp_ns;
        any = true;
      }
    }
    if (!any) {
      out.quality = DataQuality::MISSING;
      out.valid = false;
      return out;
    }
    out.stamp_ns = ref;

    // Приведение единиц.
    out.front_bogie_velocity = front.valid ? front.value * params.bogie_scale : 0.0;
    out.rear_bogie_velocity = rear.valid ? rear.value * params.bogie_scale : 0.0;
    out.driver_position_cmd = driver.valid ? driver.value * params.driver_scale : 0.0;
    out.front_stamp_ns = front.valid ? front.stamp_ns : 0;
    out.rear_stamp_ns = rear.valid ? rear.stamp_ns : 0;
    out.driver_stamp_ns = driver.valid ? driver.stamp_ns : 0;
    out.front_valid = front.valid;
    out.rear_valid = rear.valid;

    // Presence-качество: OK, если все три канала на месте, иначе MISSING.
    const bool all = front.valid && rear.valid && driver.valid;
    out.quality = all ? DataQuality::OK : DataQuality::MISSING;
    // Для оценки скорости нужны обе тележки; ручка опциональна (0 = нейтраль).
    out.valid = front.valid && rear.valid;

    return out;
  }

  void reset() override {}
  std::string name() const override {return "PassthroughPreprocessor";}
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__PASSTHROUGH_PREPROCESSOR_HPP_
