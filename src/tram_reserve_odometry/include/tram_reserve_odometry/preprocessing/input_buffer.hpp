// Потокобезопасный буфер последнего сырого образца по каждому входному каналу.

#ifndef TRAM_RESERVE_ODOMETRY__PREPROCESSING__INPUT_BUFFER_HPP_
#define TRAM_RESERVE_ODOMETRY__PREPROCESSING__INPUT_BUFFER_HPP_

#include <array>
#include <mutex>

#include "tram_reserve_odometry/core/types.hpp"

namespace tram
{

enum class Channel : uint8_t
{
  FRONT = 0,
  REAR = 1,
  DRIVER = 2,
  NUM_CHANNELS = 3
};

class RawInputBuffer
{
public:
  void update(Channel c, const InputSample & s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_[static_cast<size_t>(c)] = s;
  }

  InputSample get(Channel c) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_[static_cast<size_t>(c)];
  }

  bool has(Channel c) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_[static_cast<size_t>(c)].valid;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.fill(InputSample{});
  }

private:
  mutable std::mutex mutex_;
  std::array<InputSample, static_cast<size_t>(Channel::NUM_CHANNELS)> samples_{};
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PREPROCESSING__INPUT_BUFFER_HPP_
