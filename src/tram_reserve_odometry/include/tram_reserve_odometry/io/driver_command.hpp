// Разбор сериализованной команды машиниста tram_vehicle_msgs/msg/DriverControllerCommand
// (CDR) без сгенерированного C++-типа:
//   std_msgs/Header header   (int32 sec, uint32 nanosec, string frame_id)
//   int8 position            (0 — нейтраль, +1..+15 тяга, −1..−15 торможение)

#ifndef TRAM_RESERVE_ODOMETRY__IO__DRIVER_COMMAND_HPP_
#define TRAM_RESERVE_ODOMETRY__IO__DRIVER_COMMAND_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tram
{

/// Имя ROS-типа ручки (для GenericSubscription).
inline constexpr const char * kDriverCommandType = "tram_vehicle_msgs/msg/DriverControllerCommand";

struct DriverCommand
{
  int32_t sec = 0;
  uint32_t nanosec = 0;
  int8_t position = 0;

  int64_t stamp_ns() const
  {
    return static_cast<int64_t>(sec) * 1000000000LL + static_cast<int64_t>(nanosec);
  }
};

/// Разбор сериализованного сообщения (4 байта инкапсуляции CDR + тело).
/// \return false, если буфер короткий или инкапсуляция неизвестна.
inline bool decode_driver_command(const uint8_t * data, std::size_t size, DriverCommand & out)
{
  if (data == nullptr || size < 4) {
    return false;
  }
  // Инкапсуляция: 00 00 — CDR big-endian, 00 01 — CDR little-endian.
  if (data[0] != 0 || data[1] > 1) {
    return false;
  }
  const bool little = data[1] == 1;
  const uint8_t * body = data + 4;   // выравнивание считается от начала тела
  const std::size_t n = size - 4;
  auto u32 = [&](std::size_t off) {
      uint32_t v = 0;
      for (int i = 0; i < 4; ++i) {
        const uint32_t b = body[off + (little ? i : 3 - i)];
        v |= b << (8 * i);
      }
      return v;
    };
  // sec(4) + nanosec(4) + длина frame_id(4).
  if (n < 12) {
    return false;
  }
  out.sec = static_cast<int32_t>(u32(0));
  out.nanosec = u32(4);
  const uint32_t len = u32(8);   // с завершающим нулём
  const std::size_t pos_off = 12 + static_cast<std::size_t>(len);
  if (len > n || pos_off >= n) {
    return false;
  }
  std::memcpy(&out.position, body + pos_off, 1);
  return true;
}

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__IO__DRIVER_COMMAND_HPP_
