#include "CrtpUtils.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

static_assert(sizeof(imu_s) == 27, "provenance packet layout changed");
static_assert(offsetof(imu_s, sequenceLE) == 14, "sequence offset changed");
static_assert(offsetof(imu_s, simTimeNsLE) == 18, "timestamp offset changed");
static_assert(offsetof(imu_s, sequenceIsGazeboHeader) == 26,
              "sequence source offset changed");
static_assert(sizeof(imu_s) - 1 <= CRTP_MAX_DATA_SIZE,
              "CRTP packet exceeds the protocol payload limit");

int main() {
  std::array<uint8_t, 12> encoded{};
  constexpr uint32_t sequence = 0x12345678U;
  constexpr uint64_t simTimeNs = 0x0102030405060708ULL;
  encodeImuProvenance(encoded.data(), sequence, simTimeNs);

  const std::array<uint8_t, 12> expected = {
      0x78, 0x56, 0x34, 0x12, 0x08, 0x07,
      0x06, 0x05, 0x04, 0x03, 0x02, 0x01};
  assert(encoded == expected);
  assert(decodeImuSequence(encoded.data()) == sequence);
  assert(decodeImuSimTimeNs(encoded.data() + 4) == simTimeNs);
}
