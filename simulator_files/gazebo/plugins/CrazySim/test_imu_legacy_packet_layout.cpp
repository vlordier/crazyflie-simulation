#include "CrtpUtils.h"

#include <cassert>

static_assert(sizeof(imu_s) == 14, "legacy IMU packet layout changed");
static_assert(sizeof(imu_s) - 1 == 13, "legacy IMU payload size changed");

int main() {
  assert(sizeof(imu_s) - 1 <= CRTP_MAX_DATA_SIZE);
}
