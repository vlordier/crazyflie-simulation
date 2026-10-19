#include "CrtpUtils.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace {
constexpr float kStandardMotorConstant = 1.7965e-8f;
constexpr float kStandardMaxRotVelocity = 3052.0f;
constexpr float kUpgradeMotorConstant = 2.3375e-8f;
constexpr float kUpgradeMaxRotVelocity = 2797.0f;

float firmwareThrust(uint16_t pwm) {
  const float normalizedPwm = pwm / 65535.0f;
  return kPwmToThrustA * normalizedPwm * normalizedPwm +
         kPwmToThrustB * normalizedPwm;
}

float thrust(float omega, float motorConstant) {
  return motorConstant * omega * omega;
}

void near(float actual, float expected) {
  assert(std::fabs(actual - expected) <= std::fabs(expected) * 1e-5f);
}
}  // namespace

int main() {
  constexpr uint16_t baseThrust = 36000;
  const float expectedThrust = firmwareThrust(baseThrust);

  const float standardOmega = PWM2OMEGA(
      baseThrust, kStandardMotorConstant, kStandardMaxRotVelocity);
  const float upgradeOmega = PWM2OMEGA(
      baseThrust, kUpgradeMotorConstant, kUpgradeMaxRotVelocity);
  near(thrust(standardOmega, kStandardMotorConstant), expectedThrust);
  near(thrust(upgradeOmega, kUpgradeMotorConstant), expectedThrust);
  near(standardOmega,
       std::sqrt(expectedThrust / kStandardMotorConstant));
  near(upgradeOmega,
       std::sqrt(expectedThrust / kUpgradeMotorConstant));

  assert(PWM2OMEGA(6999, kStandardMotorConstant,
                   kStandardMaxRotVelocity) == 0.0f);
  near(PWM2OMEGA(65535, kStandardMotorConstant,
                 kStandardMaxRotVelocity),
       std::sqrt(firmwareThrust(65535) / kStandardMotorConstant));
  near(PWM2OMEGA(65535, kUpgradeMotorConstant,
                 kUpgradeMaxRotVelocity),
       std::sqrt(firmwareThrust(65535) / kUpgradeMotorConstant));
  assert(PWM2OMEGA(65535, kStandardMotorConstant,
                   kStandardMaxRotVelocity / 2.0f) ==
         kStandardMaxRotVelocity / 2.0f);
  assert(PWM2OMEGA(baseThrust, 0.0f, kStandardMaxRotVelocity) == 0.0f);
  assert(PWM2OMEGA(baseThrust, kStandardMotorConstant,
                   std::numeric_limits<float>::infinity()) == 0.0f);
  assert(PWM2OMEGA(baseThrust, kStandardMotorConstant,
                   kStandardMaxRotVelocity, -1.0f, kPwmToThrustB) == 0.0f);
  assert(PWM2OMEGA(baseThrust, kStandardMotorConstant,
                   kStandardMaxRotVelocity, kPwmToThrustA,
                   std::numeric_limits<float>::quiet_NaN()) == 0.0f);
}
