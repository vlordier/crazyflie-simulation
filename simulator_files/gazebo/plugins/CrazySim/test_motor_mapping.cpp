#include "CrtpUtils.h"

#include <cassert>
#include <cmath>

namespace {
constexpr float kHoverThrustPerMotor = 0.18f;
constexpr float kStandardMotorConstant = 1.7965e-8f;
constexpr float kStandardMaxRotVelocity = 3052.0f;
constexpr float kUpgradeMotorConstant = 2.3375e-8f;
constexpr float kUpgradeMaxRotVelocity = 2797.0f;

float thrust(float omega, float motor_constant) {
  return motor_constant * omega * omega;
}

void near(float actual, float expected) {
  assert(std::fabs(actual - expected) <= expected * 1e-5f);
}
}  // namespace

int main() {
  constexpr uint16_t pwm = 36000;
  const float expected_thrust = (pwm / 65535.0f) * kHoverThrustPerMotor;

  const float standard_omega =
      PWM2OMEGA(pwm, kStandardMotorConstant, kStandardMaxRotVelocity);
  const float upgrade_omega =
      PWM2OMEGA(pwm, kUpgradeMotorConstant, kUpgradeMaxRotVelocity);
  near(thrust(standard_omega, kStandardMotorConstant), expected_thrust);
  near(thrust(upgrade_omega, kUpgradeMotorConstant), expected_thrust);
  assert(standard_omega > upgrade_omega);

  assert(PWM2OMEGA(6999, kStandardMotorConstant,
                   kStandardMaxRotVelocity) == 0.0f);
  near(PWM2OMEGA(65535, kStandardMotorConstant, kStandardMaxRotVelocity),
       kStandardMaxRotVelocity);
  near(PWM2OMEGA(65535, kUpgradeMotorConstant, kUpgradeMaxRotVelocity),
       std::sqrt(kHoverThrustPerMotor / kUpgradeMotorConstant));
  assert(PWM2OMEGA(pwm, 0.0f, kStandardMaxRotVelocity) == 0.0f);
  assert(PWM2OMEGA(pwm, kStandardMotorConstant,
                   std::numeric_limits<float>::infinity()) == 0.0f);
}
