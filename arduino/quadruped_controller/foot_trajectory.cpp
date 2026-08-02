#include <Arduino.h>
#include <math.h>

#include "foot_trajectory.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;

float clamp01(float value)
{
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float easeOutCubic(float t)
{
  t = clamp01(t);
  const float oneMinusT = 1.0f - t;
  return 1.0f - oneMinusT * oneMinusT * oneMinusT;
}

float easeInCubic(float t)
{
  t = clamp01(t);
  return t * t * t;
}

float smoothStep(float t)
{
  t = clamp01(t);
  return t * t * (3.0f - 2.0f * t);
}
}

FootPosition calculateStepTrajectory(
  float timeSeconds,
  float phaseOffset,
  float speed,
  float zGround,
  float xCenter,
  float stepLength,
  float stepHeight,
  float stanceDepth
)
{
  float phase = fmodf(-speed * timeSeconds + phaseOffset, 2.0f * PI_F);

  if (phase < 0.0f)
  {
    phase += 2.0f * PI_F;
  }

  const float cycleT = phase / (2.0f * PI_F);
  const float xBack = xCenter - 0.5f * stepLength;
  const float xFront = xCenter + 0.5f * stepLength;

  FootPosition foot{};
  constexpr float SWING_PORTION = 0.25f;

  if (cycleT < SWING_PORTION)
  {
    const float swingT = cycleT / SWING_PORTION;
    constexpr float LIFT_END = 0.30f;
    constexpr float DROP_START = 0.70f;

    if (swingT < LIFT_END)
    {
      const float t = swingT / LIFT_END;
      const float lift = easeOutCubic(t);
      foot.x = xBack;
      foot.z = zGround + stepHeight * lift;
    }
    else if (swingT < DROP_START)
    {
      const float t = (swingT - LIFT_END) / (DROP_START - LIFT_END);
      const float forward = smoothStep(t);
      foot.x = xBack + stepLength * forward;
      foot.z = zGround + stepHeight;
    }
    else
    {
      const float t = (swingT - DROP_START) / (1.0f - DROP_START);
      const float drop = easeInCubic(t);
      foot.x = xFront;
      foot.z = zGround + stepHeight * (1.0f - drop);
    }
  }
  else
  {
    const float stanceT =
      (cycleT - SWING_PORTION) / (1.0f - SWING_PORTION);

    foot.x = xFront - stepLength * stanceT;
    foot.z = zGround - stanceDepth * sinf(PI_F * stanceT);
  }

  return foot;
}
