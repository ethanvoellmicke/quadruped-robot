#include <Arduino.h>
#include <math.h>

#include "leg_ik.h"

namespace
{
constexpr float UPPER_LEG_LENGTH = 0.110f;
constexpr float LOWER_LEG_LENGTH = 0.1105f;

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}
}

LegAngles solveLegIK(float x, float z)
{
  const float requestedDistance = sqrtf(x * x + z * z);
  const float maximumReach = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH - 0.001f;
  const float minimumReach = fabsf(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH) + 0.001f;
  const float distance = clampFloat(requestedDistance, minimumReach, maximumReach);

  float cosineKnee =
    (distance * distance
      - UPPER_LEG_LENGTH * UPPER_LEG_LENGTH
      - LOWER_LEG_LENGTH * LOWER_LEG_LENGTH)
    / (2.0f * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH);

  cosineKnee = clampFloat(cosineKnee, -1.0f, 1.0f);

  const float knee = -acosf(cosineKnee);
  const float angleToFoot = atan2f(z, x);

  const float internalAngle = atan2f(
    LOWER_LEG_LENGTH * sinf(fabsf(knee)),
    UPPER_LEG_LENGTH + LOWER_LEG_LENGTH * cosf(fabsf(knee))
  );

  const float hip = angleToFoot + internalAngle;
  return {hip, knee};
}
