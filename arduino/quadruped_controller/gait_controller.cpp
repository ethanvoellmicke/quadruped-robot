#include <Arduino.h>

#include "gait_controller.h"
#include "foot_trajectory.h"
#include "leg_ik.h"
#include "servo_controller.h"

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr uint16_t FRAME_PERIOD_MS = 20;
constexpr float TIME_STEP = static_cast<float>(FRAME_PERIOD_MS) / 1000.0f;
constexpr float GAIT_SCALE = 0.40f;
constexpr float NEUTRAL_FOOT_X = 0.040f;

constexpr float BODY_SWAY_DEG = 1.0f;
constexpr float BODY_SWAY_RAD = BODY_SWAY_DEG * PI_F / 180.0f;

constexpr float BODY_PITCH_SHIFT = 0.015f;   // 10 mm

GaitMode gaitMode = GaitMode::Stand;
float gaitTime = 0.0f;
float gaitSpeed = 2.0f;
float zGround = -0.23f;

unsigned long nextUpdateMs = 0;

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void fillPhaseOffsets(float phases[4])
{
  if (gaitMode == GaitMode::Trot)
  {
    phases[0] = 0.0f;       // FL
    phases[1] = PI_F;       // FR
    phases[2] = PI_F;       // BL
    phases[3] = 0.0f;       // BR
    return;
  }

  // Crawl order:
  // FL -> BR -> BL -> FR

  phases[0] = 0.0f;
  phases[1] = 3.0f * PI_F / 2.0f;
  phases[2] = PI_F;
  phases[3] = PI_F / 2.0f;
}

LegAngles calculateGaitDelta(
    float sampleTime,
    float phase,
    const LegAngles &neutral,
    bool rightSide,
    float bodyPitchOffset)
{
  FootPosition foot = calculateStepTrajectory(
      sampleTime,
      phase,
      gaitSpeed,
      zGround);

  // Shift body fore/aft.
  foot.x += bodyPitchOffset;

  const LegAngles current = solveLegIK(foot.x, foot.z);

  float hipDelta = (current.hip - neutral.hip) * GAIT_SCALE;
  float kneeDelta = -(current.knee - neutral.knee) * GAIT_SCALE;

  if (rightSide)
  {
    hipDelta = -hipDelta;
    kneeDelta = -kneeDelta;
  }

  return {hipDelta, kneeDelta};
}

float calculateBodySway()
{
  if (gaitMode != GaitMode::Crawl)
  {
    return 0.0f;
  }

  const float gaitPhase = gaitSpeed * gaitTime;

  return BODY_SWAY_RAD * cosf(2.0f * gaitPhase);
}

float calculateBodyPitchOffset()
{
  if (gaitMode != GaitMode::Crawl)
  {
    return 0.0f;
  }

  const float phase = fmodf(gaitSpeed * gaitTime, 2.0f * PI_F);

  // Crawl order:
  // FL -> BR -> BL -> FR

  if (phase < 0.5f * PI_F)
  {
    // Front Left swinging
    return -BODY_PITCH_SHIFT;
  }
  else if (phase < PI_F)
  {
    // Back Right swinging
    return BODY_PITCH_SHIFT;
  }
  else if (phase < 1.5f * PI_F)
  {
    // Back Left swinging
    return BODY_PITCH_SHIFT;
  }
  else
  {
    // Front Right swinging
    return -BODY_PITCH_SHIFT;
  }
}

void commandCurrentGaitPose()
{
  if (gaitMode == GaitMode::Stand)
  {
    commandNeutralPose();
    return;
  }

  float phases[4];
  fillPhaseOffsets(phases);

  const LegAngles neutral = solveLegIK(
      NEUTRAL_FOOT_X,
      zGround);

  const float bodyPitchOffset = calculateBodyPitchOffset();

  const LegAngles fl = calculateGaitDelta(
      gaitTime,
      phases[0],
      neutral,
      false,
      bodyPitchOffset);

  const LegAngles fr = calculateGaitDelta(
      gaitTime,
      phases[1],
      neutral,
      true,
      bodyPitchOffset);

  const LegAngles bl = calculateGaitDelta(
      gaitTime,
      phases[2],
      neutral,
      false,
      bodyPitchOffset);

  const LegAngles br = calculateGaitDelta(
      gaitTime,
      phases[3],
      neutral,
      true,
      bodyPitchOffset);

  const float bodySway = calculateBodySway();

  const float jointOffsets[NUM_SERVOS] =
  {
    -bodySway, fl.hip, fl.knee,
     bodySway, fr.hip, fr.knee,
    -bodySway, bl.hip, bl.knee,
     bodySway, br.hip, br.knee
  };

  commandJointOffsetsRadians(jointOffsets);
}

} // end anonymous namespace

void initializeGaitController()
{
  gaitMode = GaitMode::Stand;
  gaitTime = 0.0f;
  nextUpdateMs = millis();
  commandNeutralPose();
}

void updateGaitController()
{
  const unsigned long now = millis();

  if (static_cast<long>(now - nextUpdateMs) < 0)
  {
    return;
  }

  commandCurrentGaitPose();

  if (gaitMode != GaitMode::Stand)
  {
    gaitTime += TIME_STEP;
  }

  nextUpdateMs += FRAME_PERIOD_MS;

  if (static_cast<unsigned long>(now - nextUpdateMs) > FRAME_PERIOD_MS * 2UL)
  {
    nextUpdateMs = now + FRAME_PERIOD_MS;
  }
}

void setGaitMode(GaitMode mode)
{
  if (mode != gaitMode)
  {
    gaitMode = mode;
    gaitTime = 0.0f;
  }
}

GaitMode getGaitMode()
{
  return gaitMode;
}

void setGaitSpeed(float speed)
{
  gaitSpeed = clampFloat(speed, 0.1f, 10.0f);
}

float getGaitSpeed()
{
  return gaitSpeed;
}

void setGroundHeight(float newZGround)
{
  zGround = clampFloat(newZGround, -0.215f, -0.08f);
}

float getGroundHeight()
{
  return zGround;
}
