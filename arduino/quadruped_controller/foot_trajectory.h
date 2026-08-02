#ifndef FOOT_TRAJECTORY_H
#define FOOT_TRAJECTORY_H

#include "gait_types.h"

FootPosition calculateStepTrajectory(
  float timeSeconds,
  float phaseOffset,
  float speed,
  float zGround,
  float xCenter = 0.06f,//0.04 default
  float stepLength = 0.12f,
  float stepHeight = 0.05f,
  float stanceDepth = 0.008f//.008 default
);

#endif
