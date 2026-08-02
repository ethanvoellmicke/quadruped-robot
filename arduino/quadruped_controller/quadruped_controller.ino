#include "servo_controller.h"
#include "gait_controller.h"
#include "command_receiver.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Quadruped local gait controller starting");

  initializeServoController();
  initializeGaitController();
  initializeCommandReceiver();

  Serial.println("Ready. Commands: stand, crawl, trot, speed <value>, height <value>, status");
}

void loop()
{
  // Keep the 20 ms gait scheduler first so communication cannot starve motion.
  updateGaitController();
  updateCommandReceiver();
  updateGaitController();
}
