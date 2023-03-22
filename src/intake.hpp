#include "main.h"
#include "ports.hpp"

#pragma once

class IntakeClass {

  int vLock = 0;

public:
  pros::Motor intake_motor =
      pros::Motor(INTAKE_PORT, pros::E_MOTOR_GEARSET_18, true);
  pros::Motor intake_motor2 =
      pros::Motor(INTAKE_PORT2, pros::E_MOTOR_GEARSET_18, false);
  pros::MotorGroup intake_motors =
      pros::MotorGroup({intake_motor, intake_motor2});

  void moveVoltage(int v) {
    if (vLock > 0) return;
    intake_motors.move_voltage(v);
  }

  void loopController(bool pRollerUp, bool pRollerDown, int vel) {
    if (vLock > 0) return;

    if (pRollerUp) spinRoller(true);
    else if (pRollerDown) spinRoller(false);
    else if (vel) intake_motors.move_voltage(vel * INTAKE_MAX_VOLTAGE * 1000);
    else intake_motors.move_voltage(0);
  }

  void spinRoller(bool forward = true) {
    intake_motors.move_voltage(7000 * ((((int)forward) * 2) - 1));
  }

  void lock() { vLock++; }
  void unlock() {
    if (vLock == 0) return;
    vLock--;
  }

} Intake;