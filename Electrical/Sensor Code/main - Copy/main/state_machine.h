#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "motors.h"
#include "tensionsensor.h"


enum CONTROL_MODES {
  CALIBRATION = 0,
  STEP_POSITION,
  LISSAJOU_POSITION,
  ELLIPSE_POSITION,
  ZERO_FORCE,
  STEP_FORCE,
  MAX_FORCE,
  STARTUP = -1,
  FLEX_EXT = 100
};

#define PIN_CALIBRATION 2
#define PIN_STEP_POSITION 3
#define PIN_LISSAJOU_POSITION 4
#define PIN_ELLIPSE_POSITION 5
#define PIN_ZERO_FORCE 6
#define PIN_STEP_FORCE 7
#define PIN_MAX_FORCE 8
#define PIN_FLEX_EXT 33



void initial_universal_setup();
void state_machine_cycle();
CONTROL_MODES get_state();
#endif
