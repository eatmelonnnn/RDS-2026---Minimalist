#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "kinematics.h"
#include <stdint.h>
#include <Arduino.h>


angles generate_step_response(angles a, angles b, float freq);

angles generate_ellipse(float freq);

#endif