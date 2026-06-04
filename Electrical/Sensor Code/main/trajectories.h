#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include "kinematics.h"
#include <stdint.h>
#include <Arduino.h>


angles generate_step_response(angles a, angles b, float freq);
angles generate_step_position_benchmark(float freq);
angles generate_ellipse(float freq);
angles generate_lissajous();
float  generate_fingertip_force_step(float a, float b, float freq);
angles generate_flex_ext_sinusoid();

#endif