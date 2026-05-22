#include "trajectories.h"

float ellipse_1d(float c, float u, float v, uint32_t t, uint32_t period_ms) {
  double phase = double(t)*2*PI/double(period_ms);
  return float(c + u*cos(phase) + v*sin(phase));
}

cartesian_pos ellipse_3d(cartesian_pos c, cartesian_pos u, cartesian_pos v, uint32_t t, uint32_t period_ms) {
  cartesian_pos final_pos;
  final_pos.x = ellipse_1d(c.x, u.x, v.x, t, period_ms);
  final_pos.y = ellipse_1d(c.y, u.y, v.y, t, period_ms);
  final_pos.z = ellipse_1d(c.z, u.z, v.z, t, period_ms);
  return final_pos;
}

angles generate_ellipse(float freq) {
  uint32_t period_ms = (uint32_t)(1000.0f / freq);
  uint32_t t = millis() % period_ms;
  cartesian_pos center = {0.0, 0.06, 0.045};
  cartesian_pos u = {0.005, 0, 0};
  cartesian_pos v = {0, 0.01, 0};
  cartesian_pos cur_pos = ellipse_3d(center,  u, v, t, period_ms);
  angles cur_angles = cartesian_pos_to_joint_pos(cur_pos);
  return cur_angles;
  
}

angles generate_step_response(angles a, angles b, float freq) {
    uint32_t period_ms = (uint32_t)(1000.0f / freq);
    uint32_t t = millis() % period_ms;

    if (t < period_ms / 2) {
        return a;
    } else {
        return b;
    }
}