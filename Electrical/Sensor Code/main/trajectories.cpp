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

angles generate_step_position_benchmark(float freq)  {
  cartesian_pos start = {
    0.0f,
    0.040f,
    0.070f
};
cartesian_pos end = {
    0.0f,
    0.040f,
    0.050f
};
cartesian_pos cart_now;
uint32_t period_ms = (uint32_t)(1000.0f / freq);
    uint32_t t = millis() % period_ms;

    if (t < period_ms / 2) {
        cart_now  = start;
    } else {
        cart_now = end;
    }
    angles r = cartesian_pos_to_joint_pos(cart_now);
  return r;
}

angles generate_lissajous() {

    const float f = 0.25f;     // Hz
    const float A = 0.015f;    // 1.5 cm

    const float YC = 0.030f;
    const float ZC = 0.06f;

    cartesian_pos p;

    p.x = 0.0f;

    // Lissajous trajectory
    float t = millis()/1000.0;
    p.y = YC + A*sin(2.0f*M_PI*f*t);

    p.z = ZC + A*sin(4.0f*M_PI*f*t + 3.0f*M_PI/4.0f);
    angles cur_pos = cartesian_pos_to_joint_pos(p);
    return cur_pos;
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

angles generate_flex_ext_sinusoid() {
  angles sin_ang;
  const float freq = 0.8;
  const float amp = PI/4*0.85;
  uint32_t period_ms = (uint32_t)(1000.0f / freq);
  float t = (millis() % period_ms)/1000.0f;
  sin_ang.th1 = 0.0f;
  sin_ang.th2 = PI/4 + amp*sin(t*freq*PI);
  sin_ang.th3 = PI/4 + amp*sin(t*freq*PI);
  return sin_ang;
  
  


}

float generate_fingertip_force_step(float a, float b, float freq){
  uint32_t period_ms = (uint32_t)(1000.0f / freq);
      uint32_t t = millis() % period_ms;

      if (t < period_ms / 2) {
          return a;
      } else {
          return b;
      }
  }
