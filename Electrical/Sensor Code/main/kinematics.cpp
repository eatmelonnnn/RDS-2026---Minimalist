#include "kinematics.h"

const float A_PRIME = norm(LENGTH_A, LENGTH_D);
const float DELTA = atan2(LENGTH_D, LENGTH_A);

float sq(float a) {
  return a*a;
}

float norm(float a, float b) {
  return sqrt(sq(a) + sq(b));
}

angles cartesian_pos_to_joint_pos(cartesian_pos xyz) {
  float D = norm(cartesian_pos.y, cartesian_pos.z);
  float r = norm(cartesian_pos.x, cartesian_pos.y);
  float r_prime = r - LENGTH_C;

  angles a;
  a.th1 = atan2(cartesian_pos.x, cartesian_pos.y);
  a.th3 = acos((sq(D) - sq(LENGTH_B) - sq(A_PRIME)) / (2*LENGTH_B*A_PRIME));
  a.th2 = atan2(cartesian_pos.z, r_prime) - atan2(A_PRIME * sin(a.th3), LENGTH_B + A_PRIME * cos(a.th3)) - DELTA;
  
  return a;
}
