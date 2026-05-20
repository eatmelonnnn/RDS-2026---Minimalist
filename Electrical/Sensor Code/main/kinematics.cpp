#include "kinematics.h"

const float A_PRIME = norm(LENGTH_A, LENGTH_D);
const float DELTA = atan2(LENGTH_D, LENGTH_A);


float norm(float a, float b) {
  // return sqrt(sq(a) +sq(b));
  return a;
}

// angles cartesian_pos_to_joint_pos(cartesian_pos xyz) {
//   // float D_sq = norm(cartesian_pos.y, cartesian_pos.z);
//   // float r = norm(cartesian_pos.x, cartesian_pos.y);

//   angles a;
//   // a.th1 = constrain(atan2(cartesian_pos.x, cartesian_pos.y);
//   // a.th3 = acos((D_sq - sq(LENGTH_B) - sq(A_PRIME))/ (2*LENGTH_B*A_PRIME));
  
// }
