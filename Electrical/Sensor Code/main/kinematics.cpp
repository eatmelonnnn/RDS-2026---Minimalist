#include "kinematics.h"


const float A_PRIME = norm(LENGTH_A, LENGTH_D);
const float DELTA = atan2(LENGTH_D, LENGTH_A);

const float Blist_zero_tip[3] = {0, -0.078, -0.035};
// = {{0, 0, 0},
//                                   {1, 0, 0},
//                                   {0, 1, 1},
//                                   {0, 0.02, 0.02},
//                                   {0, -0.078, -0.035},
//                                   {0.108, 0, 0}};
                                


void zero_tip_force_to_joint_torques(float tip_force, float torques[3]) {
  for (int i = 0; i < 3; i++) {
    torques[i] = Blist_zero_tip[i]*tip_force;
  }
}

void joint_torques_to_tendon_tensions(float joint_torques[3], float tendon_tensions[2]) {
  tendon_tensions[MCP] = (joint_torques[1] + joint_torques[2])/rj;
  tendon_tensions[DIP] = joint_torques[2]/rj;
}

void tendon_tensions_to_motor_torques(float tendon_tensions[2], float motor_t[2]) {
  motor_t[MCP] = tendon_tensions[MCP]*Rm2;
  motor_t[DIP] = tendon_tensions[DIP]*Rm3;
}

void tip_zero_force_to_outputs(float tip_force, float tendon_tensions[2], float m_torques[2]) {
  float j_torques[3];
  zero_tip_force_to_joint_torques(tip_force, j_torques);
  joint_torques_to_tendon_tensions(j_torques, tendon_tensions);
  tendon_tensions_to_motor_torques(tendon_tensions, m_torques);

}


float sq(float a) {
  return a*a;
}

float norm(float a, float b) {
  return sqrt(sq(a) + sq(b));

}

angles cartesian_pos_to_joint_pos(cartesian_pos xyz) {
  float r = norm(xyz.x, xyz.y);
  float r_prime = r - LENGTH_C;
  float D = norm(r_prime, xyz.z);
  

  angles a;
  a.th1 = atan2(xyz.x, xyz.y);
  a.th3 = acos((sq(D) - sq(LENGTH_B) - sq(A_PRIME)) / (2*LENGTH_B*A_PRIME));
  a.th2 = atan2(xyz.z, r_prime) - atan2(A_PRIME * sin(a.th3), LENGTH_B + A_PRIME * cos(a.th3));
  a.th3 = a.th3 - DELTA;
  
  return a;
}


tendonLengths multiply_AT(float th1, float th2, float th3) {
    tendonLengths y;
    y.l1 = -rw * th1 - rj * th2;
    y.l2 = -rw * th1 - rj * th2 + rj * th3;

    return y;
}

angles motor_pos_to_joint_pos(float pos1, float pos2, float pos3, float calibration_offsets[3]) {
    angles j;
    j.th1 = (pos1 - calibration_offsets[0])*Rm1/rs;
    float tendon_1 = (pos2 - calibration_offsets[1])*Rm2;
    j.th2 = -(tendon_1 + rw*j.th1)/rj;
    float tendon_2 = (pos3 - calibration_offsets[2])*Rm3;
    j.th3 = (tendon_2 + rw*j.th1 + rj*j.th2)/rj;

    return j;

}

angles joint_pos_to_motor_pos(angles jointpos, float calibration_offsets[3]) {
  angles m;

    float joint_angle_diff_1 = jointpos.th1;
    float joint_angle_diff_2 =  jointpos.th2;
    float joint_angle_diff_3 =  jointpos.th3;

    tendonLengths tendon_diff =
        multiply_AT(joint_angle_diff_1, joint_angle_diff_2, joint_angle_diff_3);

    m.th2 = calibration_offsets[1] + tendon_diff.l1 / Rm2;
    m.th3 = calibration_offsets[2] + tendon_diff.l2 / Rm3;
    m.th1 = calibration_offsets[0] + joint_angle_diff_1 * rs / Rm1;

    return m;
}