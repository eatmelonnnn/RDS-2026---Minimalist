#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

#define LENGTH_A 0.035//0.0354
#define LENGTH_B 0.043
#define LENGTH_C 0.0298
#define LENGTH_D 0.01//0.02


#define Rm1 0.005f
#define Rm2 0.005f
#define Rm3 0.005f


#define rj 0.008f
#define rw 0.010f
#define rs (0.0191f/2.0f)

#define HARDSTOP_JOINT_1 -0.349f //32645023f
#define HARDSTOP_JOINT_2 -1.57079633f
#define HARDSTOP_JOINT_3 1.97079633f

enum JOINT_NAMES {
  MCP,
  DIP
};

struct tendonLengths {
    float l1;
    float l2;
};

struct angles {
    float th1;
    float th2;
    float th3;
};

struct cartesian_pos {
  float x;
  float y;
  float z;
};

float norm(float a, float b);
void tip_zero_force_to_outputs(float tip_force,  float tendon_tensions[2], float m_torques[2]);
angles cartesian_pos_to_joint_pos(cartesian_pos xyz);
tendonLengths multiply_AT(float th1, float th2, float th3);
angles joint_pos_to_motor_pos(angles jointpos, float calibration_offsets[3]);
angles motor_pos_to_joint_pos(float pos1, float pos2, float pos3, float calibration_offsets[3]);



#endif