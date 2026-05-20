#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>

#define LENGTH_A 0.0354
#define LENGTH_B 0.043
#define LENGTH_C 0.0298
#define LENGTH_D 0.02

#define SPLAY_MIN 
#define SPLAY_MAX
#define MCP_MIN
#define MCP_MAX
#define DIP_MIN



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
// angles cartesian_pos_to_joint_pos(cartesian_pos xyz);

#endif