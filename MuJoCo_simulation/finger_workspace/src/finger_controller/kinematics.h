#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <cmath>

// Link lengths (metres)
const double a_len = 0.045;   
const double b_len = 0.043;   
const double c_len = 0.030;   
const double d_len = 0.034;   

// Joint pulley radii (metres)
const double rj = 0.0;  
const double rw = 0.0;  
const double rs = 0.0;  

// Motor pulley radii (metres)
const double Rm1 = 0.0; 
const double Rm2 = 0.0; 
const double Rm3 = 0.0; 

// Ball/pendulum physical parameters
const double ball_mass    = 0.05;   // kg
const double rod_mass     = 0.01;   // kg
const double total_mass   = ball_mass + rod_mass;
const double gravity      = 9.81;   // m/s^2
const double ball_weight  = total_mass * gravity; // N

// Impedance control gains
const double Kp = 5.0;   // position stiffness (Nm/rad)
const double Kd = 0.5;   // velocity damping   (Nm·s/rad)

void compute_joint_torques_tendon_drive(double th1, double th2, double th3,
                           double f_tip_ext, double f_tip_splay,
                           double& tau1, double& tau2, double& tau3);

void compute_torques_direct_drive(double th2, double th3,
                            double fx, double fz,
                            double& tau_mcp, double& tau_pip);