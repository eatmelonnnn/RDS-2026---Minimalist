#include "kinematics.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>
#include <cmath>

void compute_joint_torques_tendon_drive(double th1, double th2, double th3,
                           double f_tip_ext, double f_tip_splay,
                           double& tau1, double& tau2, double& tau3)
{
    // ── B_list from your Colab ──
    // b1 = [0, 1, 0, 0, 0, a+b+c]
    // b2 = [0, 0, 1, d, -a-b, 0]
    // b3 = [0, 0, 1, d, -a,   0]

    // ── Wrench at fingertip ──
    // F_tip = [0, 0, 0, 0, f_tip_ext, f_tip_splay]
    double Wrench[6] = {0, 0, 0, 0, f_tip_ext, f_tip_splay};

    // ── Body Jacobian Jb from modern_robotics ──
    // Jb columns correspond to joints th1, th2, th3
    // Each column is the screw axis in body frame
    // From B_list and theta_list via mr.JacobianBody

    // Screw axes in body frame (from your B_list)
    double B1[6] = {0, 1, 0, 0,        0,     a_len+b_len+c_len};
    double B2[6] = {0, 0, 1, d_len,    -a_len-b_len,         0};
    double B3[6] = {0, 0, 1, d_len,      -a_len,          0};

    // Jb.T @ Wrench = tau (simplified — assuming small angles)
    // tau_i = B_i . Wrench (dot product)
    tau1 = 0; tau2 = 0; tau3 = 0;
    for (int k = 0; k < 6; k++) {
        tau1 += B1[k] * Wrench[k];
        tau2 += B2[k] * Wrench[k];
        tau3 += B3[k] * Wrench[k];
    }

    // ── Map joint torques to motor torques via S^-1 and RM ──
    // From your Colab:
    // S_abridged = [[rj, -rj], [0, rj]]
    // S_inv      = [[1/rj, 1/rj], [0, 1/rj]]
    // RM_mat     = diag(Rm1, Rm2, Rm3)
    // Motor torques = RM_mat @ S_inv @ [tau2, tau3]

    double S_inv[2][2] = {{1.0/rj, 1.0/rj},
                          {0.0,    1.0/rj}};

    double cable_tensions[2];
    cable_tensions[0] = S_inv[0][0]*tau2 + S_inv[0][1]*tau3;
    cable_tensions[1] = S_inv[1][0]*tau2 + S_inv[1][1]*tau3;

    // Motor torques = Rm * cable_tension
    // tau1 stays as is (splay motor)
    tau2 = Rm2 * cable_tensions[0];  // MCP motor torque
    tau3 = Rm3 * cable_tensions[1];  // PIP motor torque
}

void compute_torques_direct_drive(double th2, double th3,
                     double fx, double fz,
                     double& tau_mcp, double& tau_pip)
{

    // Planar Jacobian transpose (J^T)
    // Maps [fx, fz] at fingertip → [tau_mcp, tau_pip]
    tau_mcp = -(b_len*sin(th2) + a_len*sin(th2+th3))*fx
              + (b_len*cos(th2) + a_len*cos(th2+th3))*fz;

    tau_pip = -(a_len*sin(th2+th3))*fx
              + (a_len*cos(th2+th3))*fz;
}
