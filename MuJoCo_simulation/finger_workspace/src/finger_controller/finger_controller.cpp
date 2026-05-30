#include "kinematics.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

// Global pointers
mjModel* m = nullptr;
mjData*  d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// Mouse state for camera control
bool button_left   = false;
bool button_right  = false;
double lastx = 0, lasty = 0;

// Keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// Mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)  == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

// Mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if (!button_left && !button_right)
        return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;

    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                 glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action;

    if (button_right)
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;

    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// Scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

int main() {
    // Load model
    char error[1000] = "";
    m = mj_loadXML("../../finger_description/urdf/test.xml", nullptr, error, 1000);
    if (!m) {
        printf("Error loading model: %s\n", error);
        return 1;
    }

    d = mj_makeData(m);
    // Load starting keyframe
    int key_id = mj_name2id(m, mjOBJ_KEY, "start");
    if (key_id >= 0) {
        mj_resetDataKeyframe(m, d, key_id);
        printf("Keyframe loaded\n");
    } else {
        printf("Keyframe not found\n");
    }

    // Init GLFW
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Finger Control", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Set callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // Init MuJoCo rendering
    mjv_defaultCamera(&cam);
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.0;

    cam.distance = 1;
    cam.azimuth = 90;
    cam.elevation = -20;

    mjv_defaultOption(&opt);
    opt.flags[mjVIS_JOINT] = true;
    opt.frame = mjFRAME_BODY;

    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Config frame and joint scales
    m->vis.scale.framelength = 0.25f;
    m->vis.scale.framewidth = 0.01f;
    m->vis.scale.jointwidth = 0.02;

    glfwSwapInterval(0);

    fflush(stdout);
    int i = 0;
    bool contact_active = false;

    int pip_geom_id  = mj_name2id(m, mjOBJ_GEOM, "pip_geom");
    int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball_geom");

    double th1, th2, th3;
    double dth1, dth2, dth3;

    // Tuning knobs
    const double th2_rest = 0.0;
    const double th3_rest = 0.0;

    bool was_contact = false;
    double post_contact_timer = 0.0;
    const double recovery_time = 0.3;       // hold "open" command for 300 ms
    const double dt_outer = 0.001;          // matches single mj_step (timestep in XML = 0.001)

    while (!glfwWindowShouldClose(window)) {

        // Step simulation (ONCE per loop)
        mj_step(m, d);

        // ── Read joint states ──
        th1 = d->qpos[0];  // splay
        th2 = d->qpos[1];  // MCP
        th3 = d->qpos[2];  // PIP

        dth1 = d->qvel[0];
        dth2 = d->qvel[1];
        dth3 = d->qvel[2];

        d->ctrl[0] = 0;  // hold splay

        // ── Contact detection ──
        contact_active = false;
        for (int c_idx = 0; c_idx < d->ncon; c_idx++) {
            mjContact& con_data = d->contact[c_idx];
            if ((con_data.geom1 == pip_geom_id  && con_data.geom2 == ball_geom_id) ||
                (con_data.geom1 == ball_geom_id && con_data.geom2 == pip_geom_id)) {
                contact_active = true;
                break;
            }
        }

        // ── Control logic ──
        if (contact_active) {
            // On contact: command finger BACK to zero (pushes ball away)
            d->ctrl[1] = 0.0;
            d->ctrl[2] = 0.0;
            was_contact = true;
            post_contact_timer = recovery_time;

        } else if (post_contact_timer > 0.0) {
            // Just lost contact — keep commanding open to retract fully
            d->ctrl[1] = 0.0;
            d->ctrl[2] = 0.0;
            post_contact_timer -= dt_outer;

        } else {
            // Fully recovered — hold rest pose
            d->ctrl[1] = th2_rest;
            d->ctrl[2] = th3_rest;
        }

        // ── Render & print ──
        if (i % 50 == 0) {
            const char* branch = contact_active ? "CONTACT"
                                : (post_contact_timer > 0 ? "RECOVERY" : "REST");
            printf("Branch: %s timer=%.3f | Pos MCP: %.3f PIP: %.3f | Rod: %.3f\n",
                branch, post_contact_timer, d->qpos[1], d->qpos[2], d->qpos[3]);
            fflush(stdout);
        }

        if (i % 10 == 0) {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }

        i++;
    }

    // Cleanup
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}