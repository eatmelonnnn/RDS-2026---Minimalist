#include "kinematics.h"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cstring>

#include <chrono>
#include <thread>

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
    cam.lookat[0] = 0.250;
    cam.lookat[1] = 0.10;
    cam.lookat[2] = 0.10;

    cam.distance = 1;
    cam.azimuth = 120;
    cam.elevation = -20;

    mjv_defaultOption(&opt);
    // opt.flags[mjVIS_JOINT] = true;
    // opt.frame = mjFRAME_BODY;
    opt.frame = mjFRAME_WORLD;  

    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Config frame and joint scales
    m->vis.scale.framelength = 0.25f;
    m->vis.scale.framewidth = 0.01f;
    m->vis.scale.jointwidth = 0.02;

    glfwSwapInterval(0);  // disable vsync for faster rendering

    fflush(stdout);
    int i = 0;
    bool contact_active = false;

    int pip_geom_id  = mj_name2id(m, mjOBJ_GEOM, "pip_geom");
    int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball_geom");

    double th1, th2, th3;
    double dth1, dth2, dth3;

    // ── Tuning knobs ──
    // Rest pose is FULLY OPEN (0) so the finger retracts clear of the ball.
    // Contact pose is a distinct curl so the swat is visible.
    const double th2_rest = 0.0;   // MCP open
    const double th3_rest = 0.0;   // PIP open

    const double th2_hit  = 0.8;   // MCP curl on contact
    const double th3_hit  = 1.2;   // PIP curl on contact

    auto sim_start = std::chrono::steady_clock::now();
    double sim_time = 0.0;

    while (!glfwWindowShouldClose(window)) {

        // Step simulation (ONCE per loop)
        mj_step(m, d);

        sim_time += m->opt.timestep;
        auto target = sim_start + std::chrono::duration<double>(sim_time);
        std::this_thread::sleep_until(target);

        // ── Read joint states ──
        th1 = d->qpos[0];  // splay
        th2 = d->qpos[1];  // MCP
        th3 = d->qpos[2];  // PIP

        dth1 = d->qvel[0];
        dth2 = d->qvel[1];
        dth3 = d->qvel[2];

        d->ctrl[0] = 0;  // hold splay

        // ── State machine for active flick ──
        // States: 0 = READY (waiting), 1 = FLICK (swatting), 2 = RESET (returning)
        static int state = 0;
        static double state_timer = 0.0;
        const double dt = m->opt.timestep;

        // Get pendulum joint angle to know where the ball is.
        // qpos indices from your printout: [0]=splay, [1]=mcp, [2]=pip, [3]=rod_hinge
        double rod_angle = d->qpos[3];
        double rod_vel   = d->qvel[3];

        // Flick trigger: ball is swinging INTO the finger and is close.
        // Tune these thresholds by watching the Rod values in your printout.
        const double flick_trigger_angle = -1.3;   // start swat just before contact
        const bool   ball_approaching    = (rod_vel > 0.5);  // swinging toward finger

        const double flick_duration = 0.1;   // 80 ms swat
        const double reset_duration = 0.4;    // 400 ms hold-open before re-arming

        switch (state) {
        case 0:  // READY
            d->ctrl[1] = 0.0;
            d->ctrl[2] = 0.0;
            if (rod_angle > flick_trigger_angle && !contact_active) {
                state = 1;
                state_timer = 0.0;
            }
            break;

        case 1:  // FLICK
            d->ctrl[1] = 0.8;
            d->ctrl[2] = 1.2;
            contact_active = false;  // reset contact flag for this state
            state_timer += dt;
            if (state_timer >= flick_duration) {
                state = 2;
                state_timer = 0.0;
            }
            break;

        case 2:  // RESET
            d->ctrl[1] = 0.0;
            d->ctrl[2] = 0.0;
            state_timer += dt;
            if (state_timer >= reset_duration) {
                state = 0;
            }
            break;
        }
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

        d->ctrl[0] = 0.0;  // hold splay always

        // Update printout to show state
        if (i % 50 == 0) {
            const char* state_name = (state == 0) ? "READY" : (state == 1) ? "FLICK" : "RESET";
            printf("State: %-6s | Rod: %.3f (vel %.3f) | MCP: %.3f PIP: %.3f\n",
                state_name, d->qpos[3], d->qvel[3], d->qpos[1], d->qpos[2]);
            printf("Ball approaching = %d ", ball_approaching);
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