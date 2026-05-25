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
    m = mj_loadXML("../../finger_description/urdf/finger.xml", nullptr, error, 1000);
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
    // Better default camera
    cam.lookat[0] = 0.0;
    cam.lookat[1] = -0.350;
    cam.lookat[2] = 0.0;

    cam.distance = 1;
    cam.azimuth = 180;
    cam.elevation = -20;

    mjv_defaultOption(&opt);
    opt.flags[mjVIS_JOINT] = true;
    opt.frame = mjFRAME_BODY;   // other mjFRAME_GEOM
    // opt.frame = mjFRAME_WORLD;
    // opt.label = mjLABEL_BODY; //  mjLABEL_JOINT  mjLABEL_GEOM

    //  opt.flags[mjVIS_ACTUATOR] = true;
    //  opt.flags[mjVIS_CONTACTPOINT] = true;
    //  opt.flags[mjVIS_CONTACTFORCE] = true;

    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Config frame and joint scales
    m->vis.scale.framelength = 0.25f;
    m->vis.scale.framewidth = 0.01f;
    m->vis.scale.jointwidth = 0.02;
    
    glfwSwapInterval(0);  // disable vsync for faster rendering

    for (int j = 0; j < m->njnt; j++) {
        printf("Joint %d: %s  qpos_idx: %d  value: %f\n",
            j, mj_id2name(m, mjOBJ_JOINT, j),
            m->jnt_qposadr[j],
            d->qpos[m->jnt_qposadr[j]]);
        }
    
    mj_forward(m, d);
    // d->qpos[0] = 0.0;
    // mj_forward(m, d);
    int pip_id = mj_name2id(m, mjOBJ_BODY, "pip_link");
    printf("Fingertip pos: x=%f  y=%f  z=%f\n",
        d->xpos[pip_id*3+0],
        d->xpos[pip_id*3+1],
        d->xpos[pip_id*3+2]);

    printf("pendulum qpos0: %f\n", m->qpos0[0]);

    
    // ── Control loop ──
    // geom IDs of PIP and Ball for collision detection
    int pip_geom_id  = mj_name2id(m, mjOBJ_GEOM, "pip_geom");
    int ball_geom_id = mj_name2id(m, mjOBJ_GEOM, "ball_geom");

    int i = 0;        
    bool contact_active = false;
    printf("\n\n");
    fflush(stdout);

    while (!glfwWindowShouldClose(window)) {

        // ── SET ACTUATOR COMMANDS HERE ──
        d->ctrl[0] = 0.0;   // splay_motor (Nm)
        d->ctrl[1] = 0.0;   // mcp_motor   (Nm)
        d->ctrl[2] = 0.0;   // pip_motor   (Nm)

        // Step simulation
        mj_step(m, d);
        // simulate at real-time speed
        
        // Render
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();


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