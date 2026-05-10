#ifndef MOTORS_H
#define MOTORS_H

#include <FlexCAN_T4.h>
#include <math.h>
#include <stdint.h>

extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

#define MOTOR1_ID 3 // struggling to move
#define MOTOR2_ID 4
#define MOTOR3_ID 5

#define KT 0.127f

#define P_MIN -12.5f //rad
#define P_MAX  12.5f
#define V_MIN -30.0f //rad/s
#define V_MAX  30.0f
#define I_MIN -18.0f
#define I_MAX  18.0f
#define T_MIN -18.0f
#define T_MAX  18.0f
#define LOGGING true

#define CAL_DELAY 1000

#define Rm1 0.005f
#define Rm2 0.005f
#define Rm3 0.005f


#define rj 0.008f
#define rw 0.010f
#define rs (0.0191f/2.0f)

#define HARDSTOP_JOINT_1 -1.05f //32645023f
#define HARDSTOP_JOINT_2 -1.37079633f
#define HARDSTOP_JOINT_3 1.97079633f

#define CALIBRATION_VELOCITY -0.5f

// for printing data
extern float position;
extern float speed ;
extern float torque ;
extern float Temperature ;
extern int id;
extern int p_int;
extern int v_int;
extern int i_int;
extern float T_int;

//for different CAN packets
typedef enum {
CAN_PACKET_SET_DUTY = 0, //Duty Cycle Mode
CAN_PACKET_SET_CURRENT, //Current Loop Mode
CAN_PACKET_SET_CURRENT_BRAKE, // Current BrakeMode
CAN_PACKET_SET_RPM,
CAN_PACKET_SET_POS,
CAN_PACKET_SET_ORIGIN_HERE,
CAN_PACKET_SET_POS_SPD,
CAN_PACKET_SET_mit=8,
} CAN_PACKET_ID;

// for sine wave 
extern float t0;

struct tendonLengths {
    float l1;
    float l2;
};

struct angles {
    float th1;
    float th2;
    float th3;
};


struct motor_axis {
    uint8_t packet[8];
    float serial_pos;
    float motor_vel;
    int motor_dir;
    uint16_t controller_id;
    int16_t motor_pos;
    int16_t motor_spd;
    int16_t motor_cur;
    int8_t motor_temp;
    int8_t motor_error;
};

// =============== CAN FUNCTION DEFINITIONS ==========================================
float float_to_uint(float x, float x_min, float x_max, int bits);
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
void process_can_message(motor_axis *axis, const CAN_message_t &rxMsg);
void debug_motor(motor_axis *axis);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void get_encoder_values(motor_axis *axis);
void unpack_reply(CAN_message_t *RxMessage, int id_desired);
void print_data(CAN_message_t rxMsg, int motorid);

// =============== MOTOR CONTROL FUNCTION DEFINITIONS ================================
void enter_MIT_control_mode();
void exit_MIT_control_mode();
void setup_motor(motor_axis *axis, uint8_t id, int dir);
void set_position(motor_axis *axis, float pos_rad, float kp, float kd);

float calibration_hardstops_zero_motors(float JOINT_HARDSTOP, float motor_position, float joint_radius, float motor_radius);
tendonLengths multiply_AT(float th1, float th2, float th3);
angles joint_pos_to_motor_pos(angles jointpos, float calibration_offsets[3]);
angles motor_pos_to_joint_pos(float pos1, float pos2, float pos3, float calibration_offsets[3]);
float unwrap_angle(float current);
void set_joint_position(motor_axis *m1, motor_axis *m2, motor_axis *m3,
                        angles joint_pos, float calibration_offsets[3],
                        float kp, float kd, bool setting_motor[3]);
void set_velocity(motor_axis *axis, float vel_rad_s, float kd);
float raw_calibrate_motor(motor_axis *axis, float velocity, uint32_t motor_id, float current_threshold);
void full_calibration(float calibration_offsets[3], motor_axis *motor1, motor_axis *motor2, motor_axis *motor3);

float generate_sine_wave(motor_axis *axis, float amplitude, float angular_frequency);
angles generate_step_response(angles a, angles b, float freq);

#endif
