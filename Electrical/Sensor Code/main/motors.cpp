#include "motors.h"
// CAN COMMUNICATION
float float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len, uint8_t flag_extended){
  if (len>8){
    len = 8;
  } 

  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = flag_extended;
  msg.len = len;

  for (int i=0; i<len; i++){
    msg.buf[i] = data[i];
  }

  can3.write(msg);
}

// ====== Process Incoming CAN Messages ======
void process_can_message(motor_axis *axis, const CAN_message_t &rxMsg) {
    if ((rxMsg.id & 0xFF) == axis->controller_id) {
        axis->motor_pos = (rxMsg.buf[0] << 8) | rxMsg.buf[1];
        axis->motor_spd = (rxMsg.buf[2] << 8) | rxMsg.buf[3];
        axis->motor_cur = (rxMsg.buf[4] << 8) | rxMsg.buf[5];
        axis->motor_temp = rxMsg.buf[6];
        axis->motor_error = rxMsg.buf[7];
    }
}

// ====== Debug Function to Print Motor Data ======
void debug_motor(motor_axis *axis) {
    Serial.print("Motor ID: "); Serial.println(axis->controller_id);
    Serial.print("Position: "); Serial.println(axis->motor_pos);
    Serial.print("Speed: "); Serial.println(axis->motor_spd);
    Serial.print("Current: "); Serial.println(axis->motor_cur);
    Serial.print("Temperature: "); Serial.println(axis->motor_temp);
    Serial.print("Error: "); Serial.println(axis->motor_error);
}

//All numbers in the packet are converted to floating point by the following function. 
float uint_to_float(int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void get_encoder_values(motor_axis *axis) {
  uint8_t enc_bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  comm_can_transmit_sid(axis->controller_id, enc_bytes, 8, 0);
}

void unpack_reply(CAN_message_t *RxMessage, int id_desired){
  /// unpack ints from can buffer ///
  
  id = RxMessage->buf[0]; //driver id number
  p_int = ( RxMessage->buf[1]<<8) | RxMessage->buf[2]; // motor position buf
  v_int = ( RxMessage->buf[3]<<4)|( RxMessage->buf[4]>>4); //motor speed value
  i_int = (( RxMessage->buf[4]&0xF)<<8)| RxMessage->buf[5]; //motor torque value
  T_int = RxMessage->buf[6];

  /// convert ints to floats ///
  //float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  //float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  //float i = uint_to_float(i_int, -T_MAX, T_MAX, 12);
  //float T =T_int;

  if(id == id_desired){
    position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
    torque = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    Temperature = T_int-40; //temperature range-40~215
  }
}

float position;
float speed ;
float torque ;
float Temperature ;
int id;
int p_int;
int v_int;
int i_int;
float T_int;

void print_data(CAN_message_t rxMsg, int motorid){
    unpack_reply(&rxMsg, motorid);
    Serial.print("Id =");
    Serial.print(id);

    Serial.print("  Position = ");
    Serial.print(position);

    Serial.print("  speed = ");
    Serial.print(speed);

    Serial.print("  torque = ");
    Serial.print(torque);

    Serial.print("  Temperature = ");
    Serial.print(Temperature);
}

// ================================================ MOTORS ===============================================================
void enter_MIT_control_mode(){
  uint8_t control_mode_bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};
  comm_can_transmit_sid(MOTOR1_ID, control_mode_bytes, 8, 0);
  delay(100);
  comm_can_transmit_sid(MOTOR2_ID, control_mode_bytes, 8, 0);
  delay(100);
  comm_can_transmit_sid(MOTOR3_ID, control_mode_bytes, 8,0);

}

void exit_MIT_control_mode() {
    uint8_t bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    comm_can_transmit_sid(MOTOR1_ID, bytes, 8,0);
    delay(100);
    comm_can_transmit_sid(MOTOR2_ID, bytes, 8,0);
    delay(100);
    comm_can_transmit_sid(MOTOR3_ID, bytes, 8,0);
}

// ====== Motor Initialization ======
void setup_motor(motor_axis *axis, uint8_t id, int dir) {
    axis->controller_id = id;
    axis->motor_dir = dir;
}

void set_position(motor_axis *axis, float pos_rad, float kp, float kd) {
    // Clamp inputs to valid ranges
    pos_rad = constrain(pos_rad, P_MIN, P_MAX);
    kp      = constrain(kp, 0.0f, 500.0f);
    kd      = constrain(kd, 0.0f, 5.0f);

    // Convert to 16/12-bit unsigned ints
    uint16_t pos_int = float_to_uint(pos_rad, P_MIN, P_MAX, 16);
    uint16_t vel_int = float_to_uint(0.0f,    V_MIN, V_MAX, 12);  // zero velocity
    uint16_t kp_int  = float_to_uint(kp,      0.0f,  500.0f, 12);
    uint16_t kd_int  = float_to_uint(kd,      0.0f,  5.0f,   12);
    uint16_t tor_int = float_to_uint(0.0f,    T_MIN, T_MAX,  12);  // zero feedforward torque

    // Pack into 8 bytes — AK-series MIT mode format
    uint8_t bytes[8];
    bytes[0] = pos_int >> 8;
    bytes[1] = pos_int & 0xFF;
    bytes[2] = vel_int >> 4;
    bytes[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8);
    bytes[4] = kp_int & 0xFF;
    bytes[5] = kd_int >> 4;
    bytes[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8);
    bytes[7] = tor_int & 0xFF;

    comm_can_transmit_sid(axis->controller_id, bytes, 8, 0);
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void set_current(motor_axis *axis, float current) {
    current = constrain(current, I_MIN, I_MAX);
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_sid(axis->controller_id |
    ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, 4, 1);
}

void set_torque(motor_axis *axis, float torque) {
    float current = torque / KT;
    set_current(axis,current);
}

float calibration_hardstops_zero_motors(float JOINT_HARDSTOP, float motor_position, float joint_radius, float motor_radius) {
    return motor_position - (JOINT_HARDSTOP*joint_radius/motor_radius);
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

float unwrap_angle(float current) {
    while ((current > 2*PI) || (current < -2*PI)) {
    if (current > 2*PI) {
        current -= 2.0f * PI;
    } 
    else if (current < -2*PI) {
        current += 2.0f * PI;
    }
    }
    return current;
}

// Converts joint-space angles to motor positions and commands all three motors.
// calibration_offsets must be the array populated by full_calibration().
void set_joint_position(motor_axis *m1, motor_axis *m2, motor_axis *m3,
                        angles joint_pos, float calibration_offsets[3],
                        float kp, float kd, bool setting_motor[3]) {
    angles motor_pos = joint_pos_to_motor_pos(joint_pos, calibration_offsets);
    if (setting_motor[0]) {
        set_position(m1, motor_pos.th1, kp, kd);
    }
     if (setting_motor[1]) {
        set_position(m2, motor_pos.th2, kp, kd);
    }
    if (setting_motor[2]) {
    set_position(m3, motor_pos.th3, kp, kd);
    }
}

void set_velocity(motor_axis *axis, float vel_rad_s, float kd) {
    // Clamp inputs
    vel_rad_s = constrain(vel_rad_s, V_MIN, V_MAX);
    kd        = constrain(kd, 0.0f, 5.0f);

    // Convert to packed integers
    uint16_t pos_int = float_to_uint(0.0f, P_MIN, P_MAX, 16);   // no position control
    uint16_t vel_int = float_to_uint(vel_rad_s, V_MIN, V_MAX, 12);
    uint16_t kp_int  = float_to_uint(0.0f, 0.0f, 500.0f, 12);   // disable position loop
    uint16_t kd_int  = float_to_uint(kd, 0.0f, 5.0f, 12);
    uint16_t tor_int = float_to_uint(0.0f, T_MIN, T_MAX, 12);   // no feedforward torque

    // Pack bytes (same MIT format)
    uint8_t bytes[8];
    bytes[0] = pos_int >> 8;
    bytes[1] = pos_int & 0xFF;
    bytes[2] = vel_int >> 4;
    bytes[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8);
    bytes[4] = kp_int & 0xFF;
    bytes[5] = kd_int >> 4;
    bytes[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8);
    bytes[7] = tor_int & 0xFF;

    comm_can_transmit_sid(axis->controller_id, bytes, 8, 0);
}

float raw_calibrate_motor(motor_axis *axis, float velocity, uint32_t motor_id, float current_threshold) {
    CAN_message_t rxMsg;

    float pos_initial = 0;
    bool initialized = false;

    float calibration_sum = 0;


    // ===== 1. Get initial position =====
    if (!LOGGING) {Serial.println("Getting initial position");}
    while (!initialized) {
        get_encoder_values(axis);
        if (can3.read(rxMsg)) {
            // Serial.println("Message Read");
            // Serial.print("CAN ID: ");
            // Serial.print(rxMsg.id);
            // Serial.print(" | DATA ID: ");
            // Serial.println(rxMsg.buf[0]);
            
            if (rxMsg.id == motor_id) {
                unpack_reply(&rxMsg, motor_id);

                if (!isnan(position)) {
                    pos_initial = position;
                    initialized = true;
                }
            }
        }
    }

    if (!LOGGING) {Serial.print("Initial position: ");
    Serial.println(pos_initial);}
    // float curr_pos = pos_initial;

    // ===== 2. Calibration loop =====
    for (int i = 0; i < NUM_CAL_CYCLES; i++) {
        uint32_t start = millis();
        while (millis() - start < 500) {
            if (i == 0) {
            set_position(axis, pos_initial, 10, 2);
            }
            else{
                set_position(axis,calibration_sum/(i*NUM_CAL_MEASURES_PER_CYCLE) - velocity*0.8, 10, 2);
            }
            delay(5);  // ~200 Hz
        }
        start = millis();
        bool torque_zero = false;
        while (!torque_zero) {
            set_position(axis, pos_initial, 0, 0);
            delay(5);  // ~200 Hz
            if (can3.read(rxMsg) && rxMsg.id == motor_id) {
                unpack_reply(&rxMsg, motor_id);
                if (!LOGGING) {
                Serial.print("Reset torque: ");
                Serial.println(torque);}

                if (abs(torque) < 0.1f) {
                    torque_zero = true;
                    break;
                }
    }

        }
                    while (can3.read(rxMsg)) {
                // just discard
            }
        set_velocity(axis, velocity, 2.0f);
        delay(1000);
        bool done = false;
        uint32_t start_time = millis();
        uint32_t timeout_ms = 30000;


        while (!done) {
            // set_position(axis, curr_pos, 0.5, 1.5);
            // curr_pos = curr_pos + 0.02;
            set_velocity(axis, velocity, 2.0f);
            // timeout
            if (millis() - start_time > timeout_ms) {
                if (!LOGGING) {Serial.println("TIMEOUT");}
                set_velocity(axis, 0.0f, 2.0f);
                calibration_sum = NAN;
                done = true;
                break;
            }
            
            if (can3.read(rxMsg) && rxMsg.id == motor_id) {
                unpack_reply(&rxMsg, motor_id);
                
                float current = torque;
                static uint32_t lastprint = millis();
                if ((millis() - lastprint) > 200) {
                    // Serial.println(current);
                    lastprint= millis();
                }
                

                if (abs(current) > current_threshold) {
                    if (!LOGGING) {Serial.println("HARD STOP");}
                    for (int i = 0; i < NUM_CAL_MEASURES_PER_CYCLE; i++) {
                        set_velocity(axis, 0.0f, 1.0f);
                        calibration_sum = calibration_sum + position;
                    }
                    if (!LOGGING) {
                    Serial.println(position);
                    Serial.print("torque:");
                    Serial.println(current);}

                    done = true;
                }
            }
        }
    }

    float avg = calibration_sum/ (NUM_CAL_CYCLES * NUM_CAL_MEASURES_PER_CYCLE);
    if (!LOGGING) {
    Serial.print("Final calibration: ");
    Serial.println(avg);}

    return avg;
}



void full_calibration(float calibration_offsets[3], motor_axis *motor1, motor_axis *motor2, motor_axis *motor3) {
  if (!LOGGING) {Serial.println("Calibrating Splay");}
  float motor_pos_1 = raw_calibrate_motor(motor1, CALIBRATION_VELOCITY, MOTOR1_ID, 1.1f);
  calibration_offsets[0] = calibration_hardstops_zero_motors(HARDSTOP_JOINT_1, motor_pos_1, rw, Rm1);
  angles zero;
  zero.th1 = 0;
  zero.th2 = 0;
  zero.th3 = 0;
  bool motor_on[3] = {true, false, false};
  
    uint32_t start = millis();
    while (millis() - start < CAL_DELAY) {
        set_joint_position(motor1, motor2, motor3, zero,
                        calibration_offsets,
                        10.0f, 2.0f, motor_on);
        delay(5);  // ~200 Hz
    }
    
  if (!LOGGING) {Serial.println("Calibrating MCP");}
  float motor_pos_2 = raw_calibrate_motor(motor2, CALIBRATION_VELOCITY, MOTOR2_ID, 1.3f);
  calibration_offsets[1] = calibration_hardstops_zero_motors(HARDSTOP_JOINT_2, motor_pos_2, rj, Rm2);
  motor_on[1] = true;
   start = millis();
    while (millis() - start < CAL_DELAY) {
        set_joint_position(motor1, motor2, motor3, zero,
                        calibration_offsets,
                        10.0f, 2.0f, motor_on);
        delay(5);  // ~200 Hz
    }
    if (!LOGGING) {Serial.println("Calibrating DIP");}
  float motor_pos_3 = raw_calibrate_motor(motor3, -CALIBRATION_VELOCITY, MOTOR3_ID, 1.5f);
  calibration_offsets[2] = calibration_hardstops_zero_motors(HARDSTOP_JOINT_3, motor_pos_3, rj, Rm3);
  motor_on[2] = true;
  start = millis();
    while (millis() - start < CAL_DELAY) {
        set_joint_position(motor1, motor2, motor3, zero,
                        calibration_offsets,
                        10.0f, 2.0f, motor_on);
        delay(5);  // ~200 Hz
    }
}

float t0 = 0;
float generate_sine_wave(motor_axis *axis, float amplitude, float angular_frequency){
  float t = (millis() / 1000.0f) - t0;
  float target_position = amplitude*sin(angular_frequency*t);
  set_position(axis, target_position, 10, 0.5);

  return target_position;
}

angles generate_step_response(angles a, angles b, float freq) {
    uint32_t period_ms = (uint32_t)(1000.0f / freq);
    uint32_t t = millis() % period_ms;

    if (t < period_ms / 2) {
        return a;
    } else {
        return b;
    }
}

