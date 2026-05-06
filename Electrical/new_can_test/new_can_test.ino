#include <FlexCAN_T4.h>
#include <math.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

#define MOTOR1_ID 3 // struggling to move
#define MOTOR2_ID 4
#define MOTOR3_ID 5

#define P_MIN -12.5f //rad
#define P_MAX  12.5f
#define V_MIN -30.0f //rad/s
#define V_MAX  30.0f
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

angles poseA = {0.0f, 1.2f, 0.0f};
angles poseB = {0.0, 0.0f, 1.2f};
float pos1_unwrapped = 0;
float pos2_unwrapped = 0;
float pos3_unwrapped = 0;

tendonLengths multiply_AT(float th1, float th2, float th3);
angles joint_pos_to_motor_pos(angles jointpos, float calibration_offsets[3]);


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



float float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len){
  if (len>8){
    len = 8;
  } 

  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = 0;
  msg.len = len;

  for (int i=0; i<len; i++){
    msg.buf[i] = data[i];
  }

  can3.write(msg);
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

void enter_MIT_control_mode(){
  uint8_t control_mode_bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};
  comm_can_transmit_sid(MOTOR1_ID, control_mode_bytes, 8);
  delay(100);
  comm_can_transmit_sid(MOTOR2_ID, control_mode_bytes, 8);
  delay(100);
  comm_can_transmit_sid(MOTOR3_ID, control_mode_bytes, 8);

}

void exit_MIT_control_mode() {
    uint8_t bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    comm_can_transmit_sid(MOTOR1_ID, bytes, 8);
    delay(100);
    comm_can_transmit_sid(MOTOR2_ID, bytes, 8);
    delay(100);
    comm_can_transmit_sid(MOTOR3_ID, bytes, 8);
}

// ====== Motor Initialization ======
void setup_motor(motor_axis *axis, uint8_t id, int dir) {
    axis->controller_id = id;
    axis->motor_dir = dir;
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

    comm_can_transmit_sid(axis->controller_id, bytes, 8);
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

    comm_can_transmit_sid(axis->controller_id, bytes, 8);
}

void get_encoder_values(motor_axis *axis) {
  uint8_t enc_bytes[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  comm_can_transmit_sid(axis->controller_id, enc_bytes, 8);
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

motor_axis motor1;
motor_axis motor2;
motor_axis motor3;

float t0 = 0;

// void fake_calibration(float calibration_offsets[3], motor_axis *motor1, motor_axis *motor2, motor_axis *motor3) {
//   bool initialized = false;
//   CAN_message_t rxMsg;
//   while (!initialized) {
//         Serial.println("Motor1");
//         get_encoder_values(motor1);
//         if (can3.read(rxMsg)) {
          
//             uint32_t motor_id = motor1 ->controller_id;
//             if (rxMsg.id == motor_id) {
//                 unpack_reply(&rxMsg, motor_id);

//                 if (!isnan(position)) {

//                     initialized = true;
//                     calibration_offsets[0] = position;
//                 }
//             }
//         }
//     }
//     initialized = false;
//     while (!initialized) {
//         Serial.println("Motor2");
//         get_encoder_values(motor2);
//         if (can3.read(rxMsg)) {
          
//             uint32_t motor_id = motor2 ->controller_id;
//             if (rxMsg.id == motor_id) {
//                 unpack_reply(&rxMsg, motor_id);

//                 if (!isnan(position)) {

//                     initialized = true;
//                     calibration_offsets[1] = position;
//                 }
//             }
//         }
//     }
//     initialized = false;
//     while (!initialized) {
//         Serial.println("Motor3");
//         get_encoder_values(motor3);
//         if (can3.read(rxMsg)) {
          
//             uint32_t motor_id = motor3 ->controller_id;
//             if (rxMsg.id == motor_id) {
//                 unpack_reply(&rxMsg, motor_id);

//                 if (!isnan(position)) {

//                     initialized = true;
//                     calibration_offsets[2] = position;
//                 }
//             }
//         }
//     }
// }

float raw_calibrate_motor(motor_axis *axis, float velocity, uint32_t motor_id, float current_threshold) {
    CAN_message_t rxMsg;

    float pos_initial = 0;
    bool initialized = false;

    float calibration_offsets[3];


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
    for (int i = 0; i < 3; i++) {
        uint32_t start = millis();
        while (millis() - start < 500) {
            set_position(axis, pos_initial, 10, 2);
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
                calibration_offsets[i] = NAN;
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

                    set_velocity(axis, 0.0f, 1.0f);

                    calibration_offsets[i] = position;
                    if (!LOGGING) {
                    Serial.println(calibration_offsets[i]);
                    Serial.print("torque:");
                    Serial.println(current);}

                    done = true;
                }
            }
        }
    }

    float avg =
        (calibration_offsets[0] +
         calibration_offsets[1] +
         calibration_offsets[2]) / 3.0f;
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

// angles generate_joint_test(float t) {
//     angles j = {0.0f, 0.0f, 0.0f};
//     if      (t < 10.0f) j.th1 = 0.4f * sin(1.5f * t);
//     else if (t < 20.0f) j.th2 = 0.4f * sin(1.5f * t);
//     else                j.th3 = 0.4f * sin(1.5f * t);
//     return j;
// }

float calibration_hardstops[3] = {0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  if (!LOGGING) {Serial.println("Starting...");}
  delay(100);

  can3.begin();
  can3.setBaudRate(1000000);

  setup_motor(&motor1, MOTOR1_ID, 1);
  setup_motor(&motor2, MOTOR2_ID, 1);
  setup_motor(&motor3, MOTOR3_ID, 1);  

  exit_MIT_control_mode();
  delay(1000);
  enter_MIT_control_mode();

  if (!LOGGING) {Serial.println("Entered MIT mode");}
  // set_position(&motor1, 2.5f, 8.0f, 0.5f);
  // set_position(&motor2, 2.5f, 8.0f, 0.5f);
  //  set_position(&motor3, 2*PI, 8.0f, 0.5f);  
  delay(100);
  if (!LOGGING) {Serial.println("Starting Calibration");}
  
  full_calibration(calibration_hardstops, &motor1, &motor2, &motor3);
  if (!LOGGING) {Serial.print("Calibration offsets: ");}
  for (int i = 0; i < 3; i++) {
    if (!LOGGING) { Serial.print(calibration_hardstops[i]);}
  }
  delay(1000);
if (!LOGGING) {Serial.println("");}
  t0 = millis() / 1000.0f; 
  
}

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

float pos1 = 0, pos2 = 0, pos3 = 0;
float jtarget1 = 0, jtarget2 = 0, jtarget3 = 0;
angles target_joint;


void loop() {
  
  static uint32_t lastCmd = 0;
if (millis() - lastCmd >= 10) {  

    target_joint = generate_step_response(poseA, poseB, 0.5f); // 0.5 Hz
    bool motor_on[3] = {true, true, true};
    // Send to motors
    set_joint_position(&motor1, &motor2, &motor3,
                       target_joint,
                       calibration_hardstops,
                       25.0f, 3.0f, motor_on);

    
    jtarget1 = unwrap_angle(target_joint.th1);
    jtarget2 = unwrap_angle(target_joint.th2);
    jtarget3 = unwrap_angle(target_joint.th3);

    lastCmd = millis();
}
//   static uint32_t lastCmd = 0;
//   if (millis() - lastCmd >= 10) {
//       target1 = 0; //generate_sine_wave(&motor1, 0.5f, 1.5f);
//       target2 = generate_sine_wave(&motor2, 0.5f, 2.0f);
//       target3 = 0; //generate_sine_wave(&motor3, 0.5f, 2.5f);
//       lastCmd = millis();
//   }
  

  CAN_message_t rxMsg;
  while (can3.read(rxMsg)) {
      // Serial.print("GOT RESPONSE -->  ");
      int incoming_id = rxMsg.id;

      if (incoming_id == MOTOR1_ID){
        unpack_reply(&rxMsg, MOTOR1_ID); 
        pos1 = unwrap_angle(position);
        // print_data(rxMsg, MOTOR1_ID);
      }
      else if (incoming_id == MOTOR2_ID){
        unpack_reply(&rxMsg, MOTOR2_ID); 
        pos2 = unwrap_angle(position);
        // print_data(rxMsg, MOTOR2_ID);
      }
      else if (incoming_id == MOTOR3_ID){
        unpack_reply(&rxMsg, MOTOR3_ID); 
        pos3 = unwrap_angle(position);
        // print_data(rxMsg, MOTOR3_ID);
      }     

      // Serial.println();
  }

static uint32_t lastPrint = 0;
if (millis() - lastPrint >= 100) {
    angles ajointpos = motor_pos_to_joint_pos(pos1,pos2, pos3, calibration_hardstops);
    angles tmotorpos = joint_pos_to_motor_pos(target_joint, calibration_hardstops);
        Serial.print(millis());
    Serial.print(",");
    Serial.print(jtarget1);
    Serial.print(",");
    Serial.print(ajointpos.th1);
    Serial.print(",");
    Serial.print(jtarget2);
    Serial.print(",");
    Serial.print(ajointpos.th2);
    Serial.print(",");
    Serial.print(jtarget3);
    Serial.print(",");
    Serial.print(ajointpos.th3);
    Serial.print(",");
    Serial.print(tmotorpos.th1);
    Serial.print(",");
    Serial.print(pos1);
    Serial.print(",");
    Serial.print(tmotorpos.th2);
    Serial.print(",");
    Serial.print(pos2);
    Serial.print(",");
    Serial.print(tmotorpos.th3);
    Serial.print(",");
    Serial.println(pos3);

    lastPrint = millis();
}
  // delay(100);
}



// void loop() {
//     float t = (millis() / 1000.0f) - t0;

//     static uint32_t lastCmd = 0;
//     if (millis() - lastCmd >= 10) {
//         target_joint = generate_joint_test(t);

//         // Print the joint target and the resulting motor commands for verification
//         angles motor_targets = joint_pos_to_motor_pos(target_joint, calibration_hardstops);
//         Serial.print("t:"); Serial.print(t, 2);
//         Serial.print("\tJ1:"); Serial.print(target_joint.th1, 3);
//         Serial.print("\tJ2:"); Serial.print(target_joint.th2, 3);
//         Serial.print("\tJ3:"); Serial.print(target_joint.th3, 3);
//         Serial.print("\tM1:"); Serial.print(motor_targets.th1, 3);
//         Serial.print("\tM2:"); Serial.print(motor_targets.th2, 3);
//         Serial.print("\tM3:"); Serial.print(motor_targets.th3, 3);

//         set_joint_position(&motor1, &motor2, &motor3,
//                            target_joint, calibration_hardstops,
//                            10.0f, 0.5f);
//         lastCmd = millis();
//     }

//     CAN_message_t rxMsg;
//     while (can3.read(rxMsg)) {
//         if (rxMsg.id == MOTOR1_ID) { unpack_reply(&rxMsg, MOTOR1_ID); pos1 = position; }
//         else if (rxMsg.id == MOTOR2_ID) { unpack_reply(&rxMsg, MOTOR2_ID); pos2 = position; }
//         else if (rxMsg.id == MOTOR3_ID) { unpack_reply(&rxMsg, MOTOR3_ID); pos3 = position; }
//     }

//     // Actual motor positions for serial plotter
//     Serial.print("\tA1:"); Serial.print(pos1, 3);
//     Serial.print("\tA2:"); Serial.print(pos2, 3);
//     Serial.print("\tA3:"); Serial.println(pos3, 3);
// }


