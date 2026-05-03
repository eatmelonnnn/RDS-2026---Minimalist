#include <FlexCAN_T4.h>
#include <math.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

#define MOTOR1_ID 3 // struggling to move
#define MOTOR2_ID 4
#define MOTOR3_ID 5

#define P_MIN -12.5f
#define P_MAX  12.5f
#define V_MIN -30.0f
#define V_MAX  30.0f
#define T_MIN -18.0f
#define T_MAX  18.0f

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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
  Serial.println("Starting...");
  delay(100);

  can3.begin();
  can3.setBaudRate(1000000);

  setup_motor(&motor1, MOTOR1_ID, 1);
  setup_motor(&motor2, MOTOR2_ID, 1);
  setup_motor(&motor3, MOTOR3_ID, 1);  

  exit_MIT_control_mode();
  delay(1000);
  enter_MIT_control_mode();

  Serial.println("Entered MIT mode");
  // set_position(&motor1, 2.5f, 8.0f, 0.5f);
  // set_position(&motor2, 2.5f, 8.0f, 0.5f);
  set_position(&motor3, 2*PI, 8.0f, 0.5f);  
  delay(100);
  t0 = millis() / 1000.0f; 

}

float generate_sine_wave(motor_axis *axis, float amplitude, float angular_frequency){
  float t = (millis() / 1000.0f) - t0;
  float target_position = amplitude*sin(angular_frequency*t);
  set_position(axis, target_position, 10, 0.5);

  return target_position;
}

float pos1 = 0, pos2 = 0, pos3 = 0;
float target1 = 0, target2 = 0, target3 = 0;

void loop() {  
  /*
  static uint32_t lastCmd = 0;
  if (millis() - lastCmd >= 10) {
      target1 = generate_sine_wave(&motor1, 0.5f, 1.5f);
      target2 = generate_sine_wave(&motor2, 0.5f, 2.0f);
      target3 = generate_sine_wave(&motor3, 0.5f, 2.5f);
      lastCmd = millis();
  }
  */

  CAN_message_t rxMsg;
  while (can3.read(rxMsg)) {
      // Serial.print("GOT RESPONSE -->  ");
      int incoming_id = rxMsg.buf[0];

      if (incoming_id == MOTOR1_ID){
        unpack_reply(&rxMsg, MOTOR1_ID); 
        pos1 = position;
        // print_data(rxMsg, MOTOR1_ID);
      }
      else if (incoming_id == MOTOR2_ID){
        unpack_reply(&rxMsg, MOTOR2_ID); 
        pos2 = position;
        // print_data(rxMsg, MOTOR2_ID);
      }
      else if (incoming_id == MOTOR3_ID){
        unpack_reply(&rxMsg, MOTOR3_ID); 
        pos3 = position;
        // print_data(rxMsg, MOTOR3_ID);
      }     

      // Serial.println();
  }
  // for serial plotter and monitor
  Serial.print("T1:"); Serial.print(target1);
  Serial.print("\tA1:"); Serial.print(pos1);
  Serial.print("\tT2:"); Serial.print(target2);
  Serial.print("\tA2:"); Serial.print(pos2);
  Serial.print("\tT3:"); Serial.print(target3);
  Serial.print("\tA3:"); Serial.println(pos3);

  // delay(100);
}




