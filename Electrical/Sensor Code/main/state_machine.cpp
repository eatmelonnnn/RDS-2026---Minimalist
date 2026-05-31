#include "state_machine.h"

float calibration_hardstops_motors[3] = {0, 0, 0};

volatile CONTROL_MODES cur_system_state = CALIBRATION;
CONTROL_MODES prev_system_state = STARTUP;

k dip_control_force = {0.005, 0, 0};

k mcp_control_force = {0.005, 0, 0};

uint32_t initial_max_force_time = 0;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_3;


motor_axis m1;
motor_axis m2;
motor_axis m3;


void switch_calibration() {
  cur_system_state = CALIBRATION;
}
void switch_step_position() {
  cur_system_state = STEP_POSITION;
}
void switch_lissajou_position() {
  cur_system_state = LISSAJOU_POSITION;
}
void switch_ellipse_position() {
  cur_system_state = ELLIPSE_POSITION;
}
void switch_zero_force() {
  cur_system_state = ZERO_FORCE;
}
void switch_step_force() {
  cur_system_state = STEP_FORCE;
}
void switch_max_force() {
  cur_system_state = MAX_FORCE;
}


void initial_universal_setup() {
  
  Serial.begin(115200);  
  if (!LOGGING) {Serial.println("Starting...");}
  delay(100);

  can_3.begin();
  can_3.setBaudRate(1000000);

  setup_motor(&m1, MOTOR1_ID, 1);
  setup_motor(&m2, MOTOR2_ID, 1);
  setup_motor(&m3, MOTOR3_ID, 1);  

  exit_MIT_control_mode();
  delay(1000);
  enter_MIT_control_mode();
    
  if (!LOGGING) {Serial.println("Entered MIT mode");}
  // set_position(&motor1, 2.5f, 8.0f, 0.5f);
  // set_position(&motor2, 2.5f, 8.0f, 0.5f);
  //  set_position(&motor3, 2*PI, 8.0f, 0.5f);  
  delay(100);
  cs_setup(PIN_CS_DIP,  PIN_DRDY_DIP);
  cs_setup(PIN_CS_MCP, PIN_DRDY_MCP);
  Serial.println("CS pins set up");
  spi_setup();
  Serial.println("SPI setup");
  adsInit(PIN_CS_DIP);
  adsInit(PIN_CS_MCP);
  Serial.println("ADS initialize");
}

void calibration_setup() {
  Serial.println("Under zero torque for 5 seconds. Move pulleys accordingly.");
  uint32_t start_zero_force = millis();
  while ((millis() - start_zero_force) < 5000) {
    set_torque(&m1, 0);
    set_torque(&m2, 0);
    set_torque(&m3, 0);
  }
  // position calibration
  full_calibration(calibration_hardstops_motors, &m1, &m2, &m3);
  Serial.print("Calibration offsets: ");
  Serial.println();
  for (int i = 0; i < 3; i++) {
    Serial.print(calibration_hardstops_motors[i]);
    Serial.println();
  }
  // tension sensor calibration
  static uint32_t start_zero_time = 0;
  while (millis() - start_zero_time < 1000) {
    set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
  zero_sensors();
}
}


void calibration_state() {
  // just stay in zero position after that
  set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
}

void position_setup() {}

void force_setup() {
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY_DIP), isr_dip, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY_MCP), isr_mcp, FALLING);
}

void step_position_state() {
  static uint32_t lastCmd = 0;
    if (millis() - lastCmd >= 10) { 
    angles target_joint = generate_step_position_benchmark(1);
    bool motor_on[3] = {true, true, true};
    // Send to motors
    set_joint_position(&m1, &m2, &m3,
                    target_joint,
                    calibration_hardstops_motors,
                    25.0f, 3.0f, motor_on);
    }                  
}

void lissajou_position_state() {
  static uint32_t lastCmd = 0;
    if (millis() - lastCmd >= 10) {  
        angles target_joint = generate_lissajous();
        bool motor_on[3] = {true, true, true};
        // Send to motors
        set_joint_position(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
    }
}

void ellipse_position_state() {
  static uint32_t lastCmd = 0;
    if (millis() - lastCmd >= 10) {  
        angles target_joint = generate_ellipse(0.5f);
        bool motor_on[3] = {true, true, true};
        // Send to motors
        set_joint_position(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
    }
}

void step_force_state() {
  
  step_force_command(&m1,&m2,&m3, 25.0, 3.0, mcp_control_force, dip_control_force,calibration_hardstops_motors, 1, 3, 1);
                               
  //   set_torque(&motor2, torque_val);
  //   CAN_message_t  rxMsg;
  // if (can_3.read(rxMsg) && rxMsg.id == MOTOR2_ID) {
  //                 unpack_reply(&rxMsg, MOTOR2_ID);
  //                 Serial.println(torque);}

  // Serial.print("MCP: ");
  // Serial.println(get_mcp_tension());
  // Serial.print("DIP: ");
  // Serial.println(get_dip_tension());
}

void zero_force_state() {
    set_torque(&m1, 0);
    set_torque(&m2, 0);
    set_torque(&m3, 0);
    // set_fingertip_force_zero(&m1, &m2, &m3, 0, 0, 0, mcp_control_force, dip_control_force, calibration_hardstops_motors);
}

void max_force_state() {
  float period = 25.0f; // num seconds to get to 20N
  float cur_torque = 0;
  uint32_t cur_time = millis();
  if ((cur_time -  initial_max_force_time)  >  (1000.0f*period)) {
    cur_torque = MAX_FINGERTIP_FORCE;
  }
  else {
    cur_torque = MAX_FINGERTIP_FORCE*(cur_time -  initial_max_force_time)/(1000.0f*period);
  }

  // Serial.println("force control");
  set_fingertip_force_zero(&m1,&m2,&m3, cur_torque, 25.0, 3.0, mcp_control_force,dip_control_force,calibration_hardstops_motors);
}

void exit(uint32_t prev_state, uint32_t new_state) {
  Serial.print("Leaving State ");
  Serial.println(prev_state);
  Serial.print("Entering State ");
  Serial.println(new_state);
  uint32_t start = millis();
  while ((millis() - start) < 500) {
    set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
  }
  if (prev_state > 3) {
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY_DIP));
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY_MCP));
  }
}


void state_machine_cycle() {
  CONTROL_MODES cur_state = cur_system_state;
  // if changing states
  if (prev_system_state != cur_state) {
    exit(prev_system_state, cur_state);
    if (cur_state == CALIBRATION) {
      calibration_setup();
    }
    else if (cur_state < 4) {
      position_setup();
    }
    else {
      force_setup();
      initial_max_force_time = millis();
    }
  }
  // run current state
  switch (cur_system_state) {
  case CALIBRATION:
    calibration_state();
    break;

  case STEP_POSITION:
    step_position_state();
    break;

  case LISSAJOU_POSITION:
    lissajou_position_state();
    break;

  case ELLIPSE_POSITION:
    ellipse_position_state();
    break;

  case ZERO_FORCE:
    zero_force_state();
    break;

  case STEP_FORCE:
    step_force_state();
    break;

  case MAX_FORCE:
    max_force_state();
    break;

  default:
    Serial.println("Unknown State");
    break;
}
  
}

