#include "state_machine.h"


float calibration_hardstops_motors[3] = {0, 0, 0};
bool first_time = true;
uint8_t flex_ext_print_cycle = 0;
float mcp_temp = 0;
float dip_temp = 0;
float splay_temp = 0;
float mcp_v = 0;
float dip_v = 0;
float splay_v = 0;
float mcp_i = 0;
float dip_i = 0;
float splay_i = 0;
bool header_printed = false;

volatile CONTROL_MODES cur_system_state = CALIBRATION;
CONTROL_MODES prev_system_state = STARTUP;

k dip_control_force = {0.009, 0, 0.001};
k dip_control_force_current = {0.006, 0, 0};


k mcp_control_force = {0.009, 0, 0.001};
k mcp_control_force_current = {0.006, 0, 0.001};

uint32_t initial_max_force_time = 0;
uint32_t lastCmd = 0;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_3;


motor_axis m1;
motor_axis m2;
motor_axis m3;

bool is_force(CONTROL_MODES cur_state) {
 return (cur_state == STEP_FORCE || cur_state == MAX_FORCE);
};

void switch_calibration() {
  cur_system_state = CALIBRATION;
}
void switch_step_position() {
  // Serial.println("step_pos");
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

void switch_flex_ext() {
  cur_system_state = FLEX_EXT;
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
  if (!LOGGING) {Serial.println("CS pins set up");}
  spi_setup();
  if (!LOGGING) {Serial.println("SPI setup");}
  adsInit(PIN_CS_DIP);
  adsInit(PIN_CS_MCP);
  if (!LOGGING) {Serial.println("ADS initialize");}
  pinMode(PIN_CALIBRATION, INPUT);
  pinMode(PIN_STEP_POSITION, INPUT);
  pinMode(PIN_LISSAJOU_POSITION, INPUT);
  pinMode(PIN_ELLIPSE_POSITION, INPUT);
  pinMode(PIN_ZERO_FORCE, INPUT);
  pinMode(PIN_STEP_FORCE, INPUT);
  pinMode(PIN_MAX_FORCE, INPUT);
  pinMode(PIN_FLEX_EXT, INPUT);

}

void calibration_setup() {
  if (!LOGGING) {Serial.println("Under zero torque for 5 seconds. Move pulleys accordingly.");}
  uint32_t start_zero_force = millis();
  while ((millis() - start_zero_force) < 5000) {
    set_torque(&m1, 0);
    set_torque(&m2, 0);
    set_torque(&m3, 0);
  }
  delay(1000);
  if (!LOGGING) {Serial.println("Now calibrating");}
  // position calibration
  full_calibration(calibration_hardstops_motors, &m1, &m2, &m3);
  if (!LOGGING) {Serial.print("Calibration offsets: ");
  Serial.println();}
  for (int i = 0; i < 3; i++) {
    if (!LOGGING) {Serial.print(calibration_hardstops_motors[i]);
    Serial.println();}
  }
  // tension sensor calibration
  uint32_t start_zero_time = millis();
  while (millis() - start_zero_time < 1000) {
    set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
  zero_sensors();
}
}


void calibration_state() {
  // just stay in zero position after that
  set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
  delay(10);
}

void position_setup() {}

void force_setup() {
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY_DIP), isr_dip, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY_MCP), isr_mcp, FALLING);
}

void flex_ext_setup() {
  detachInterrupt(digitalPinToInterrupt(PIN_CALIBRATION));
  detachInterrupt(digitalPinToInterrupt(PIN_STEP_POSITION));
  detachInterrupt(digitalPinToInterrupt(PIN_LISSAJOU_POSITION));
  detachInterrupt(digitalPinToInterrupt(PIN_ELLIPSE_POSITION));
  detachInterrupt(digitalPinToInterrupt(PIN_ZERO_FORCE));
  detachInterrupt(digitalPinToInterrupt(PIN_STEP_FORCE));
  detachInterrupt(digitalPinToInterrupt(PIN_MAX_FORCE));
  detachInterrupt(digitalPinToInterrupt(PIN_FLEX_EXT));
}

void flex_ext_state() {
  angles target_joint = generate_flex_ext_sinusoid();
        bool motor_on[3] = {true, true, true};
        // Send to motors
        // set_joint_position_w_automatic_ff_torque(&m1,&m2,&m3,
        //                 target_joint,
        //                 calibration_hardstops_motors,
        //                 25.0f, 3.0f, motor_on);
        set_joint_position(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
    delay(10);
     CAN_message_t rxMsg;
     if (!header_printed) {
    Serial.println("time_ms,splay_temp,splay_vel,splay_torque,mcp_temp,mcp_vel,mcp_torque,dip_temp,dip_vel,dip_torque");
    header_printed = true;
}
  while (can_3.read(rxMsg)) {
      // Serial.print("GOT RESPONSE -->  ");
      int incoming_id = rxMsg.id;

      if (incoming_id == MOTOR1_ID){
        unpack_reply(&rxMsg, MOTOR1_ID); 
        splay_temp = Temperature;
        splay_v = speed;
        splay_i = torque;
        // print_data(rxMsg, MOTOR1_ID);
      }
      else if (incoming_id == MOTOR2_ID){
        unpack_reply(&rxMsg, MOTOR2_ID); 
        mcp_temp = Temperature;
        mcp_v = speed;
        mcp_i = torque;
        // print_data(rxMsg, MOTOR2_ID);
      }
      else if (incoming_id == MOTOR3_ID){
        unpack_reply(&rxMsg, MOTOR3_ID); 
        dip_temp = Temperature;
        dip_v = speed;
        dip_i = torque;
        // print_data(rxMsg, MOTOR3_ID);
      }     
  }
      // Serial.println();
    if (flex_ext_print_cycle >= 5) {
      flex_ext_print_cycle = 0;
      Serial.print(millis());
        Serial.print(",");
        Serial.print(splay_temp);
        Serial.print(",");
        Serial.print(splay_v);
        Serial.print(",");
        Serial.print(splay_i);
        Serial.print(",");
        Serial.print(mcp_temp);
        Serial.print(",");
        Serial.print(mcp_v);
        Serial.print(",");
        Serial.print(mcp_i);
        Serial.print(",");
        Serial.print(dip_temp);
        Serial.print(",");
        Serial.print(dip_v);
        Serial.print(",");
        Serial.println(dip_i);
    }
    else{
      flex_ext_print_cycle = flex_ext_print_cycle+1;
    }
}

void step_position_state() {
    
    lastCmd = millis();
    angles target_joint = generate_step_position_benchmark(0.5);
    bool motor_on[3] = {true, true, true};
    // Send to motors
    set_joint_position_w_automatic_ff_torque(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
    // set_joint_position(&m1, &m2, &m3,
    //                 target_joint,
    //                 calibration_hardstops_motors,
    //                 25.0f, 3.0f, motor_on);     
    delay(10);           
}

void lissajou_position_state() { 
        angles target_joint = generate_lissajous();
        bool motor_on[3] = {true, true, true};
        // Send to motors
        set_joint_position_w_automatic_ff_torque(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
        // set_joint_position(&m1,&m2,&m3,
        //                 target_joint,
        //                 calibration_hardstops_motors,
        //                 25.0f, 3.0f, motor_on);
    delay(10);
}

void ellipse_position_state() {
        angles target_joint = generate_ellipse(0.5f);
        bool motor_on[3] = {true, true, true};
        // Send to motors
        set_joint_position(&m1,&m2,&m3,
                        target_joint,
                        calibration_hardstops_motors,
                        25.0f, 3.0f, motor_on);
        delay(10);
}

void step_force_state() {
  //  uint32_t start = millis();
  //  while ((millis() - start) < 10000) {
  //  set_fingertip_force_zero(&m1,&m2,&m3, 1, 25.0, 3.0, mcp_control_force,dip_control_force,calibration_hardstops_motors);
  //  delay(10);
  //  }
  //  Serial.println("Change!!");
  // start = millis();
  //  while ((millis() - start) < 10000) {
  //  set_fingertip_force_zero(&m1,&m2,&m3, 3, 25.0, 3.0, mcp_control_force,dip_control_force,calibration_hardstops_motors);
  //  delay(10);
  //  }
  if (CURRENT_CONTROL_FOR_FORCE) {
    step_force_command(&m1,&m2,&m3, 25.0, 3.3, mcp_control_force_current, dip_control_force_current,calibration_hardstops_motors, 1, 3, 0.5);
  }
  else {
    step_force_command(&m1,&m2,&m3, 25.0, 3.3, mcp_control_force, dip_control_force,calibration_hardstops_motors, 1, 3, 0.5);
  }   
  delay(10);                         

  // Serial.print("MCP: ");
  // Serial.println(get_mcp_tension());
  // Serial.print("DIP: ");
  // Serial.println(get_dip_tension());
}

void zero_force_state() {
    set_torque(&m1, 0);
    set_torque(&m2, 0);
    set_torque(&m3, 0);
    update_sensor_readings(PIN_CS_MCP, zero_offset_mcp,  PIN_DRDY_MCP, &is_mcp);
    // Serial.println("DIP Tension Sensor Reading");
    update_sensor_readings(PIN_CS_DIP, zero_offset_dip,  PIN_DRDY_DIP, &is_dip);

    Serial.print(get_mcp_tension());
    Serial.print(", ");
    Serial.println(get_dip_tension());
    delay(10);
    // set_fingertip_force_zero(&m1, &m2, &m3, 0, 0, 0, mcp_control_force, dip_control_force, calibration_hardstops_motors);
}

void max_force_state() {
  float period = 45.0f; // num seconds to get to 20N
  float cur_torque = 0;
  uint32_t cur_time = millis();
  if ((cur_time -  initial_max_force_time)  >  (1000.0f*period)) {
    cur_torque = MAX_FINGERTIP_FORCE;
  }
  else {
    cur_torque = MAX_FINGERTIP_FORCE*(cur_time -  initial_max_force_time)/(1000.0f*period);
  }
  // Serial.println(cur_torque);
  delay(10);

  // Serial.println("force control");
  if (CURRENT_CONTROL_FOR_FORCE) {
      set_fingertip_force_zero_w_current_control(&m1,&m2,&m3, cur_torque, 25.0, 3.0, mcp_control_force_current,dip_control_force_current,calibration_hardstops_motors);
  }
  else {
    set_fingertip_force_zero(&m1,&m2,&m3, cur_torque, 25.0, 3.0, mcp_control_force,dip_control_force,calibration_hardstops_motors);
  }
}

void exit(uint32_t prev_state, uint32_t new_state) {
  if (!LOGGING) {Serial.print("Leaving State ");
  Serial.println(prev_state);
  Serial.print("Entering State ");
  Serial.println(new_state);}
  uint32_t start = millis();
  while ((millis() - start) < 500) {
    set_home_joint_position(&m1,&m2,&m3, calibration_hardstops_motors);
    delay(10);
  }
  if (prev_state > 3) {
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY_DIP));
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY_MCP));
  }
}

CONTROL_MODES get_state() {
  return cur_system_state;
}


void state_machine_cycle() {
  CONTROL_MODES cur_state = cur_system_state;
  // if changing states
  if (prev_system_state != cur_state) {
    if (!first_time)  {
      exit(prev_system_state, cur_state);
    }
    else {
      first_time = false;
      attachInterrupt(digitalPinToInterrupt(PIN_CALIBRATION), switch_calibration, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_STEP_POSITION), switch_step_position, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_LISSAJOU_POSITION), switch_lissajou_position, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_ELLIPSE_POSITION), switch_ellipse_position, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_ZERO_FORCE), switch_zero_force, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_STEP_FORCE), switch_step_force, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_MAX_FORCE), switch_max_force, FALLING);
      attachInterrupt(digitalPinToInterrupt(PIN_FLEX_EXT), switch_flex_ext, RISING);
    }
    if (cur_state == CALIBRATION) {
      calibration_setup();
    }
    else if (cur_state == FLEX_EXT) {
      flex_ext_setup();
    }
    else if (!is_force(cur_state)) {
      position_setup();
    }
    else {
      force_setup();
      initial_max_force_time = millis();
    }
  }
  //
  prev_system_state = cur_system_state;
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
    // Serial.println("Step Force");
    step_force_state();
    break;

  case MAX_FORCE:
    // Serial.println("Max Force");
    max_force_state();
    break;
  case FLEX_EXT:
    // Serial.println("Flex Ext");
    flex_ext_state();
    break;

  default:
    Serial.println("Unknown State");
    break;
}
  
}

