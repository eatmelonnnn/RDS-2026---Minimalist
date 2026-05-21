#include "motors.h"
#include "tensionsensor.h"

#define FINGER_POSITION_CONTROL_MODE 1

k dip_control = {1, 0, 0};

k mcp_control = {1, 0, 0};

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

angles poseA = {0.0f, PI/3, 0.0f};
angles poseB = {0.0, 0.0f, PI/3};
// angles poseA = {0.0f, 0.0f, 0.0f};
// angles poseB = {0.0, 0.0f, 0.0f};
float pos1_unwrapped = 0;
float pos2_unwrapped = 0;
float pos3_unwrapped = 0;

motor_axis motor1;
motor_axis motor2;
motor_axis motor3;

float tension_offset_dip  = 0.0;

float calibration_hardstops[3] = {0.46, -1.01, 0.94};

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

  if (FINGER_POSITION_CONTROL_MODE) {
    enter_MIT_control_mode();
    
    if (!LOGGING) {Serial.println("Entered MIT mode");}
    // set_position(&motor1, 2.5f, 8.0f, 0.5f);
    // set_position(&motor2, 2.5f, 8.0f, 0.5f);
    //  set_position(&motor3, 2*PI, 8.0f, 0.5f);  
    delay(100);
    if (!LOGGING) {Serial.println("Starting Calibration");}
    
    /*
    full_calibration(calibration_hardstops, &motor1, &motor2, &motor3);
    if (!LOGGING) {Serial.print("Calibration offsets: ");}
    for (int i = 0; i < 3; i++) {
      if (!LOGGING) { Serial.print(calibration_hardstops[i]);}
    }
    delay(1000);
    if (!LOGGING) {Serial.println("");}
    t0 = millis() / 1000.0f; 
    */
    
  }
else {
    cs_setup(PIN_CS_DIP);
    cs_setup(PIN_CS_MCP);
    spi_setup();
    adsInit(PIN_CS_DIP);
    adsInit(PIN_CS_MCP);
    zero_sensors();
    attachInterrupt(digitalPinToInterrupt(PIN_CS_DIP), isr_dip, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_CS_MCP), isr_mcp, FALLING);
  }
}

float pos1 = 0, pos2 = 0, pos3 = 0;
float jtarget1 = 0, jtarget2 = 0, jtarget3 = 0;
angles target_joint;


void loop() {

if (FINGER_POSITION_CONTROL_MODE) {
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
}
else {
  // TODO: actual loop
  
  static bool initial_loop_mcp = true;
  static float prev_error_mcp;
  static float i_error_mcp = 0;
  finger_tensions_torques desired = generate_step_tensions();
  float torque_mcp = pid_correction(get_mcp_tension(),
   desired.mcp_tension,
    &prev_error_mcp,
     &i_error_mcp,
      mcp_control,
       &initial_loop_mcp) + desired.mcp_torque;
  set_torque(&motor2, torque_mcp);

}
}