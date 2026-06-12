#include "motors.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

angles poseA = {0.0f, 1.2f, 0.0f};
angles poseB = {0.0, 0.0f, 1.2f};
float pos1_unwrapped = 0;
float pos2_unwrapped = 0;
float pos3_unwrapped = 0;

motor_axis motor1;
motor_axis motor2;
motor_axis motor3;

float calibration_hardstops[3] = {0, 0, 0};

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
  //  set_position(&motor3, 2*PI, 8.0f, 0.5f);  
  delay(100);
  Serial.println("Starting Calibration");

  full_calibration(calibration_hardstops, &motor1, &motor2, &motor3);
  Serial.print("Calibration offsets: ");
  Serial.println();
  for (int i = 0; i < 3; i++) {
    Serial.print(calibration_hardstops[i]);
    Serial.println();
  }
  delay(1000);
}

void loop(){

}