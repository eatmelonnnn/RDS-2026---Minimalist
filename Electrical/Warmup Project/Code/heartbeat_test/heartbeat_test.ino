#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can0;

const uint8_t NODE_ID = 0;   // change if needed


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Listening for ODrive heartbeat...");

  Can0.begin();
  Can0.setBaudRate(250000);   // must match ODrive
}

void loop() {
  CAN_message_t msg;

  if (Can0.read(msg)) {

    uint32_t expected_id = (NODE_ID << 5) | 0x01;

    if (msg.id == expected_id) {

      Serial.println("Heartbeat received!");

      uint32_t axis_error;
      memcpy(&axis_error, &msg.buf[0], 4);

      uint8_t axis_state = msg.buf[4];
      uint8_t procedure_result = msg.buf[6];
      uint8_t traj_done = msg.buf[7];

      Serial.print("Axis Error: ");
      Serial.println(axis_error);

      Serial.print("Axis State: ");
      Serial.println(axis_state);

      Serial.print("Procedure Result: ");
      Serial.println(procedure_result);

      Serial.print("Trajectory Done: ");
      Serial.println(traj_done);

      Serial.println("------------------------");
    }
  }
}
