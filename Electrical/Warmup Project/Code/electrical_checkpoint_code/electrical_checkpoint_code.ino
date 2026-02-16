#include <FlexCAN_T4.h>

#define BUILTIN_LED 13

// ---------------- CAN SETUP ----------------
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN_odrive;

// ---------------- ODRIVE CONFIG ----------------
#define ODRIVE_NODE_ID 0

#define CMD_SET_AXIS_STATE 0x07
#define CMD_SET_INPUT_MODE 0x0B
#define CMD_SET_INPUT_POS 0x0C
#define CMD_SET_INPUT_VOLTAGE 0x0E
#define CMD_SET_INPUT_VELOCITY 0x0D
#define CMD_HEARTBEAT 0x01
#define CMD_SET_CONTROLLER_GAINS 0x1A
#define CMD_SET_VEL_INTEGRATOR_GAIN 0x1B
#define CMD_GET_ENCODER_ESTIMATES 0x09

#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define AXIS_STATE_IDLE 1

#define BAUD_RATE 250000

// ---------------- Status Variables ----------------
uint32_t axis_error = 0;
uint8_t axis_state = 0;
uint8_t procedure_result = 0;
uint8_t traj_done = 0;

// ---------------- Modes ----------------
enum class ControlMode : uint32_t {
  VOLTAGE = 0,
  TORQUE = 1,
  VELOCITY = 2,
  POSITION = 3,
  NONE = 10
};

enum class InputMode : uint32_t {
  INACTIVE = 0,
  PASSTHROUGH = 1,
  VEL_RAMP = 2,
  POS_FILTER = 3,
  TRAP_TRAJ = 5,
  TORQUE_RAMP = 6
};

ControlMode command_type = ControlMode::POSITION;
bool heartbeat_commanded = true;

// ---------------- Sinusoid Parameters ----------------

float amplitude = 0.5f;     
float frequency = 0.3f;     // Hz

float encoder_position = 0.0f;
float encoder_velocity = 0.0f;

// ---------------- Helper Functions ----------------
void writeAndSendCAN(uint32_t cmd_id, uint8_t *data, uint8_t len) {
  CAN_message_t msg;
  msg.id = (ODRIVE_NODE_ID << 5) | cmd_id;
  msg.len = len;
  memcpy(msg.buf, data, len);
  CAN_odrive.write(msg);
}

void setControllerGains(float pos_gain, float vel_gain, float vel_integrator_gain) {
  uint8_t data[8];
  memcpy(data, &pos_gain, 4);
  memcpy(data + 4, &vel_gain, 4);
  writeAndSendCAN(CMD_SET_CONTROLLER_GAINS, data, 8);

  delay(5);

  uint8_t data_i[4];
  memcpy(data_i, &vel_integrator_gain, 4);
  writeAndSendCAN(CMD_SET_VEL_INTEGRATOR_GAIN, data_i, 4);
}

void setODriveAxisState(uint32_t state) {
  uint8_t data[4];
  memcpy(data, &state, 4);
  writeAndSendCAN(CMD_SET_AXIS_STATE, data, 4);
}

void setControlMode(uint32_t control_mode, uint32_t input_mode) {
  uint8_t data[8];
  memcpy(data, &control_mode, 4);
  memcpy(data + 4, &input_mode, 4);
  writeAndSendCAN(CMD_SET_INPUT_MODE, data, 8);
}

void setPosition(float turns) {
  uint8_t data[8];
  float vel_ff = 0.0f;
  memcpy(data, &turns, 4);
  memcpy(data + 4, &vel_ff, 4);
  writeAndSendCAN(CMD_SET_INPUT_POS, data, 8);
}

void setVelocity(float turns_per_sec) {
  uint8_t data[8];
  float torque_ff = 0.0f;
  memcpy(data, &turns_per_sec, 4);
  memcpy(data + 4, &torque_ff, 4);
  writeAndSendCAN(CMD_SET_INPUT_VELOCITY, data, 8);
}

void setVoltage(float volts) {
  uint8_t data[4];
  memcpy(data, &volts, 4);
  writeAndSendCAN(CMD_SET_INPUT_VOLTAGE, data, 4);
}

// ---------------- Setup ----------------
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  Serial.begin(115200);
  delay(500);

  CAN_odrive.begin();
  CAN_odrive.setBaudRate(BAUD_RATE);
  CAN_odrive.enableFIFO();

  switch (command_type) {
    case ControlMode::VOLTAGE:
      setControlMode(static_cast<uint32_t>(ControlMode::VOLTAGE),
                     static_cast<uint32_t>(InputMode::PASSTHROUGH));
      break;

    case ControlMode::VELOCITY:
      setControlMode(static_cast<uint32_t>(ControlMode::VELOCITY),
                     static_cast<uint32_t>(InputMode::VEL_RAMP));
      break;

    case ControlMode::TORQUE:
      setControlMode(static_cast<uint32_t>(ControlMode::TORQUE),
                     static_cast<uint32_t>(InputMode::TORQUE_RAMP));
      break;

    case ControlMode::POSITION:
      setControlMode(static_cast<uint32_t>(ControlMode::POSITION),
                     static_cast<uint32_t>(InputMode::POS_FILTER));
      break;

    default:
      break;
  }

  delay(20);

  if (command_type != ControlMode::NONE) {
    setControllerGains(20.0f, 0.5f, 2.0f);
    setODriveAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
}

// ---------------- Loop ----------------
void loop() {

  CAN_message_t msg;

  while (CAN_odrive.read(msg)) {

    uint32_t heartbeat_id = (ODRIVE_NODE_ID << 5) | CMD_HEARTBEAT;
    uint32_t encoder_id   = (ODRIVE_NODE_ID << 5) | CMD_GET_ENCODER_ESTIMATES;

    if (msg.id == heartbeat_id && msg.len == 8) {
      memcpy(&axis_error, &msg.buf[0], 4);
      axis_state = msg.buf[4];
      procedure_result = msg.buf[5];
      traj_done = msg.buf[6];
    }

    if (msg.id == encoder_id && msg.len == 8) {
      memcpy(&encoder_position, &msg.buf[0], 4);
      memcpy(&encoder_velocity, &msg.buf[4], 4);
    }
  }

  // -------- 100 Hz Sinusoid Command --------
  static uint32_t last_send = 0;
  static uint32_t start_time = millis();

  uint32_t now = millis();

  if (now - last_send >= 10) {

    last_send = now;

    float t = (now - start_time) / 1000.0f;

    float sine_value = amplitude *
                       sinf(2.0f * PI * frequency * t);

    if (axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {

      switch (command_type) {

        case ControlMode::VOLTAGE:
          setVoltage(sine_value);
          break;

        case ControlMode::VELOCITY:
          setVelocity(sine_value);
          break;

        case ControlMode::POSITION:
          setPosition(sine_value);
          break;

        case ControlMode::TORQUE:
          break;

        default:
          break;
      }
    }

    // ---- Serial Plotter Output ----
    Serial.print(sine_value);
    Serial.print(",");
    Serial.println(encoder_position);
  }
}
