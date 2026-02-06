#include <arduino.h>
#include <FlexCAN_T4.h> // Library for CAN Bus on Teensy

// --- CAN Setup for M2006/C610 ---
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
CAN_message_t msg_rx;
CAN_message_t msg_tx;

// M2006 Constants
const int C610_ID = 1;
const float GEAR_RATIO = 36.0f;
const float CTS_PER_REV = 8192.0f; // Rotor counts
const float TOTAL_CTS = CTS_PER_REV * GEAR_RATIO;
const float MAX_CURRENT_AMPS = 10.0f; // C610 continuous max approx 10A
const float CURRENT_SCALE = 1000.0f;  // 1A = 1000 units sent to C610

// --- Tune here (Note: PID Units changed from Velocity to Amps!) ---
float TARGET_RAD = 6.0f;
float Kp = 2.5f;   // Amps per Radian error
float Ki = 0.05f;  // Amps per Radian-second
float Kd = 0.1f;   // Amps per (Rad/s)

float I_MAX = 9.0f; // Max current (Amps) - Safety limit
float A_MAX_RAMP = 20.0f; // Amps/sec ramp limit
float E_HOLD  = 0.02f;
float I_HOLD  = 0.10f; // Current deadband
// --------------------

// --- Timing and State Variables ---
elapsedMicros ctrl_tick;
const uint32_t CTRL_DT_US = 4000; // 250Hz
const float    CTRL_DT    = 0.004f;

// State
float ang = 0.0f;
float vel_raw = 0.0f;
float amps = 0.0f;
float temp_C = 0.0f;
float u_cmd = 0.0f; // Now represents Current in Amps
float e_int = 0.0f;

// Encoder Unwrapping
uint16_t raw_ang_prev = 0;
int32_t rotations = 0;
bool first_reading = true;

// --- Filters ---
const float VEL_ALPHA = 0.10f;
float vel_filt = 0.0f;

// --- Logging ---
uint32_t t0_ms = 0;
elapsedMicros log_tick;
float prev_vel_filt = 0.0f;

// --- Utility Functions ---
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}

// Helper to send current to C610
void send_current(float current_amps) {
  // Clamp hard safety limit
  current_amps = clampf(current_amps, -MAX_CURRENT_AMPS, MAX_CURRENT_AMPS);
  
  // Convert to C610 units (-10000 to 10000)
  int16_t command = (int16_t)(current_amps * CURRENT_SCALE);
  
  msg_tx.id = 0x200; // ID 0x200 controls ESC IDs 1-4
  msg_tx.len = 8;
  
  // C610 expects Big Endian. ESC ID 1 is bytes 0 and 1.
  msg_tx.buf[0] = (command >> 8) & 0xFF;
  msg_tx.buf[1] = command & 0xFF;
  // Zero out other motor slots
  memset(&msg_tx.buf[2], 0, 6);
  
  Can1.write(msg_tx);
}

void setup(){
  Serial.begin(115200);
  while(!Serial && millis() < 3000); // Wait up to 3s

  // Init CAN
  Can1.begin();
  Can1.setBaudRate(1000000); // C610 requires 1Mbps
  
  // Setup Zeroing (Reset internal counters)
  ang = 0.0f;
  rotations = 0;
  first_reading = true;

  // --- Start Timers and Logging ---
  ctrl_tick = 0;
  log_tick  = 0;
  t0_ms = millis();

  Serial.println("t_ms,ref_rad,ang_rad,vel_raw,vel_filt,accel_filt,u_cmd_Amps,err_rad,current_A,temp_C");
}

void loop(){
  // --- Telemetry (Read CAN) ---
  if (Can1.read(msg_rx)) {
    // 0x200 + ID (1) = 0x201
    if (msg_rx.id == 0x201) { 
      // 1. Parse Mechanical Angle (0-8191)
      uint16_t raw_ang = (msg_rx.buf[0] << 8) | msg_rx.buf[1];
      
      // 2. Handle Wrapping
      if (!first_reading) {
        int diff = raw_ang - raw_ang_prev;
        if (diff < -4000) rotations++;
        else if (diff > 4000) rotations--;
      } else {
        first_reading = false;
        // Optional: Auto-zero on startup
        // rotations = 0; // or offset logic
      }
      raw_ang_prev = raw_ang;

      // 3. Calculate Output Shaft Angle (Radians)
      float total_counts = (float)rotations * CTS_PER_REV + (float)raw_ang;
      // Divide by Gear Ratio to get Output Shaft angle
      ang = (total_counts / TOTAL_CTS) * 2.0f * PI; 

      // 4. Parse Velocity (RPM) -> Rad/s
      int16_t raw_rpm = (msg_rx.buf[2] << 8) | msg_rx.buf[3];
      // RPM / 60 * 2Pi / GearRatio
      vel_raw = ((float)raw_rpm / 60.0f * 2.0f * PI) / GEAR_RATIO;

      // 5. Parse Current (Not actual Amps, somewhat arbitrary units from C610)
      // Usually mapped similarly to input, we approximate /1000.0f
      int16_t raw_current = (msg_rx.buf[4] << 8) | msg_rx.buf[5];
      amps = (float)raw_current / CURRENT_SCALE;

      // 6. Parse Temp
      temp_C = (float)msg_rx.buf[6];
    }
  }

  // --- Control (250 Hz) ---
  if (ctrl_tick >= CTRL_DT_US){
    ctrl_tick -= CTRL_DT_US;

    // 1) Filter velocity
    vel_filt = VEL_ALPHA*vel_raw + (1.0f - VEL_ALPHA)*vel_filt;

    // 2) PID 
    // NOTE: This calculates Target Current directly (Position -> Current Loop)
    // The previous code calculated Velocity, but M2006 takes Current.
    float e      = TARGET_RAD - ang;
    float dterm = -Kd * vel_filt;
    e_int += e * CTRL_DT;

    float i_cap = (Ki > 1e-6f) ? (I_MAX / Ki) : 0.0f;
    if (Ki > 0.0f) e_int = clampf(e_int, -i_cap, i_cap);

    float u_des = Kp*e + Ki*e_int + dterm;

    // 3) Near-target hold
    if (fabsf(e) < E_HOLD && fabsf(vel_filt) < 0.1f) {
      u_des = 0.0f;
      e_int = 0.0f; // Reset integrator to prevent windup drift at hold
    }

    // 4) Current Clamp + Ramp Limit
    u_des = clampf(u_des, -I_MAX, I_MAX);
    
    float du    = u_des - u_cmd;
    float duMax = A_MAX_RAMP * CTRL_DT; // Limits change in Current (Jerk-ish)
    
    if      (du >  duMax) du =  duMax;
    else if (du < -duMax) du = -duMax;
    
    u_cmd += du;

    // 5) Send command (CAN)
    send_current(u_cmd);
  }

  // --- CSV logging (~250 Hz, for ~10 s only) ---
  if (log_tick >= 4000){
    float dt = log_tick * 1e-6f;
    log_tick = 0;

    float accel_filt = (dt > 1e-6f) ? (vel_filt - prev_vel_filt) / dt : 0.0f;
    prev_vel_filt = vel_filt;

    uint32_t t_ms = millis() - t0_ms;
    float err = TARGET_RAD - ang;

    if (t_ms <= 10000) {
      Serial.print(t_ms);            Serial.print(',');
      Serial.print(TARGET_RAD,4);    Serial.print(',');
      Serial.print(ang,4);           Serial.print(',');
      Serial.print(vel_raw,4);       Serial.print(',');
      Serial.print(vel_filt,4);      Serial.print(',');
      Serial.print(accel_filt,4);    Serial.print(',');
      Serial.print(u_cmd,4);         Serial.print(',');
      Serial.print(err,4);           Serial.print(',');
      Serial.print(amps,4);          Serial.print(',');
      Serial.println(temp_C,1);
    } else {
      send_current(0.0f); // Stop motor
      while(1){ delay(1000); }
    }
  }
}