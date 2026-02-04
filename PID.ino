#include <iq_module_communication.hpp>
#include <temperature_monitor_uc_client.hpp> // Already included, which is great

// --- Serial and Client Setup ---
IqSerial ser(Serial1);
MultiTurnAngleControlClient m(0);
PowerMonitorClient pwr(0);
TemperatureMonitorUcClient tmp(0); // Added client for temperature sensor

const uint32_t BAUD = 115200;

// ---- Tune here ----
float TARGET_RAD = 6.0f;
float Kp = 4.8f;
float Ki = 1.2f;
float Kd = 1.6f;

float V_MAX = 12.0f;
float A_MAX = 40.0f;
float V_GUARD = 100.0f;
float E_HOLD  = 0.02f;
float V_HOLD  = 0.12f;
// --------------------

// --- Timing and State Variables ---
elapsedMicros ctrl_tick;
const uint32_t CTRL_DT_US = 4000;
const float    CTRL_DT    = 0.004f;

float ang=0.0f, vel_raw=0.0f, amps=0.0f, temp_C=0.0f; // Added temp_C variable
float u_cmd=0.0f;
float e_int=0.0f;

// --- Filters ---
const float VEL_ALPHA = 0.10f;
float vel_filt=0.0f;

// --- Logging ---
uint32_t t0_ms = 0;
elapsedMicros log_tick;
float prev_vel_filt = 0.0f;

// --- Utility Functions ---
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}

void setup(){
  Serial.begin(115200);
  while(!Serial) {}

  ser.begin(BAUD);

  // --- Homing and Zeroing ---
  for(int i=0;i<6;i++){ ser.set(m.ctrl_velocity_, 0.0f); delay(15); }
  ser.set(m.sample_zero_angle_);
  ser.set(m.obs_angular_displacement_, 0.0f);

  // --- Start Timers and Logging ---
  ctrl_tick = 0;
  log_tick  = 0;
  t0_ms = millis();

  // CSV header now includes Temperature
  Serial.println("t_ms,ref_rad,ang_rad,vel_raw,vel_filt,accel_filt,u_cmd,err_rad,current_A,temp_C");
}

void loop(){
  // --- Telemetry (best-effort) ---
  float a,v,I,T; // Added variable T to temporarily hold temperature
  if (ser.get(m.obs_angular_displacement_, a)) ang = a;
  if (ser.get(m.obs_angular_velocity_,     v)) vel_raw = v;
  if (ser.get(pwr.amps_,                   I)) amps = I;
  if (ser.get(tmp.temp_celsius_,           T)) temp_C = T; // Get the temperature

  // --- Control (250 Hz) ---
  if (ctrl_tick >= CTRL_DT_US){
    ctrl_tick -= CTRL_DT_US;

    // 1) Filter velocity
    vel_filt = VEL_ALPHA*vel_raw + (1.0f - VEL_ALPHA)*vel_filt;

    // 2) PID
    float e     = TARGET_RAD - ang;
    float dterm = -Kd * vel_filt;
    e_int += e * CTRL_DT;

    float i_cap = (Ki > 1e-6f) ? (V_MAX / Ki) : 0.0f;
    if (Ki > 0.0f) e_int = clampf(e_int, -i_cap, i_cap);

    float u_des = Kp*e + Ki*e_int + dterm;

    // 3) Near-target hold
    if (fabsf(e) < E_HOLD && fabsf(vel_filt) < V_HOLD) {
      u_des = 0.0f;
    }

    // 4) Speed clamp + accel limit
    u_des = clampf(u_des, -V_MAX, V_MAX);
    float du    = u_des - u_cmd;
    float duMax = A_MAX * CTRL_DT;
    if      (du >  duMax) du =  duMax;
    else if (du < -duMax) du = -duMax;
    u_cmd += du;

    // 5) Guards
    if (fabsf(vel_filt) > V_GUARD) {
      u_cmd = 0.0f;
    }

    // 6) Send command
    ser.set(m.ctrl_velocity_, u_cmd);
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
      Serial.print(t_ms);           Serial.print(',');
      Serial.print(TARGET_RAD,6);   Serial.print(',');
      Serial.print(ang,6);          Serial.print(',');
      Serial.print(vel_raw,6);      Serial.print(',');
      Serial.print(vel_filt,6);     Serial.print(',');
      Serial.print(accel_filt,6);   Serial.print(',');
      Serial.print(u_cmd,6);        Serial.print(',');
      Serial.print(err,6);          Serial.print(',');
      Serial.print(amps,6);         Serial.print(',');
      Serial.println(temp_C,2);     // Added temperature to the CSV output
    } else {
      ser.set(m.ctrl_velocity_, 0.0f);
      while(1){ delay(1000); }
    }
  }
}
