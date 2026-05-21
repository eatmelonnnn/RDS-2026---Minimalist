#include "tensionsensor.h"


SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

volatile float tension_dip = 0.0f;
volatile float tension_mcp = 0.0f;

volatile bool is_dip = true;

int32_t zero_offset_dip = 0;
int32_t zero_offset_mcp = 0;



// ---------- SPI helpers ----------
inline void csLow(uint8_t PIN_CS)  { digitalWriteFast(PIN_CS, LOW);  delayMicroseconds(5); }
inline void csHigh(uint8_t PIN_CS) { delayMicroseconds(5); digitalWriteFast(PIN_CS, HIGH); }

void sendCmd(uint8_t cmd, uint8_t PIN_CS) {
  SPI1.beginTransaction(adsSPI);
  csLow(PIN_CS);
  SPI1.transfer(cmd);
  csHigh(PIN_CS);
  SPI1.endTransaction();
  delayMicroseconds(20);
}


void writeReg(uint8_t reg, uint8_t val, uint8_t PIN_CS) {
  SPI1.beginTransaction(adsSPI);
  csLow(PIN_CS);
  SPI1.transfer(CMD_WREG | (reg << 2));
  SPI1.transfer(val);
  csHigh(PIN_CS);
  SPI1.endTransaction();
  delayMicroseconds(20);
}

finger_tensions_torques generate_step_tensions() {
  finger_tensions_torques t;
  t.mcp_tension = 235;
  t.dip_tension = 145;
  t.mcp_torque = 1.04;
  t.dip_torque = 0.5;
  return t;
}

// ---------- DRDY wait ----------
bool waitForDRDY() {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (millis() - t0 > DRDY_TIMEOUT_MS) {
      Serial.println("WARNING: DRDY timeout - check wiring.");
      return false;
    }
  }
  return true;
}

// ---------- Read one fresh conversion ----------
int32_t readData(uint8_t PIN_CS) {
  if (!waitForDRDY()) return 0;
  SPI1.beginTransaction(adsSPI);
  csLow(PIN_CS);
  uint8_t b2 = SPI1.transfer(0x00);
  uint8_t b1 = SPI1.transfer(0x00);
  uint8_t b0 = SPI1.transfer(0x00);
  csHigh(PIN_CS);
  SPI1.endTransaction();
  int32_t r = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (r & 0x800000) r |= 0xFF000000;  // sign-extend 24->32 bit
  return r;
}

void isr_dip() {
  is_dip = true;
  update_sensor_readings(PIN_CS_DIP, zero_offset_dip);
}

void isr_mcp() {
  is_dip = false;
  update_sensor_readings(PIN_CS_MCP, zero_offset_mcp);
}

float get_dip_tension() {
  return tension_dip;
}

float get_mcp_tension() {
  return tension_mcp;
}

float pid_correction(float actual, float desired, float* prev_error, float *i_error, k pid, bool *initial_loop) {
  float e = actual - desired;
  float d_error;
  if (*initial_loop) {
    d_error = 0;
    *initial_loop = false;
  }
  else {
    d_error = e-*prev_error;
  }
  float correction = pid.p*e + pid.i*(*i_error) + pid.d*d_error;
  *i_error = constrain(*i_error + e, INTERROR_MIN, INTERROR_MAX);
  *prev_error = e;
  return correction;

}


void update_sensor_readings(uint8_t PIN_CS, int32_t zeroOffset) {
  int32_t raw = readData(PIN_CS);
  float tension = (raw - zeroOffset) / countsPerUnit;
  if (is_dip) {
    tension_dip = tension;
  }
  else {
    tension_mcp = tension;
  }
}

// ---------- Average N fresh conversions ----------
int32_t averageRaw(uint16_t samples,uint8_t PIN_CS) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readData(PIN_CS);
  }
  return (float)sum / (float)samples;
}

void zero_sensors() {
  zero_offset_mcp = averageRaw(NUM_ZERO_SAMPLES, PIN_CS_MCP);
  Serial.printf("Zero offset at startup: %.2f counts\n", zero_offset_mcp);
  zero_offset_dip = averageRaw(NUM_ZERO_SAMPLES, PIN_CS_DIP);
  Serial.printf("Zero offset at startup: %.2f counts\n", zero_offset_dip);
}

// ---------- ADS1220 init ----------
void adsInit(uint8_t PIN_CS) {
  sendCmd(CMD_RESET, PIN_CS);
  delay(50);
  writeReg(0x00, 0x0E, PIN_CS);  // AIN0/AIN1, gain=128, PGA on
  writeReg(0x01, 0x04, PIN_CS);  // 20 SPS, normal mode, continuous conversion
  writeReg(0x02, 0x70, PIN_CS);  // external ref REFP0/REFN0, 50/60Hz rejection
  writeReg(0x03, 0x00, PIN_CS);
  sendCmd(CMD_START, PIN_CS);
  delay(100);
}

void cs_setup(uint8_t PIN_CS){
    pinMode(PIN_CS, OUTPUT);
    pinMode(PIN_DRDY, INPUT);
    digitalWriteFast(PIN_CS, HIGH);
}

void spi_setup(){
    SPI1.setMOSI(PIN_MOSI);
    SPI1.setMISO(PIN_MISO);
    SPI1.setSCK(PIN_SCK);
    SPI1.begin();
    delay(50);
}

