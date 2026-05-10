/*
 * ADS1220 Tension Reader - Teensy 4.1, SPI1
 *
 * Wiring:
 *   Load cell Red   (EXC+)  -> ADS1220 AVDD (5V)
 *   Load cell Black (EXC-)  -> ADS1220 AGND
 *   Load cell White (SIG+)  -> ADS1220 AIN0
 *   Load cell Green (SIG-)  -> ADS1220 AIN1
 *   ADS1220 REFP0           -> EXC+ node (5V)
 *   ADS1220 REFN0           -> AGND
 *   ADS1220 /DRDY           -> Teensy pin 29
 *
 * Automatically tares on startup, then streams tension readings at 20 SPS.
 * Ensure NO load is applied at power-on so the auto-tare is accurate.
 */

#include <SPI.h>

// ---------- Pins ----------
constexpr uint8_t PIN_CS   = 38;
constexpr uint8_t PIN_MOSI = 26;
constexpr uint8_t PIN_MISO = 39;
constexpr uint8_t PIN_SCK  = 27;
constexpr uint8_t PIN_DRDY = 29;

SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

// ---------- ADS1220 commands ----------
constexpr uint8_t CMD_RESET = 0x06;
constexpr uint8_t CMD_START = 0x08;
constexpr uint8_t CMD_WREG  = 0x40;

// ---------- Calibration constants ----------
// Update countsPerUnit if you recalibrate with the calibration tool.
// const float countsPerUnit =  3116.4614f;
// const float countsPerUnit =  2886.4956;
const float countsPerUnit =  2418.9988;
const char* unitLabel     = "N";
float       zeroOffset    = 0.0f;   // set automatically at startup

// ---------- DRDY timeout ----------
constexpr uint32_t DRDY_TIMEOUT_MS = 500;

// ---------- SPI helpers ----------
inline void csLow()  { digitalWriteFast(PIN_CS, LOW);  delayMicroseconds(5); }
inline void csHigh() { delayMicroseconds(5); digitalWriteFast(PIN_CS, HIGH); }

void sendCmd(uint8_t cmd) {
  SPI1.beginTransaction(adsSPI);
  csLow();
  SPI1.transfer(cmd);
  csHigh();
  SPI1.endTransaction();
  delayMicroseconds(20);
}

void writeReg(uint8_t reg, uint8_t val) {
  SPI1.beginTransaction(adsSPI);
  csLow();
  SPI1.transfer(CMD_WREG | (reg << 2));
  SPI1.transfer(val);
  csHigh();
  SPI1.endTransaction();
  delayMicroseconds(20);
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
int32_t readData() {
  if (!waitForDRDY()) return 0;
  SPI1.beginTransaction(adsSPI);
  csLow();
  uint8_t b2 = SPI1.transfer(0x00);
  uint8_t b1 = SPI1.transfer(0x00);
  uint8_t b0 = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  int32_t r = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (r & 0x800000) r |= 0xFF000000;  // sign-extend 24->32 bit
  return r;
}

// ---------- Average N fresh conversions ----------
float averageRaw(uint16_t samples) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readData();
  }
  return (float)sum / (float)samples;
}

// ---------- ADS1220 init ----------
void adsInit() {
  sendCmd(CMD_RESET);
  delay(50);
  writeReg(0x00, 0x0E);  // AIN0/AIN1, gain=128, PGA on
  writeReg(0x01, 0x04);  // 20 SPS, normal mode, continuous conversion
  writeReg(0x02, 0x70);  // external ref REFP0/REFN0, 50/60Hz rejection
  writeReg(0x03, 0x00);
  sendCmd(CMD_START);
  delay(100);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS,   OUTPUT);
  pinMode(PIN_DRDY, INPUT);
  digitalWriteFast(PIN_CS, HIGH);

  SPI1.setMOSI(PIN_MOSI);
  SPI1.setMISO(PIN_MISO);
  SPI1.setSCK(PIN_SCK);
  SPI1.begin();
  delay(50);

  adsInit();
  zeroOffset = averageRaw(64);
  Serial.printf("Zero offset at startup: %.2f counts\n", zeroOffset);
  // Auto-tare: average 64 readings at startup as the zero reference
  zeroOffset = averageRaw(64);
}

// ---------- Loop ----------
void loop() {
  if (digitalRead(PIN_DRDY) == LOW) {
    int32_t raw = readData();
    float   val = (raw - zeroOffset) / countsPerUnit;
    Serial.printf("raw=%9ld  delta=%9.0f  %.4f %s\n",
                  raw, raw - zeroOffset, val, unitLabel);
  }
}
