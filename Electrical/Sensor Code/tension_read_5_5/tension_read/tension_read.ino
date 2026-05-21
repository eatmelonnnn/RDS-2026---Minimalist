/*
 * Dual ADS1220 Tension Reader - Teensy 4.1
 *
 * Both ADCs share SPI1 (MOSI=26, MISO=39, SCK=27).
 * They are addressed by separate CS and DRDY lines:
 *   ADC0:  CS=10, DRDY=9
 *   ADC1:  CS=38, DRDY=29
 *
 * Both auto-tare at startup (no load applied), then stream paired readings.
 */

#include <SPI.h>

// ---------- Shared SPI1 bus pins ----------
constexpr uint8_t PIN_MOSI = 26;
constexpr uint8_t PIN_MISO = 39;
constexpr uint8_t PIN_SCK  = 27;

// ---------- Per-chip pins ----------
constexpr uint8_t PIN_CS0   = 10;
constexpr uint8_t PIN_DRDY0 = 9;

constexpr uint8_t PIN_CS1   = 38;
constexpr uint8_t PIN_DRDY1 = 29;

SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

// ---------- ADS1220 commands ----------
constexpr uint8_t CMD_RESET = 0x06;
constexpr uint8_t CMD_START = 0x08;
constexpr uint8_t CMD_WREG  = 0x40;

// ---------- Calibration ----------
const float countsPerUnit = 4157.5908f;
const char* unitLabel     = "N";

float zeroOffset0 = 0.0f;
float zeroOffset1 = 0.0f;

// ---------- DRDY timeout ----------
constexpr uint32_t DRDY_TIMEOUT_MS = 500;

// ---------- Per-channel state ----------
struct AdcChannel {
  SPIClass& bus;
  uint8_t   csPin;
  uint8_t   drdyPin;
  const char* name;
};

AdcChannel adc0 = { SPI1, PIN_CS0, PIN_DRDY0, "ADC0" };
AdcChannel adc1 = { SPI1, PIN_CS1, PIN_DRDY1, "ADC1" };

// ---------- Low-level SPI helpers (per channel) ----------
inline void csLow (const AdcChannel& a) { digitalWriteFast(a.csPin, LOW);  delayMicroseconds(5); }
inline void csHigh(const AdcChannel& a) { delayMicroseconds(5); digitalWriteFast(a.csPin, HIGH); }

void sendCmd(AdcChannel& a, uint8_t cmd) {
  a.bus.beginTransaction(adsSPI);
  csLow(a);
  a.bus.transfer(cmd);
  csHigh(a);
  a.bus.endTransaction();
  delayMicroseconds(20);
}

void writeReg(AdcChannel& a, uint8_t reg, uint8_t val) {
  a.bus.beginTransaction(adsSPI);
  csLow(a);
  a.bus.transfer(CMD_WREG | (reg << 2));
  a.bus.transfer(val);
  csHigh(a);
  a.bus.endTransaction();
  delayMicroseconds(20);
}

bool waitForDRDY(AdcChannel& a) {
  uint32_t t0 = millis();
  while (digitalRead(a.drdyPin) == HIGH) {
    if (millis() - t0 > DRDY_TIMEOUT_MS) {
      Serial.printf("WARNING: %s DRDY timeout.\n", a.name);
      return false;
    }
  }
  return true;
}

int32_t readData(AdcChannel& a) {
  if (!waitForDRDY(a)) return 0;
  a.bus.beginTransaction(adsSPI);
  csLow(a);
  uint8_t b2 = a.bus.transfer(0x00);
  uint8_t b1 = a.bus.transfer(0x00);
  uint8_t b0 = a.bus.transfer(0x00);
  csHigh(a);
  a.bus.endTransaction();
  int32_t r = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (r & 0x800000) r |= 0xFF000000;
  return r;
}

float averageRaw(AdcChannel& a, uint16_t samples) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) sum += readData(a);
  return (float)sum / (float)samples;
}

void adsInit(AdcChannel& a) {
  sendCmd(a, CMD_RESET);
  delay(50);
  writeReg(a, 0x00, 0x0E);  // AIN0/AIN1, gain=128, PGA on
  writeReg(a, 0x01, 0x04);  // 20 SPS, normal mode, continuous conversion
  writeReg(a, 0x02, 0x70);  // external ref REFP0/REFN0, 50/60Hz rejection
  writeReg(a, 0x03, 0x00);
  sendCmd(a, CMD_START);
  delay(100);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  // Chip selects idle high BEFORE SPI1.begin() so neither chip latches stray clocks
  pinMode(PIN_CS0, OUTPUT); digitalWriteFast(PIN_CS0, HIGH);
  pinMode(PIN_CS1, OUTPUT); digitalWriteFast(PIN_CS1, HIGH);

  // DRDY inputs
  pinMode(PIN_DRDY0, INPUT);
  pinMode(PIN_DRDY1, INPUT);

  // Shared SPI1 bus
  SPI1.setMOSI(PIN_MOSI);
  SPI1.setMISO(PIN_MISO);
  SPI1.setSCK (PIN_SCK);
  SPI1.begin();

  delay(50);

  adsInit(adc0);
  adsInit(adc1);

  zeroOffset0 = averageRaw(adc0, 64);
  zeroOffset1 = averageRaw(adc1, 64);
  Serial.printf("Zero offsets:  ADC0=%.2f   ADC1=%.2f   counts\n",
                zeroOffset0, zeroOffset1);
}

// ---------- Loop ----------
void loop() {
  // Wait until both ADCs have a fresh sample, then print one paired line.
  if (digitalRead(PIN_DRDY0) == LOW && digitalRead(PIN_DRDY1) == LOW) {
    int32_t raw0 = readData(adc0);
    int32_t raw1 = readData(adc1);

    float val0 = (raw0 - zeroOffset0) / countsPerUnit;
    float val1 = (raw1 - zeroOffset1) / countsPerUnit;

    Serial.printf("ADC0 raw=%9ld  %.4f %s   |   ADC1 raw=%9ld  %.4f %s\n",
                  raw0, val0, unitLabel,
                  raw1, val1, unitLabel);
  }
}