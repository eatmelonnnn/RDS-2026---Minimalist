/*
 * ADS1220 Load Cell Calibration - Teensy 4.1, SPI1
 * Wiring: E+ -> 3.3V (and AIN0/REFP1), E- -> GND (and AIN3/REFN1),
 *         S+ -> AIN1, S- -> AIN2
 * Ratiometric reference uses REFP1/REFN1 (= AIN0/AIN3 pins)
 *
 * Workflow:
 *   1. Upload, open serial monitor at 115200, line ending "Newline"
 *   2. With NO load applied: type 't' to tare
 *   3. Apply known load, type 'c <value>' (e.g. 'c 5.0')
 *   4. Repeat with different loads (3+ points recommended)
 *   5. Type 'f' to fit and print calibration constants
 *   6. Type 's' to stream calibrated readings
 *   7. Copy printed constants into your main sketch
 *
 * Commands:
 *   t          Tare (zero with no load)
 *   c <value>  Record calibration point at known value
 *   l          List calibration points
 *   x          Clear all calibration points
 *   f          Fit and print constants
 *   s          Toggle streaming
 *   r          Single raw reading
 *   ?          Help
 */

#include <SPI.h>

// ---------- Pin and SPI config ----------
constexpr uint8_t PIN_CS   = 38;
constexpr uint8_t PIN_MOSI = 26;
constexpr uint8_t PIN_MISO = 39;
constexpr uint8_t PIN_SCK  = 27;

SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

// ---------- ADS1220 commands ----------
constexpr uint8_t CMD_RESET = 0x06;
constexpr uint8_t CMD_START = 0x08;
constexpr uint8_t CMD_WREG  = 0x40;
constexpr uint8_t CMD_RREG  = 0x20;

// ---------- Calibration state ----------
constexpr uint8_t MAX_CAL_POINTS = 10;

struct CalPoint {
  float knownValue;
  float rawAvg;
};

CalPoint calPoints[MAX_CAL_POINTS];
uint8_t numCalPoints = 0;

float zeroOffset    = 0.0f;
float countsPerUnit = 1.0f;
const char* unitLabel = "units";  // change to "kg", "N", "lb"

bool streaming = false;
uint32_t lastStreamMs = 0;

// ---------- Low-level helpers ----------
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

uint8_t readReg(uint8_t reg) {
  SPI1.beginTransaction(adsSPI);
  csLow();
  SPI1.transfer(CMD_RREG | (reg << 2));
  uint8_t v = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  delayMicroseconds(20);
  return v;
}

int32_t readData() {
  SPI1.beginTransaction(adsSPI);
  csLow();
  uint8_t b2 = SPI1.transfer(0x00);
  uint8_t b1 = SPI1.transfer(0x00);
  uint8_t b0 = SPI1.transfer(0x00);
  csHigh();
  SPI1.endTransaction();
  int32_t r = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  if (r & 0x800000) r |= 0xFF000000;
  return r;
}

// ---------- ADC setup for load cell ----------
void adsInit() {
  sendCmd(CMD_RESET);
  delay(50);

  // REG0: MUX=0101 (AIN1/AIN2 differential), GAIN=111 (128x), PGA enabled -> 0x5E
  writeReg(0x00, 0x5E);
  // REG1: 20 SPS, normal mode, continuous conversion -> 0x04
  writeReg(0x01, 0x04);
  // REG2: VREF=10 (external REFP1/REFN1 = AIN0/AIN3 pins), 50/60Hz reject -> 0xB0
  writeReg(0x02, 0xB0);
  // REG3: defaults
  writeReg(0x03, 0x00);

  // Verify
  Serial.println("Register settings:");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("  REG%u = 0x%02X\n", i, readReg(i));
  }

  sendCmd(CMD_START);
  delay(100);
}

// ---------- Sample averaging ----------
float averageRaw(uint16_t samples) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    delay(55);  // pace at ~20 SPS
    sum += readData();
  }
  return (float)sum / samples;
}

// ---------- Linear fit (least squares) ----------
bool fitCalibration() {
  if (numCalPoints < 1) {
    Serial.println("Need at least 1 calibration point (plus tare).");
    return false;
  }

  // Include tare point and all calibration points
  float xs[MAX_CAL_POINTS + 1];
  float ys[MAX_CAL_POINTS + 1];
  uint8_t n = 0;

  xs[n] = 0.0f;
  ys[n] = zeroOffset;
  n++;

  for (uint8_t i = 0; i < numCalPoints; i++) {
    xs[n] = calPoints[i].knownValue;
    ys[n] = calPoints[i].rawAvg;
    n++;
  }

  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (uint8_t i = 0; i < n; i++) {
    sumX  += xs[i];
    sumY  += ys[i];
    sumXY += xs[i] * ys[i];
    sumX2 += xs[i] * xs[i];
  }
  float denom = n * sumX2 - sumX * sumX;
  if (fabsf(denom) < 1e-9f) {
    Serial.println("Fit failed: degenerate data.");
    return false;
  }
  float slope     = (n * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / n;

  countsPerUnit = slope;
  zeroOffset    = intercept;

  // R^2
  float meanY = sumY / n;
  float ssTot = 0, ssRes = 0;
  for (uint8_t i = 0; i < n; i++) {
    float yPred = slope * xs[i] + intercept;
    ssRes += (ys[i] - yPred) * (ys[i] - yPred);
    ssTot += (ys[i] - meanY)  * (ys[i] - meanY);
  }
  float r2 = (ssTot > 0.0f) ? (1.0f - ssRes / ssTot) : 1.0f;

  Serial.println();
  Serial.println("=== CALIBRATION RESULT ===");
  Serial.printf("  Points used:    %u\n", n);
  Serial.printf("  Zero offset:    %.2f counts\n", zeroOffset);
  Serial.printf("  Counts per %s:  %.4f\n", unitLabel, countsPerUnit);
  Serial.printf("  R^2 fit:        %.6f\n", r2);
  Serial.println();
  Serial.println("Copy these into your main sketch:");
  Serial.printf("  float zeroOffset    = %.4ff;\n", zeroOffset);
  Serial.printf("  float countsPerUnit = %.4ff;\n", countsPerUnit);
  Serial.println();
  Serial.printf("Conversion: %s = (raw - zeroOffset) / countsPerUnit;\n", unitLabel);
  Serial.println("==========================\n");
  return true;
}

// ---------- Command handlers ----------
void cmdTare() {
  Serial.println("Taring... ensure NO load is applied.");
  Serial.println("Sampling 64 readings...");
  zeroOffset = averageRaw(64);
  Serial.printf("Zero offset: %.2f counts.\n", zeroOffset);
}

void cmdAddPoint(float knownValue) {
  if (numCalPoints >= MAX_CAL_POINTS) {
    Serial.println("Buffer full. Use 'x' or 'f'.");
    return;
  }
  Serial.printf("Recording point at %.4f %s.\n", knownValue, unitLabel);
  Serial.println("Sampling 64 readings...");
  float avg = averageRaw(64);
  calPoints[numCalPoints].knownValue = knownValue;
  calPoints[numCalPoints].rawAvg     = avg;
  numCalPoints++;
  Serial.printf("Point %u: known=%.4f %s, raw=%.2f (delta=%.2f)\n",
                numCalPoints, knownValue, unitLabel, avg, avg - zeroOffset);
}

void cmdListPoints() {
  Serial.println("\n=== Calibration points ===");
  Serial.printf("Tare:  known=0.0000 %s, raw=%.2f\n", unitLabel, zeroOffset);
  for (uint8_t i = 0; i < numCalPoints; i++) {
    Serial.printf("Pt %u:  known=%.4f %s, raw=%.2f (delta=%.2f)\n",
                  i + 1, calPoints[i].knownValue, unitLabel,
                  calPoints[i].rawAvg, calPoints[i].rawAvg - zeroOffset);
  }
  Serial.println("==========================\n");
}

void cmdClear() {
  numCalPoints = 0;
  zeroOffset = 0.0f;
  Serial.println("Cleared. Run 't' to tare.");
}

void cmdHelp() {
  Serial.println("\n=== Commands ===");
  Serial.println("  t          Tare (zero with no load)");
  Serial.println("  c <value>  Record calibration point");
  Serial.println("  l          List points");
  Serial.println("  x          Clear all");
  Serial.println("  f          Fit and print constants");
  Serial.println("  s          Toggle streaming");
  Serial.println("  r          Single raw reading");
  Serial.println("  ?          Help");
  Serial.println("================\n");
}

// ---------- Serial command parser ----------
void processCommand(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  char cmd = s.charAt(0);
  String arg = (s.length() > 1) ? s.substring(1) : String("");
  arg.trim();

  switch (cmd) {
    case 't': cmdTare(); break;
    case 'c':
      if (arg.length() == 0) Serial.println("Usage: c <value>");
      else cmdAddPoint(arg.toFloat());
      break;
    case 'l': cmdListPoints(); break;
    case 'x': cmdClear(); break;
    case 'f': fitCalibration(); break;
    case 's':
      streaming = !streaming;
      Serial.println(streaming ? "Streaming ON." : "Streaming OFF.");
      break;
    case 'r': {
      int32_t r = readData();
      Serial.printf("raw = %ld\n", r);
      break;
    }
    case '?': case 'h': cmdHelp(); break;
    default:
      Serial.printf("Unknown command: '%c'. Type ? for help.\n", cmd);
      break;
  }
}

// ---------- Setup and loop ----------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  delay(200);

  pinMode(PIN_CS, OUTPUT);
  digitalWriteFast(PIN_CS, HIGH);

  SPI1.setMOSI(PIN_MOSI);
  SPI1.setMISO(PIN_MISO);
  SPI1.setSCK(PIN_SCK);
  SPI1.begin();
  delay(50);

  adsInit();

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  ADS1220 Load Cell Calibration");
  Serial.println("  Inputs: AIN1/AIN2, Ref: REFP1/REFN1");
  Serial.println("==========================================");
  Serial.printf("  Unit: %s (edit 'unitLabel' to change)\n", unitLabel);
  cmdHelp();
}

void loop() {
  static String inputBuf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuf.length() > 0) {
        processCommand(inputBuf);
        inputBuf = "";
      }
    } else {
      inputBuf += c;
    }
  }

  if (streaming && (millis() - lastStreamMs >= 100)) {
    lastStreamMs = millis();
    int32_t raw = readData();
    float value = (raw - zeroOffset) / countsPerUnit;
    Serial.printf("raw=%ld  %.4f %s\n", raw, value, unitLabel);
  }
}