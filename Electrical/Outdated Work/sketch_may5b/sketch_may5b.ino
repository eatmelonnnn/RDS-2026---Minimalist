/*
 * ADS1220 Load Cell Calibration - Teensy 4.1, SPI1
 *
 * Workflow:
 *   1. Upload sketch, open serial monitor at 115200 baud, set line ending to "Newline"
 *   2. With NO load applied, type 't' and press enter -> tares (records zero offset)
 *   3. Apply a known force/weight and let the reading stabilize
 *   4. Type 'c <known_value>' and press enter (e.g., "c 5.0" for 5 kg or 5 N)
 *      -> records a calibration point
 *   5. Repeat step 3-4 with different known weights for better linearity
 *   6. Type 'f' to fit and print final calibration constants
 *   7. Type 's' to start streaming calibrated readings
 *   8. Copy the printed constants into your main sketch
 *
 * Commands:
 *   t                 - Tare (set current reading as zero)
 *   c <value>         - Record calibration point at current reading with this known value
 *   l                 - List all calibration points so far
 *   x                 - Clear all calibration points
 *   f                 - Fit linear calibration and print constants
 *   s                 - Start/stop streaming calibrated readings
 *   r                 - Show raw counts (no calibration applied)
 *   ?                 - Print help
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
  float knownValue;   // user-supplied (kg, N, lb, etc.)
  float rawAvg;       // averaged raw counts at that load
};

CalPoint calPoints[MAX_CAL_POINTS];
uint8_t numCalPoints = 0;

float zeroOffset    = 0.0f;     // raw counts at zero load
float countsPerUnit = 1.0f;     // slope: counts per known-value-unit
const char* unitLabel = "units";  // change to "kg", "N", "lb" etc.

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

  writeReg(0x00, 0x5E);  // AIN1/AIN2, gain=128, PGA on
  writeReg(0x01, 0x04);  // 20 SPS, normal mode, continuous conversion
  writeReg(0x02, 0x70);  // EXTERNAL ref REFP0/REFN0, 50/60Hz reject (ratiometric)
                         // Change to 0x30 for internal 2.048V ref if not ratiometric
  writeReg(0x03, 0x00);

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
// Fits: rawAvg = slope * knownValue + intercept
// Then: knownValue = (raw - intercept) / slope
// We treat zeroOffset as the "intercept" and countsPerUnit as the "slope"
bool fitCalibration() {
  if (numCalPoints < 2) {
    Serial.println("Need at least 2 calibration points to fit (3+ is better).");
    Serial.println("Tip: a 'tare' counts as the (0, zeroOffset) point.");
    return false;
  }

  // Build the dataset including the zero/tare point
  float xs[MAX_CAL_POINTS + 1];
  float ys[MAX_CAL_POINTS + 1];
  uint8_t n = 0;

  // Include the tare point (known=0, raw=zeroOffset) if a tare has been done
  if (zeroOffset != 0.0f || true) {
    xs[n] = 0.0f;
    ys[n] = zeroOffset;
    n++;
  }
  for (uint8_t i = 0; i < numCalPoints; i++) {
    xs[n] = calPoints[i].knownValue;
    ys[n] = calPoints[i].rawAvg;
    n++;
  }

  // Standard least-squares linear regression: y = m*x + b
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (uint8_t i = 0; i < n; i++) {
    sumX  += xs[i];
    sumY  += ys[i];
    sumXY += xs[i] * ys[i];
    sumX2 += xs[i] * xs[i];
  }
  float denom = n * sumX2 - sumX * sumX;
  if (fabsf(denom) < 1e-9f) {
    Serial.println("Fit failed: degenerate data (all known values identical?).");
    return false;
  }
  float slope     = (n * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / n;

  countsPerUnit = slope;
  zeroOffset    = intercept;

  // Compute R^2 for the user
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
  Serial.println("Conversion formula:");
  Serial.printf("  %s = (raw - zeroOffset) / countsPerUnit;\n", unitLabel);
  Serial.println("==========================\n");
  return true;
}

// ---------- Command handlers ----------
void cmdTare() {
  Serial.println("Taring... ensure NO load is applied.");
  Serial.println("Sampling 64 readings for stable zero...");
  zeroOffset = averageRaw(64);
  Serial.printf("Zero offset set to %.2f counts.\n", zeroOffset);
}

void cmdAddPoint(float knownValue) {
  if (numCalPoints >= MAX_CAL_POINTS) {
    Serial.println("Calibration buffer full. Use 'x' to clear or 'f' to fit.");
    return;
  }
  Serial.printf("Recording point at %.4f %s.\n", knownValue, unitLabel);
  Serial.println("Sampling 64 readings...");
  float avg = averageRaw(64);
  calPoints[numCalPoints].knownValue = knownValue;
  calPoints[numCalPoints].rawAvg     = avg;
  numCalPoints++;
  Serial.printf("Point %u: known=%.4f %s, raw=%.2f counts (delta from zero: %.2f)\n",
                numCalPoints, knownValue, unitLabel, avg, avg - zeroOffset);
}

void cmdListPoints() {
  Serial.println();
  Serial.println("=== Calibration points ===");
  Serial.printf("Tare point:  known=0.0000 %s, raw=%.2f\n", unitLabel, zeroOffset);
  for (uint8_t i = 0; i < numCalPoints; i++) {
    Serial.printf("Point %u:     known=%.4f %s, raw=%.2f (delta=%.2f)\n",
                  i + 1, calPoints[i].knownValue, unitLabel,
                  calPoints[i].rawAvg, calPoints[i].rawAvg - zeroOffset);
  }
  Serial.println("==========================\n");
}

void cmdClear() {
  numCalPoints = 0;
  zeroOffset = 0.0f;
  Serial.println("All calibration points cleared. Run 't' to tare again.");
}

void cmdHelp() {
  Serial.println();
  Serial.println("=== Commands ===");
  Serial.println("  t          Tare (zero with no load)");
  Serial.println("  c <value>  Record calibration point with known value");
  Serial.println("             e.g. 'c 5.0' if 5 kg/N is currently applied");
  Serial.println("  l          List all calibration points");
  Serial.println("  x          Clear all calibration points");
  Serial.println("  f          Fit linear calibration, print constants");
  Serial.println("  s          Toggle streaming calibrated readings");
  Serial.println("  r          Show raw counts (single read)");
  Serial.println("  ?          Print this help");
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
    case 'c': {
      if (arg.length() == 0) {
        Serial.println("Usage: c <known_value>   e.g. 'c 5.0'");
      } else {
        cmdAddPoint(arg.toFloat());
      }
      break;
    }
    case 'l': cmdListPoints(); break;
    case 'x': cmdClear(); break;
    case 'f': fitCalibration(); break;
    case 's': {
      streaming = !streaming;
      Serial.println(streaming ? "Streaming ON." : "Streaming OFF.");
      break;
    }
    case 'r': {
      int32_t r = readData();
      Serial.printf("raw = %ld\n", r);
      break;
    }
    case '?':
    case 'h': cmdHelp(); break;
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
  Serial.println("  ADS1220 Load Cell Calibration Tool");
  Serial.println("==========================================");
  Serial.printf("  Unit: %s (edit 'unitLabel' in source to change)\n", unitLabel);
  cmdHelp();
}

void loop() {
  // Serial command handler
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

  // Streaming mode: print calibrated readings
  if (streaming && (millis() - lastStreamMs >= 100)) {
    lastStreamMs = millis();
    int32_t raw = readData();
    float value = (raw - zeroOffset) / countsPerUnit;
    Serial.printf("raw=%ld  %.4f %s\n", raw, value, unitLabel);
  }
}