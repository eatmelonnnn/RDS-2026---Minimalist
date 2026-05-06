/*
 * ADS1220 Load Cell Calibration - Teensy 4.1, SPI1
 *
 * Wiring (ATO-LC-ZL107 or similar 4-wire Wheatstone bridge):
 *   Load cell Red   (EXC+)  -> ADS1220 AVDD
 *   Load cell Black (EXC-)  -> ADS1220 AGND
 *   Load cell White (SIG+)  -> ADS1220 AIN0
 *   Load cell Green (SIG-)  -> ADS1220 AIN1
 *
 *   ADS1220 REFP0           -> EXC+ node (ratiometric reference)
 *   ADS1220 REFN0           -> EXC- / AGND node
 *   ADS1220 /DRDY           -> Teensy pin 30
 *
 * Workflow:
 *   1. Upload sketch, open serial monitor at 115200 baud, line ending = "Newline"
 *   2. With NO load applied, type 't' -> tares (records zero offset)
 *   3. Apply a known force/weight and let the reading stabilise
 *   4. Type 'c <known_value>' (e.g. "c 5.0" for 5 kg or 5 N)
 *   5. Repeat steps 3-4 with additional known weights for better linearity
 *   6. Type 'f' to fit and print final calibration constants
 *   7. Type 's' to start streaming calibrated readings
 *   8. Copy the printed constants into your main sketch
 *
 * Commands:
 *   t                 - Tare (set current reading as zero)
 *   c <value>         - Record calibration point at current reading with known value
 *   l                 - List all calibration points
 *   x                 - Clear all calibration points
 *   f                 - Fit linear calibration and print constants
 *   s                 - Start/stop streaming calibrated readings
 *   r                 - Show raw counts (single read, waits for DRDY)
 *   ?                 - Print help
 */

#include <SPI.h>

// ---------- Pin config ----------
constexpr uint8_t PIN_CS   = 38;
constexpr uint8_t PIN_MOSI = 26;
constexpr uint8_t PIN_MISO = 39;
constexpr uint8_t PIN_SCK  = 27;
constexpr uint8_t PIN_DRDY = 29;   // ADS1220 /DRDY -> Teensy pin 29

SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

// ---------- ADS1220 commands ----------
constexpr uint8_t CMD_RESET = 0x06;
constexpr uint8_t CMD_START = 0x08;
constexpr uint8_t CMD_WREG  = 0x40;
constexpr uint8_t CMD_RREG  = 0x20;

// ---------- DRDY timeout ----------
constexpr uint32_t DRDY_TIMEOUT_MS = 500;  // bail out if no conversion in 500 ms

// ---------- Calibration state ----------
constexpr uint8_t MAX_CAL_POINTS = 10;

struct CalPoint {
  float knownValue;   // user-supplied (kg, N, lb, etc.)
  float rawAvg;       // averaged raw counts at that load
};

CalPoint calPoints[MAX_CAL_POINTS];
uint8_t  numCalPoints = 0;

float zeroOffset    = 0.0f;   // raw counts at zero load (intercept)
float countsPerUnit = 1.0f;   // slope: raw counts per user unit
const char* unitLabel = "units";  // change to "kg", "N", "lb", etc.

bool     streaming    = false;
uint32_t lastStreamMs = 0;

// ---------- Low-level SPI helpers ----------
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

// ---------- DRDY-gated data read ----------
// Returns true and fills 'result' when a fresh conversion is available.
// Returns false if DRDY does not assert within DRDY_TIMEOUT_MS.
bool waitForDRDY() {
  uint32_t t0 = millis();
  while (digitalRead(PIN_DRDY) == HIGH) {
    if (millis() - t0 > DRDY_TIMEOUT_MS) {
      Serial.println("WARNING: DRDY timeout - check wiring and ADS1220 power.");
      return false;
    }
  }
  return true;
}

// Reads exactly one fresh 24-bit conversion, sign-extended to int32.
// Returns 0 on timeout (prints a warning).
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
  if (r & 0x800000) r |= 0xFF000000;   // sign-extend 24->32 bit
  return r;
}

// ---------- ADC setup ----------
void adsInit() {
  sendCmd(CMD_RESET);
  delay(50);

  // Register 0: MUX = AIN0/AIN1 (0b0000), Gain = 128 (0b111), PGA enabled (0b0)
  //   Byte: 0b 0000 111 0 = 0x0E
  writeReg(0x00, 0x0E);

  // Register 1: DR = 20 SPS (0b000), Mode = Normal (0b00), CM = Continuous (0b1),
  //             TS = off (0b0), BCS = off (0b0)
  //   Byte: 0b 000 00 1 0 0 = 0x04
  writeReg(0x01, 0x04);

  // Register 2: VREF = REFP0/REFN0 external (0b10), 50/60Hz rejection (0b11),
  //             low-side power switch off (0b0), IDAC = off (0b000)
  //   Byte: 0b 10 11 0 000 = 0x70  <- ratiometric, correct for EXC-tied reference
  writeReg(0x02, 0x70);

  // Register 3: IDAC1/IDAC2 routing not used
  writeReg(0x03, 0x00);

  sendCmd(CMD_START);
  delay(100);   // let the first conversion complete before we start reading
}

// ---------- Verified register dump (debug helper) ----------
void printRegisters() {
  Serial.println("ADS1220 register dump:");
  for (uint8_t i = 0; i < 4; i++) {
    Serial.printf("  REG[%u] = 0x%02X\n", i, readReg(i));
  }
}

// ---------- Sample averaging (DRDY-gated) ----------
// Takes 'samples' fresh conversions and returns their average as a float.
float averageRaw(uint16_t samples) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readData();   // blocks until DRDY asserts for each sample
  }
  return (float)sum / (float)samples;
}

// ---------- Linear least-squares calibration fit ----------
bool fitCalibration() {
  if (numCalPoints < 2) {
    Serial.println("Need at least 2 calibration points to fit (3+ recommended).");
    Serial.println("Tip: the tare point (0, zeroOffset) is always included automatically.");
    return false;
  }

  // Build dataset including the tare point as (known=0, raw=zeroOffset)
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

  // Standard OLS: y = m*x + b  ->  knownValue = (raw - b) / m
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (uint8_t i = 0; i < n; i++) {
    sumX  += xs[i];
    sumY  += ys[i];
    sumXY += xs[i] * ys[i];
    sumX2 += xs[i] * xs[i];
  }
  float denom = (float)n * sumX2 - sumX * sumX;
  if (fabsf(denom) < 1e-9f) {
    Serial.println("Fit failed: degenerate data (all known values identical?).");
    return false;
  }
  float slope     = ((float)n * sumXY - sumX * sumY) / denom;
  float intercept = (sumY - slope * sumX) / (float)n;

  countsPerUnit = slope;
  zeroOffset    = intercept;

  // R² for goodness-of-fit feedback
  float meanY = sumY / (float)n;
  float ssTot = 0, ssRes = 0;
  for (uint8_t i = 0; i < n; i++) {
    float yPred = slope * xs[i] + intercept;
    ssRes += (ys[i] - yPred) * (ys[i] - yPred);
    ssTot += (ys[i] - meanY)  * (ys[i] - meanY);
  }
  float r2 = (ssTot > 0.0f) ? (1.0f - ssRes / ssTot) : 1.0f;

  Serial.println();
  Serial.println("=== CALIBRATION RESULT ===");
  Serial.printf("  Points used:        %u (including tare)\n", n);
  Serial.printf("  Zero offset:        %.2f counts\n", zeroOffset);
  Serial.printf("  Counts per %-6s  %.4f\n", unitLabel, countsPerUnit);
  Serial.printf("  R^2 fit:            %.6f\n", r2);
  if (r2 < 0.9999f) {
    Serial.println("  NOTE: R^2 < 0.9999 - consider re-taring and re-collecting points.");
  }
  Serial.println();
  Serial.println("Copy these constants into your main sketch:");
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
  Serial.println("Sampling 64 readings for stable zero (DRDY-gated)...");
  zeroOffset = averageRaw(64);
  Serial.printf("Zero offset set to %.2f counts.\n\n", zeroOffset);
}

void cmdAddPoint(float knownValue) {
  if (numCalPoints >= MAX_CAL_POINTS) {
    Serial.println("Calibration buffer full. Use 'x' to clear or 'f' to fit.");
    return;
  }
  Serial.printf("Recording point at %.4f %s.\n", knownValue, unitLabel);
  Serial.println("Sampling 64 readings (DRDY-gated)...");
  float avg = averageRaw(64);
  calPoints[numCalPoints].knownValue = knownValue;
  calPoints[numCalPoints].rawAvg     = avg;
  numCalPoints++;
  Serial.printf("Point %u: known=%.4f %s, raw=%.2f counts, delta from zero=%.2f\n\n",
                numCalPoints, knownValue, unitLabel, avg, avg - zeroOffset);
}

void cmdListPoints() {
  Serial.println();
  Serial.println("=== Calibration points ===");
  Serial.printf("  Tare:      known=0.0000 %s, raw=%.2f\n", unitLabel, zeroOffset);
  for (uint8_t i = 0; i < numCalPoints; i++) {
    Serial.printf("  Point %2u:  known=%.4f %s, raw=%.2f  (delta=%.2f)\n",
                  i + 1,
                  calPoints[i].knownValue, unitLabel,
                  calPoints[i].rawAvg,
                  calPoints[i].rawAvg - zeroOffset);
  }
  Serial.println("==========================\n");
}

void cmdClear() {
  numCalPoints = 0;
  zeroOffset   = 0.0f;
  Serial.println("All calibration points cleared. Run 't' to tare again.\n");
}

void cmdHelp() {
  Serial.println();
  Serial.println("=== Commands ===");
  Serial.println("  t          Tare (zero with no load) - do this first");
  Serial.println("  c <value>  Record calibration point with known load");
  Serial.println("             e.g. 'c 5.0' if 5 kg is currently applied");
  Serial.println("  l          List all calibration points recorded so far");
  Serial.println("  x          Clear all calibration points and tare");
  Serial.println("  f          Fit linear calibration, print constants");
  Serial.println("  s          Toggle streaming calibrated readings");
  Serial.println("  r          Single raw count read (DRDY-gated)");
  Serial.println("  d          Dump ADS1220 register values (debug)");
  Serial.println("  ?          Print this help");
  Serial.println("================\n");
}

// ---------- Serial command parser ----------
void processCommand(const String& line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  char   cmd = s.charAt(0);
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
    case 'l': cmdListPoints();  break;
    case 'x': cmdClear();       break;
    case 'f': fitCalibration(); break;
    case 's': {
      streaming = !streaming;
      Serial.println(streaming ? "Streaming ON." : "Streaming OFF.");
      break;
    }
    case 'r': {
      int32_t raw = readData();
      Serial.printf("raw = %ld\n", raw);
      break;
    }
    case 'd': printRegisters(); break;
    case '?':
    case 'h': cmdHelp(); break;
    default:
      Serial.printf("Unknown command: '%c'. Type ? for help.\n", cmd);
      break;
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  delay(200);

  pinMode(PIN_CS,   OUTPUT);
  pinMode(PIN_DRDY, INPUT);   // /DRDY is an output from ADS1220, input to Teensy
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
  Serial.printf( "  Unit label : %s  (edit 'unitLabel' in source)\n", unitLabel);
  Serial.printf( "  DRDY pin   : %u\n", PIN_DRDY);
  Serial.printf( "  CS pin     : %u\n", PIN_CS);
  Serial.println("  Reference  : external ratiometric (REFP0/REFN0)");
  Serial.println("  Gain       : 128x  |  Rate: 20 SPS continuous");
  Serial.println();
  Serial.println("Tip: if DRDY timeouts appear, run 'd' to inspect registers.");
  cmdHelp();
}

// ---------- Loop ----------
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

  // Streaming: only read when DRDY is asserted to avoid busy-polling
  if (streaming && digitalRead(PIN_DRDY) == LOW) {
    int32_t raw   = readData();
    float   value = (raw - zeroOffset) / countsPerUnit;
    Serial.printf("raw=%ld  %.4f %s\n", raw, value, unitLabel);
  }
}
