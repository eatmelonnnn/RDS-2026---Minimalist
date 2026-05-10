#include "tensionsensor.h"


SPISettings adsSPI(100'000, MSBFIRST, SPI_MODE1);

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

// ---------- Average N fresh conversions ----------
float averageRaw(uint16_t samples,uint8_t PIN_CS) {
  int64_t sum = 0;
  for (uint16_t i = 0; i < samples; i++) {
    sum += readData(PIN_CS);
  }
  return (float)sum / (float)samples;
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

