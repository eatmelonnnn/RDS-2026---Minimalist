#ifndef TENSIONSENSOR_H
#define TENSIONSENSOR_H

#include <SPI.h>


// ---------- Pins ----------
#define PIN_CS_DIP  38
#define PIN_MOSI 26
#define PIN_MISO 39
#define PIN_SCK 27
#define PIN_DRDY 29

// ---------- ADS1220 commands ----------
#define CMD_RESET 0x06
#define CMD_START 0x08
#define CMD_WREG 0x40

// ---------- Calibration constants ----------
// Update countsPerUnit if you recalibrate with the calibration tool.
// const float countsPerUnit =  3116.4614f;
// const float countsPerUnit =  2886.4956;
#define countsPerUnit 2418.9988

// ---------- DRDY timeout ----------
#define DRDY_TIMEOUT_MS 500


// ---------- SPI helpers ----------
inline void csLow(uint8_t PIN_CS);
inline void csHigh(uint8_t PIN_CS);

void sendCmd(uint8_t cmd, uint8_t PIN_CS);

void writeReg(uint8_t reg, uint8_t val, uint8_t PIN_CS);

// ---------- DRDY wait ----------
bool waitForDRDY();

// ---------- Read one fresh conversion ----------
int32_t readData(uint8_t PIN_CS);

// ---------- Average N fresh conversions ----------
float averageRaw(uint16_t samples,uint8_t PIN_CS);

// ---------- ADS1220 init ----------
void adsInit(uint8_t PIN_CS);

void cs_setup(uint8_t PIN_CS);


void spi_setup();



#endif