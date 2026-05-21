#ifndef TENSIONSENSOR_H
#define TENSIONSENSOR_H

#include <SPI.h>


// ---------- Pins ----------
#define PIN_CS_DIP  38
#define PIN_CS_MCP  10
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

#define NUM_ZERO_SAMPLES 64


#define INTERROR_MAX 100
#define INTERROR_MIN -100


struct finger_tensions_torques {
  float mcp_tension;
  float dip_tension;
  float mcp_torque;
  float dip_torque;
};

struct k{
  float p;
  float i;
  float d;
};

// ---------- SPI helpers ----------
inline void csLow(uint8_t PIN_CS);
inline void csHigh(uint8_t PIN_CS);

void sendCmd(uint8_t cmd, uint8_t PIN_CS);

void writeReg(uint8_t reg, uint8_t val, uint8_t PIN_CS);

void update_sensor_readings(uint8_t PIN_CS, int32_t zeroOffset);
void isr_dip();
void isr_mcp();


// ---------- DRDY wait ----------
bool waitForDRDY();

// ---------- Read one fresh conversion ----------
int32_t readData(uint8_t PIN_CS);

// ---------- Average N fresh conversions ----------
int32_t averageRaw(uint16_t samples,uint8_t PIN_CS);

// ---------- ADS1220 init ----------
void adsInit(uint8_t PIN_CS);

void cs_setup(uint8_t PIN_CS);


void spi_setup();

void zero_sensors();

finger_tensions_torques generate_step_tensions();
float pid_correction(float actual, float desired, float* prev_error, float *i_error, k pid, bool *initial_loop);
float get_dip_tension();
float get_mcp_tension();

#endif