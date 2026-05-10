#include "HX711.h"
#define SCK 27
#define DT 26
#define CALIBRATION_FACTOR -12238.0
//65100.83

HX711 scale;
// Scale weight reading counter
int count = 1;
 

/* Initialising the teensy & load cell */
void setup() {
  Serial.begin(9600);
  scale.begin(DT, SCK);
  // Calibration values
  scale.set_offset(7600);
  scale.set_scale(CALIBRATION_FACTOR);
  // Set the scale to 0
  scale.tare();
  Serial.println("Please place weight on scales");
}
 

/* Calculating & displaying the weight & average weight readings */
void loop() {
  // Reading scale input
  float weight = scale.get_units();
 

  // Checking for valid weight and updating the average
  //if (weight > 1) {
  Serial.printf("reading %d: ", count++);
  Serial.println(scale.get_units(), 3);
  Serial.println("Please place another weight on scales");
  delay(1000);
  //}
  
  //delay(1000);
}