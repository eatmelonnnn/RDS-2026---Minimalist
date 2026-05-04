#include "HX711.h"
#define SCK 27
#define DT 26
// Preset known weight
// Please update to match your known weight
#define KNOWN_WEIGHT 111.21
HX711 scale;

/* Init teensy & load cell */
void setup() {
  Serial.begin(9600);
  scale.begin(DT, SCK);
}


/* Calculating the calibration factor
  when scale input is available */
void loop() {
  // Checking if there is any scale inputS
  if (scale.is_ready()) {
    // Set & tare the scale
    scale.set_scale();
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    scale.tare();
    Serial.println("Tare done...");
 

    // Collecting & displaying the known weight input
    Serial.print("Place a known weight on the scale...");
    delay(20000);
    long reading = scale.get_units(10);
    Serial.print("Result: ");
    Serial.println(reading);
 

    // Calculating & displaying the calibration factor
    Serial.print("Calibration factor: ");
    Serial.println(reading / KNOWN_WEIGHT);
  }
  else {
    Serial.println("HX711 not found.");
  }
  delay(1000);
}