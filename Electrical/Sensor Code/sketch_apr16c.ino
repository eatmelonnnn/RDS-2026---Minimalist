#include "HX711.h"

#define DT 26
#define SCK 27

HX711 scale;

void setup() {
  Serial.begin(9600);
  scale.begin(DT, SCK);
  
  Serial.println("Finding offset... keep the scale empty!");
  delay(2000); // Give you time to let go of the scale
}

void loop() {
  if (scale.is_ready()) {
    // read_average(20) takes 20 readings and averages them for stability
    long raw_offset = scale.read_average(20); 
    
    Serial.print("Your Offset Number: ");
    Serial.println(raw_offset);
  } else {
    Serial.println("HX711 not found. Check wiring.");
  }
  delay(1000);
}